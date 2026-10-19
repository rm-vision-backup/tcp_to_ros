#!/usr/bin/env python3
"""GCS_A group task dispatcher: ExecuteGroupTask Action server on the local GCS.

Frozen decisions implemented here:
- Group lifecycle stays on the local ``ExecuteGroupTask.action``.
- Per-UAV task acceptance and HOLD use cross-master Services.
- Per-UAV execution feedback/terminal state uses cross-master Topics.
- Tasks use fully structured ROS fields (no payload_json).
- Unique key is (mission_id, command_id, uav_id).
- Reliability: at-least-once request, idempotent UAV execution, ALL fail-fast.
- Concurrent confirmation across all participating UAVs:
  single wait 1.0s, first try + 2 retries, 0.2s interval,
  group ACCEPTED/FAILED decided within 4.0s.
- Group ACCEPTED is only sent after every participating UAV returns accepted=true.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State as MavrosState
from std_msgs.msg import Bool, Float64

from tcp_to_ros.msg import (
    ExecuteGroupTaskAction,
    ExecuteGroupTaskFeedback,
    ExecuteGroupTaskResult,
)
from tcp_to_ros.srv import (
    EmergencyHold,
    EmergencyHoldResponse,
)
from swarm_uav_interfaces.msg import TaskAssignment, UavLinkHealth, UavTaskState
from swarm_uav_interfaces.srv import (
    UavHold,
    UavHoldRequest,
    UavTask,
    UavTaskControl,
    UavTaskControlRequest,
    UavTaskRequest,
)

from tcp_to_ros.identity import IdentityMap
from tcp_to_ros.link_health import LOST, UavLinkMonitor
from tcp_to_ros.calibration import task_point_to_local_enu, task_vector_to_enu
from tcp_to_ros.time_offset_scheduler import (
    GEOMETRY_UNSUITABLE,
    GLOBAL_SCHEDULE_INFEASIBLE,
    PAIR_SAFE_SET_EMPTY,
    SCHEDULE_LIMIT_REACHED,
    SCHEDULE_OK,
    ScheduleParams,
    ScheduleResult,
    UavSegment,
    compute_schedule,
)

PROTOCOL_VERSION = "1.0"

# Static per-UAV cross-master naming convention (documented in CONTEXT.md / docs):
# task Service  : /{exec_target}/uav_task
# hold Service  : /{exec_target}/uav_hold
# state Topic   : /{exec_target}/uav_task_state
TASK_SERVICE_SUFFIX = "uav_task"
TASK_CONTROL_SERVICE_SUFFIX = "uav_task_control"
HOLD_SERVICE_SUFFIX = "uav_hold"
STATE_TOPIC_SUFFIX = "uav_task_state"
MAVROS_STATE_SUFFIX = "mavros/state"
LOCAL_POSE_SUFFIX = "mavros/local_position/pose"

# Confirm plan constants
DEFAULT_CONFIRM_WAIT_S = 1.0
DEFAULT_CONFIRM_RETRIES = 2
DEFAULT_CONFIRM_RETRY_INTERVAL_S = 0.2
DEFAULT_CONFIRM_DEADLINE_S = 4.0
# Group HOLD fan-out: batch size + retries (0.5s apart) to survive transient
# bridge service forwarding timeouts (26082700: A12 /uav_hold timed out).
DEFAULT_HOLD_BATCH_SIZE = 4
DEFAULT_HOLD_RETRY_COUNT = 2
DEFAULT_HOLD_RETRY_INTERVAL_S = 0.5

# Time-offset scheduling defaults. Canonical values live in backend_defaults.yaml
# (implementation_plan_26090106 §7); these fallbacks mirror the YAML production
# values so a missing rosparam cannot silently drift (26082916 §7 convention).
DEFAULT_HORIZONTAL_SPEED_MPS = 0.7
DEFAULT_SCHEDULE_GEOMETRIC_DISTANCE_M = 1.3
DEFAULT_SCHEDULE_SAFETY_MARGIN_M = 1.0
DEFAULT_SCHEDULE_POSE_TIMEOUT_S = 2.0  # 2026-10-19 与 backend_defaults.yaml 同步放宽（原 1.0）
DEFAULT_SCHEDULE_TIME_EPSILON_S = 1e-6
DEFAULT_SCHEDULE_MAX_START_DELAY_S = 30.0
DEFAULT_SCHEDULE_MAX_INTERVAL_COMBINATIONS = 2000
DEFAULT_SCHEDULE_MAX_COMPUTE_TIME_S = 5.0
DEFAULT_START_SERVICE_BUDGET_S = 1.5
DEFAULT_START_DISPATCH_JITTER_MARGIN_S = 0.5

# Schedule-specific failure codes returned to runtime (plan 6.2 allows new
# schedule-specific error codes). schedule_status is log-only (plan 6.1).
SCHEDULE_POSE_UNAVAILABLE = "SCHEDULE_POSE_UNAVAILABLE"
SCHEDULE_GEOMETRY_UNSUITABLE = "SCHEDULE_GEOMETRY_UNSUITABLE"
SCHEDULE_PAIR_SAFE_SET_EMPTY = "SCHEDULE_PAIR_SAFE_SET_EMPTY"
SCHEDULE_INFEASIBLE = "SCHEDULE_INFEASIBLE"
SCHEDULE_LIMIT_REACHED = "SCHEDULE_LIMIT_REACHED"
SCHEDULE_TIMEOUT_INSUFFICIENT = "SCHEDULE_TIMEOUT_INSUFFICIENT"

STATE_ACCEPTED = "ACCEPTED"
STATE_COMPLETED = "COMPLETED"
STATE_FAILED = "FAILED"

# implementation_plan_26090900：结构化安全终态白名单（error_code 精确匹配，禁止
# 按 message 文本/任意 FAILED 模糊触发）。BRAKE_HOLD 为 C++ 直锁终态；
# MIN_DISTANCE_BREACH 为 executor 实时 3D 兜底（§0，仍属白名单）；
# EMERGENCY_BRAKE 保留兼容（旧版本可能残留，新实现不再产生）。
GROUP_SAFETY_ERROR_CODES = frozenset(
    {"BRAKE_HOLD", "MIN_DISTANCE_BREACH", "EMERGENCY_BRAKE"}
)
# runtime 指定 HOLD / 组级安全 HOLD 的机载 reason（executor→driver 据此路由
# /brake_hold 直接锁存；详见 plans/implementation_plan_26090900 §4.1/§4.3）。
RUNTIME_BRAKE_HOLD_REASON = "RUNTIME_BRAKE_HOLD"
GROUP_BRAKE_HOLD_REASON_PREFIX = "GROUP_BRAKE_HOLD"


@dataclass
class SingleUavConfirm:
    uav_id: str
    exec_target: str
    accepted: Optional[bool] = None
    status: str = ""
    error_code: str = ""
    message: str = ""


@dataclass
class _CallOutcome:
    response: object = None
    exception: Optional[Exception] = None


@dataclass
class GroupState:
    """Mutable per-goal aggregation state, guarded by ``lock``."""

    mission_id: str
    command_id: str
    command: str
    uav_ids: List[str]
    exec_by_uav: Dict[str, str]
    start_time: float
    start_monotonic: float = 0.0

    lock: threading.RLock = field(default_factory=threading.RLock)
    stop_event: threading.Event = field(default_factory=threading.Event)

    accepted: bool = False
    accepted_sent: bool = False
    # UAVs that returned accepted=true and therefore own an action which may
    # require compensation even before group ACCEPTED is formed.
    accepted_uavs: List[str] = field(default_factory=list)
    started_uavs: List[str] = field(default_factory=list)
    terminal: Dict[str, str] = field(default_factory=dict)  # uav_id -> COMPLETED/FAILED
    latest_states: Dict[str, UavTaskState] = field(default_factory=dict)
    seen_seq: Dict[str, int] = field(default_factory=dict)  # uav_id -> max status_seq
    pre_accept: List[UavTaskState] = field(default_factory=list)  # terminal states arriving before group ACCEPTED
    failed_uavs: List[str] = field(default_factory=list)
    link_lost_uavs: List[str] = field(default_factory=list)
    hold_pending: List[str] = field(default_factory=list)
    held: List[str] = field(default_factory=list)
    hold_rejected: List[str] = field(default_factory=list)
    error_code: str = ""
    message: str = ""
    # Time-offset scheduling results (plan 5.3/5.4). schedule_tau is the shifted
    # non-negative tau_i' keyed by task-layer uav_id; schedule_status is log-only.
    schedule_tau: Dict[str, float] = field(default_factory=dict)
    schedule_status: str = ""
    # 组级安全 fan-out 一次性防重（implementation_plan_26090900 §4.3）：任一参与
    # UAV 上报白名单安全终态后只触发一次对同任务其他参与者的 UavHold。
    group_safety_hold_started: bool = False
    group_safety_trigger_uav: str = ""
    group_safety_error_code: str = ""

    @property
    def key(self) -> str:
        return f"{self.mission_id}|{self.command_id}"

    def terminal_count(self) -> int:
        return len(self.terminal)


def default_identity_map_path() -> str:
    """Resolve the packaged identity map without a workspace-absolute path.

    implementation_plan_26091101 §7: prefer the ROS package path; fall back to
    the source-tree layout when ``rospkg`` is unavailable (plain unit tests).
    """
    import os

    try:
        import rospkg

        return os.path.join(rospkg.RosPack().get_path("tcp_to_ros"),
                            "config", "uav_identity_map.yaml")
    except Exception:
        return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            os.pardir, "config", "uav_identity_map.yaml")


class GcsABackendNode:
    def __init__(self) -> None:
        self.action_name = rospy.get_param("~action_name", "/group_a/execute_task")
        self.emergency_service_name = rospy.get_param(
            "~emergency_hold_service",
            "/group_a/emergency_hold",
        )
        self.identity_map_path = rospy.get_param(
            "~map_file",
            default_identity_map_path(),
        )

        self.confirm_wait_s = float(
            rospy.get_param("~confirm_wait_s", DEFAULT_CONFIRM_WAIT_S)
        )
        self.confirm_retries = int(
            rospy.get_param("~confirm_retries", DEFAULT_CONFIRM_RETRIES)
        )
        self.confirm_retry_interval_s = float(
            rospy.get_param("~confirm_retry_interval_s", DEFAULT_CONFIRM_RETRY_INTERVAL_S)
        )
        self.confirm_deadline_s = float(
            rospy.get_param("~confirm_deadline_s", DEFAULT_CONFIRM_DEADLINE_S)
        )
        self.hold_batch_size = int(
            rospy.get_param("~hold_batch_size", DEFAULT_HOLD_BATCH_SIZE)
        )
        self.hold_retry_count = int(
            rospy.get_param("~hold_retry_count", DEFAULT_HOLD_RETRY_COUNT)
        )
        self.hold_retry_interval_s = float(
            rospy.get_param("~hold_retry_interval_s", DEFAULT_HOLD_RETRY_INTERVAL_S)
        )

        # Time-offset scheduling parameters (implementation_plan_26090106 §7).
        # Canonical values come from backend_defaults.yaml via the launch
        # rosparam load; the DEFAULT_* fallbacks mirror the YAML production
        # values (26082916 §7 convention) and validate() fails fast on invalid
        # YAML instead of silently running with a broken schedule.
        self.move_to_schedule_enabled = bool(
            rospy.get_param("~move_to_schedule_enabled", True))
        self.fault_exit_schedule_enabled = bool(
            rospy.get_param("~fault_exit_schedule_enabled", True))
        self.schedule_params = ScheduleParams(
            horizontal_speed_mps=float(rospy.get_param(
                "~horizontal_speed_mps", DEFAULT_HORIZONTAL_SPEED_MPS)),
            geometric_distance_m=float(rospy.get_param(
                "~schedule_geometric_distance_m", DEFAULT_SCHEDULE_GEOMETRIC_DISTANCE_M)),
            safety_margin_m=float(rospy.get_param(
                "~schedule_safety_margin_m", DEFAULT_SCHEDULE_SAFETY_MARGIN_M)),
            time_epsilon_s=float(rospy.get_param(
                "~schedule_time_epsilon_s", DEFAULT_SCHEDULE_TIME_EPSILON_S)),
            max_start_delay_s=float(rospy.get_param(
                "~schedule_max_start_delay_s", DEFAULT_SCHEDULE_MAX_START_DELAY_S)),
            max_interval_combinations=int(rospy.get_param(
                "~schedule_max_interval_combinations",
                DEFAULT_SCHEDULE_MAX_INTERVAL_COMBINATIONS)),
            max_compute_time_s=float(rospy.get_param(
                "~schedule_max_compute_time_s", DEFAULT_SCHEDULE_MAX_COMPUTE_TIME_S)),
        )
        self.schedule_params.validate()
        self.schedule_pose_timeout_s = float(rospy.get_param(
            "~schedule_pose_timeout_s", DEFAULT_SCHEDULE_POSE_TIMEOUT_S))
        self.start_service_budget_s = float(rospy.get_param(
            "~start_service_budget_s", DEFAULT_START_SERVICE_BUDGET_S))
        self.start_dispatch_jitter_margin_s = float(rospy.get_param(
            "~start_dispatch_jitter_margin_s", DEFAULT_START_DISPATCH_JITTER_MARGIN_S))
        for name, value in (("schedule_pose_timeout_s", self.schedule_pose_timeout_s),
                            ("start_service_budget_s", self.start_service_budget_s),
                            ("start_dispatch_jitter_margin_s",
                             self.start_dispatch_jitter_margin_s)):
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError("%s must be a positive finite number" % name)
        # Bridge-forwarded current pose cache: exec_target -> {x, y, recv_monotonic}.
        self._pose_cache: Dict[str, dict] = {}
        self._pose_lock = threading.Lock()

        self.identity_map = IdentityMap.load(self.identity_map_path)
        (self.formation_offsets, self.formation_participants,
         self.formation_leader, self.mission_anchor) = self._load_formation_offsets(
            rospy.get_param("~formation_offsets_file", ""))
        self.require_calibration_ready = bool(rospy.get_param("~require_calibration_ready", True))
        self.calibration_ready = not self.require_calibration_ready
        self.reference_heading = 0.0
        enabled = [(item.uav_id, item.exec_target) for item in self.identity_map.uavs if item.enabled]
        self.link_monitor = UavLinkMonitor(
            enabled,
            # fallback 与 backend_defaults.yaml 生产值一致（10.0/15.0/3.0），
            # 防止缺失 YAML 时漂移（implementation_plan_26082916 §7）。
            float(rospy.get_param("~link_suspect_after_s", 10.0)),
            float(rospy.get_param("~link_lost_after_s", 15.0)),
            float(rospy.get_param("~link_healthy_window_s", 3.0)),
        )
        self.link_health_publisher = rospy.Publisher(
            rospy.get_param("~link_health_topic", "/group_a/uav_link_health"),
            UavLinkHealth, queue_size=100, latch=True)
        self.link_health_pre_task_wait_s = float(
            rospy.get_param("~link_health_pre_task_wait_s", 10.0))
        self.server = actionlib.SimpleActionServer(
            self.action_name,
            ExecuteGroupTaskAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self.emergency_hold_service = rospy.Service(
            self.emergency_service_name, EmergencyHold, self._handle_emergency_hold
        )

        self._active: Optional[GroupState] = None
        self._active_lock = threading.RLock()

        self._subscribe_uav_states()
        rospy.Subscriber("/group_a/calibration_ready", Bool,
                         lambda msg: setattr(self, "calibration_ready", bool(msg.data)), queue_size=1)
        rospy.Subscriber("/group_a/reference_heading", Float64,
                         lambda msg: setattr(self, "reference_heading", float(msg.data)), queue_size=1)
        self._safety_stop = threading.Event()
        self._safety_thread = threading.Thread(target=self._safety_loop, daemon=True)
        self._safety_thread.start()
        rospy.on_shutdown(self.shutdown)
        self.server.start()
        rospy.loginfo(
            "GCS_A backend ready: action=%s service=%s uavs=%d",
            self.action_name,
            self.emergency_service_name,
            len(self.identity_map.uavs),
        )

    def _subscribe_uav_states(self) -> None:
        """Subscribe to every enabled UAV's state topic.

        In multi-master simulation the topic is relayed by swarm_topology_bridge
        under /{exec_target}/{STATE_TOPIC_SUFFIX}.
        """
        seen: set = set()
        for ident in self.identity_map.uavs:
            if not ident.enabled or ident.exec_target in seen:
                continue
            seen.add(ident.exec_target)
            topic = "/{}/{}".format(ident.exec_target, STATE_TOPIC_SUFFIX)
            rospy.Subscriber(
                topic, UavTaskState, self._on_uav_state, queue_size=50
            )
            rospy.loginfo("GCS_A subscribed to %s", topic)
            mavros_topic = "/{}/{}".format(ident.exec_target, MAVROS_STATE_SUFFIX)
            rospy.Subscriber(mavros_topic, MavrosState, self._on_mavros_state,
                             callback_args=(ident.uav_id, ident.exec_target), queue_size=10)
            rospy.loginfo("GCS_A subscribed to %s", mavros_topic)
            pose_topic = "/{}/{}".format(ident.exec_target, LOCAL_POSE_SUFFIX)
            rospy.Subscriber(pose_topic, PoseStamped, self._on_local_pose,
                             callback_args=ident.exec_target, queue_size=10)
            rospy.loginfo("GCS_A subscribed to %s", pose_topic)

    def _on_local_pose(self, msg: PoseStamped, exec_target: str) -> None:
        """Cache the latest bridge-forwarded local pose for scheduling (plan 5.2).

        Only the horizontal (x, y) is kept; the scheduler never reads z. The
        received monotonic time drives freshness checking against
        schedule_pose_timeout_s.
        """
        with self._pose_lock:
            self._pose_cache[exec_target] = {
                "x": float(msg.pose.position.x),
                "y": float(msg.pose.position.y),
                "recv_monotonic": time.monotonic(),
            }


    @staticmethod
    def _load_formation_offsets(path: str):
        """Load GCS_A-side formation slot configuration.

        Format (task-layer A-ids): leader_id, mission_anchor and named
        formations with exact participants and per-aircraft offsets.
        """
        if not path:
            return {}, {}, "A01", (0.0, 0.0, 0.0)
        import yaml
        try:
            with open(path, "r", encoding="utf-8") as handle:
                raw = yaml.safe_load(handle) or {}
            offsets = {}
            participants = {}
            for name, config in (raw.get("formations", {}) or {}).items():
                participants[str(name)] = frozenset(str(v) for v in config.get("participants", []))
                offsets[str(name)] = {
                    str(uav): tuple(float(v) for v in value)
                    for uav, value in (config.get("offsets", {}) or {}).items()
                }
            # mission_anchor 必须为 [0,0,0]：任务原点（右/前 0,0）与 EKF 原点对齐，
            # EKF 原点由标定冻结为 A01-A06 全球位置中位数（set_gp_origin）。
            anchor = tuple(float(v) for v in raw.get("mission_anchor", [0.0, 0.0, 0.0]))
            return offsets, participants, str(raw.get("leader_id", "A01")), anchor
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn("failed to load formation_offsets from %s: %s", path, exc)
            return {}, {}, "A01", (0.0, 0.0, 0.0)

    def _on_mavros_state(self, msg, identity) -> None:
        self.link_monitor.record_state(identity[0], identity[1], bool(msg.connected))

    def _safety_loop(self) -> None:
        while not self._safety_stop.wait(0.1):
            self._evaluate_link_health()

    def _evaluate_link_health(self) -> None:
        transitions = self.link_monitor.evaluate()
        for transition in transitions:
            snapshot = self.link_monitor.snapshot(
                transition.uav_id, rospy.Time.now().to_sec(), "", "")
            self.link_health_publisher.publish(snapshot)
            rospy.logwarn("UAV link %s/%s %s -> %s age=%.3f reason=%s",
                          transition.uav_id, transition.exec_target,
                          transition.previous, transition.current,
                          transition.age_s, transition.reason)
            if transition.current == LOST:
                self._inject_link_lost(transition.uav_id)

    def _inject_link_lost(self, uav_id: str) -> None:
        with self._active_lock:
            state = self._active
        if state is None or uav_id not in state.exec_by_uav:
            return
        with state.lock:
            if uav_id not in state.link_lost_uavs: state.link_lost_uavs.append(uav_id)
            if uav_id not in state.failed_uavs: state.failed_uavs.append(uav_id)
            state.error_code = "UAV_LINK_LOST"
            state.message = "UAV link lost: %s" % uav_id
            state.stop_event.set()

    def _translate_onboard_uav_id(self, msg: UavTaskState, state: GroupState):
        """Map an onboard-reported exec-layer UAVn id back to the task-layer
        A-id this group aggregates on. Returns a rewritten message or None."""
        import copy as _copy
        if msg.uav_id in state.exec_by_uav:
            return msg
        task_uav = None
        for cand, exec_target in state.exec_by_uav.items():
            if exec_target == (msg.exec_target or msg.uav_id):
                task_uav = cand
                break
        if task_uav is None:
            return None
        rewritten = _copy.deepcopy(msg)
        rewritten.uav_id = task_uav
        return rewritten

    def _on_uav_state(self, msg: UavTaskState) -> None:
        with self._active_lock:
            state = self._active
        if state is None:
            return
        if msg.command_id != state.command_id or msg.mission_id != state.mission_id:
            return
        translated = self._translate_onboard_uav_id(msg, state)
        if translated is None:
            rospy.logwarn_throttle(
                5.0,
                "ignored task state from non-participant or wrong identity: %s/%s",
                msg.uav_id,
                msg.exec_target,
            )
            return
        msg = translated
        if (
            msg.uav_id not in state.exec_by_uav
            or msg.exec_target != state.exec_by_uav[msg.uav_id]
        ):
            rospy.logwarn_throttle(
                5.0,
                "ignored task state from non-participant or wrong identity: %s/%s",
                msg.uav_id,
                msg.exec_target,
            )
            return
        with state.lock:
            if state.stop_event.is_set():
                return
            if not state.accepted_sent:
                # Terminal/final states may arrive before the group ACCEPTED
                # is formed (e.g. idempotent duplicate with a cached final
                # state). Buffer them and replay once ACCEPTED is set.
                if str(msg.status or "").upper() in {STATE_COMPLETED, STATE_FAILED}:
                    state.pre_accept.append(msg)
                return
            self._absorb_state_locked(state, msg)
            if state.terminal_count() >= len(state.uav_ids):
                state.stop_event.set()

    def _execute(self, goal) -> None:
        uav_ids = [item.uav_id for item in goal.assignments]
        if not uav_ids:
            result = self._build_result(
                final_status="FAILED",
                failed_uavs=[],
                error_code="EMPTY_ASSIGNMENTS",
                message="goal has no assignments",
            )
            self.server.set_aborted(result, text=result.message)
            return

        if self.require_calibration_ready and not self.calibration_ready:
            result = self._build_result(
                final_status="FAILED", failed_uavs=uav_ids,
                error_code="CALIBRATION_NOT_READY",
                message="global origin/reference heading calibration is not ready")
            self.server.set_aborted(result, text=result.message)
            return

        formation_name = self._formation_for_goal(goal)
        if str(goal.command or "") == "FOLLOW_ROUTE" and formation_name is None:
            result = self._build_result(
                final_status="FAILED", failed_uavs=uav_ids,
                error_code="FORMATION_PARTICIPANTS_MISMATCH",
                message="FOLLOW_ROUTE participants do not match a configured fixed formation")
            self.server.set_aborted(result, text=result.message)
            return
        if (str(goal.command or "") == "FOLLOW_ROUTE" and
                str(goal.leader_id or "") != self.formation_leader):
            result = self._build_result(
                final_status="FAILED", failed_uavs=uav_ids,
                error_code="INVALID_FORMATION_LEADER",
                message="FOLLOW_ROUTE leader must be %s" % self.formation_leader)
            self.server.set_aborted(result, text=result.message)
            return

        try:
            resolved = self.identity_map.resolve(uav_ids)
        except KeyError as exc:
            result = self._build_result(
                final_status="FAILED",
                failed_uavs=uav_ids,
                error_code="UNKNOWN_UAV_ID",
                message=str(exc),
            )
            self.server.set_aborted(result, text=result.message)
            return

        with self._active_lock:
            if self._active is not None:
                result = self._build_result(
                    final_status="FAILED",
                    failed_uavs=uav_ids,
                    error_code="COMMAND_ALREADY_ACTIVE",
                    message="another group command is already active",
                )
                self.server.set_aborted(result, text=result.message)
                return
            state = GroupState(
                mission_id=goal.mission_id,
                command_id=goal.command_id,
                command=str(goal.command or ""),
                uav_ids=uav_ids,
                exec_by_uav={uav_id: ident.exec_target for uav_id, ident in resolved},
                start_time=rospy.Time.now().to_sec(),
                start_monotonic=time.monotonic(),
            )
            self._active = state

        unhealthy = [uav_id for uav_id in state.uav_ids
                     if not self.link_monitor.is_healthy_or_undiscovered(uav_id)]
        if unhealthy:
            with self._active_lock:
                self._active = None
            result = self._build_result("FAILED", failed_uavs=unhealthy,
                error_code="UAV_LINK_UNHEALTHY",
                message="participating UAV link is not stably healthy: %s" % ",".join(unhealthy))
            self.server.set_aborted(result, text=result.message)
            return

        rospy.loginfo(
            "GCS_A goal %s/%s command=%s uavs=%s",
            goal.mission_id,
            goal.command_id,
            goal.command,
            ",".join(uav_ids),
        )

        try:
            self._run_goal(goal, state)
        finally:
            with self._active_lock:
                if self._active is state:
                    self._active = None

    def _schedule_applies(self, command: str) -> bool:
        """True when this batch must run the t_off scheduler (plan 5.3)."""
        return (
            (str(command or "") == "MOVE_TO" and self.move_to_schedule_enabled)
            or (str(command or "") == "FAULT_EXIT" and self.fault_exit_schedule_enabled)
        )

    def _pose_snapshot(self, exec_targets: List[str]):
        """Consistent horizontal pose snapshot with freshness gating (plan 5.2)."""
        now = time.monotonic()
        snapshot: Dict[str, tuple] = {}
        missing: List[str] = []
        with self._pose_lock:
            for exec_target in exec_targets:
                entry = self._pose_cache.get(exec_target)
                age = None if entry is None else now - entry["recv_monotonic"]
                if entry is None or age > self.schedule_pose_timeout_s:
                    missing.append(exec_target)
                else:
                    snapshot[exec_target] = (entry["x"], entry["y"])
        if missing:
            rospy.logwarn("GCS_A schedule pose unavailable/stale for %s", ",".join(missing))
        return snapshot, missing

    def _build_schedule_segments(self, goal, state: GroupState):
        """Build scheduler input: bridge pose start + task goal translated to local
        ENU (plan 4.1/4.2). MOVE_TO uses target_pose, FAULT_EXIT the final
        waypoint. Returns (segments, fail_ids); None segments means pre-check
        failed (missing pose or assignment)."""
        exec_targets = [state.exec_by_uav[uav] for uav in state.uav_ids]
        snapshot, missing = self._pose_snapshot(exec_targets)
        if missing:
            fail_ids = [uav for uav in state.uav_ids
                        if state.exec_by_uav[uav] in missing]
            return None, fail_ids
        now = time.monotonic()
        ages = {}
        with self._pose_lock:
            for exec_target in exec_targets:
                ages[exec_target] = now - self._pose_cache[exec_target]["recv_monotonic"]
        segments = []
        for uav_id in state.uav_ids:
            exec_target = state.exec_by_uav[uav_id]
            x0, y0 = snapshot[exec_target]
            assignment = None
            for item in goal.assignments:
                if item.uav_id == uav_id:
                    assignment = item
                    break
            if assignment is None:
                return None, [uav_id]
            if str(state.command) == "MOVE_TO":
                goal_pose = assignment.target_pose
            else:  # FAULT_EXIT: final exit waypoint
                if not assignment.waypoints:
                    return None, [uav_id]
                goal_pose = assignment.waypoints[-1]
            gx, gy, _gz = task_point_to_local_enu(
                goal_pose.x, goal_pose.y, goal_pose.z,
                self.reference_heading, self.mission_anchor)
            segments.append(UavSegment(
                uav_id=uav_id, start_xy=(x0, y0), goal_xy=(gx, gy)))
            rospy.loginfo(
                "GCS_A schedule input %s start=(%.3f, %.3f) goal=(%.3f, %.3f) pose_age=%.3fs",
                uav_id, x0, y0, gx, gy, ages[exec_target])
        return segments, []

    def _compute_and_store_schedule(self, goal, state: GroupState) -> bool:
        """Run the pre-PREPARE schedule (plan 5.3 step 5/6). On failure it does not
        call PREPARE and returns a FAILED result with the related fail ids."""
        segments, fail_ids = self._build_schedule_segments(goal, state)
        if segments is None:
            result = ScheduleResult(
                success=False, schedule_status=SCHEDULE_OK, fail_ids=fail_ids,
                failure_reason="pose snapshot unavailable or stale")
            self._fail_schedule(goal, state, result, SCHEDULE_POSE_UNAVAILABLE)
            return False
        schedule = compute_schedule(segments, self.schedule_params)
        if not schedule.success:
            code = {
                GEOMETRY_UNSUITABLE: SCHEDULE_GEOMETRY_UNSUITABLE,
                PAIR_SAFE_SET_EMPTY: SCHEDULE_PAIR_SAFE_SET_EMPTY,
                GLOBAL_SCHEDULE_INFEASIBLE: SCHEDULE_INFEASIBLE,
                SCHEDULE_LIMIT_REACHED: SCHEDULE_LIMIT_REACHED,
            }[schedule.schedule_status]
            self._fail_schedule(goal, state, schedule, code)
            return False
        # plan 5.4: task timeout_s must cover PREPARE-after wait + execution.
        needed = (schedule.max_tau_s() + schedule.max_horizontal_time_s
                  + self.start_service_budget_s
                  + self.start_dispatch_jitter_margin_s)
        if needed > float(goal.timeout_s):
            self._log_schedule(goal, state, schedule)
            rospy.logwarn(
                "GCS_A schedule timeout insufficient: needed=%.3f timeout=%.3f",
                needed, float(goal.timeout_s))
            result = self._build_result(
                final_status="FAILED", failed_uavs=list(state.uav_ids),
                error_code=SCHEDULE_TIMEOUT_INSUFFICIENT,
                message=("task timeout_s %.3f insufficient for scheduled start"
                         % float(goal.timeout_s)))
            self._publish_feedback(state, phase="FAILED", detail=result.message)
            self.server.set_aborted(result, text=result.message)
            return False
        with state.lock:
            state.schedule_tau = dict(zip(state.uav_ids, schedule.tau))
            state.schedule_status = schedule.schedule_status
        self._log_schedule(goal, state, schedule)
        return True

    def _fail_schedule(self, goal, state: GroupState, schedule, error_code: str) -> None:
        """Plan 6.2: pre-schedule failure returns FAILED + fail ids; no task is
        sent to any UAV."""
        self._log_schedule(goal, state, schedule)
        failed_uavs = schedule.fail_ids or list(state.uav_ids)
        result = self._build_result(
            final_status="FAILED", failed_uavs=failed_uavs, error_code=error_code,
            message=schedule.failure_reason or "schedule pre-check failed")
        self._publish_feedback(state, phase="FAILED", detail=result.message)
        self.server.set_aborted(result, text=result.message)

    def _log_schedule(self, goal, state: GroupState, schedule, status: Optional[str] = None) -> None:
        """schedule_status / tau / pair diagnostics only into tcp_to_ros logs
        (plan 6.1), never into runtime or onboard messages."""
        log_status = status or schedule.schedule_status
        rospy.loginfo(
            "GCS_A schedule mission=%s command_id=%s command=%s status=%s",
            state.mission_id, state.command_id, state.command, log_status)
        if schedule.fail_ids:
            rospy.logwarn("GCS_A schedule fail_ids=%s reason=%s",
                          ",".join(schedule.fail_ids), schedule.failure_reason)
        if schedule.tau:
            tau_s = ", ".join("%s=%.3f" % (uav, t)
                               for uav, t in zip(state.uav_ids, schedule.tau))
            rospy.loginfo(
                "GCS_A schedule tau {%s} variance=%.4f compute=%.3fs non_global=%s",
                tau_s, schedule.variance, schedule.compute_time_s,
                schedule.non_global_optimal)
        for pair in schedule.pair_results:
            dmin = pair.predicted_min_distance_m
            rospy.loginfo(
                "GCS_A schedule pair %s/%s safe_intervals=%d predicted_dmin=%s",
                pair.uav_i, pair.uav_j, len(pair.safe_intervals),
                "inf" if dmin is None else "%.3f" % dmin)

    def _dispatch_start_scheduled(self, state: GroupState) -> List[SingleUavConfirm]:
        """START fan-out scheduled by shifted tau_i' (plan 5.4).

        t0 is rebuilt now, after every PREPARE returned accepted. Each UAV gets
        an independent worker that waits until t0 + tau_i' and then calls the
        existing UavTaskControl START within the start_service_budget_s budget;
        the group deadline is max(tau_i') + budget + jitter margin.
        """
        t0 = time.monotonic()
        max_tau = max(state.schedule_tau.values()) if state.schedule_tau else 0.0
        group_deadline = (t0 + max_tau + self.start_service_budget_s
                          + self.start_dispatch_jitter_margin_s)
        results: Dict[str, SingleUavConfirm] = {
            uav: SingleUavConfirm(uav, target)
            for uav, target in state.exec_by_uav.items()
        }

        def worker(uav_id: str, exec_target: str) -> None:
            target = t0 + state.schedule_tau.get(uav_id, 0.0)
            while not rospy.is_shutdown() and time.monotonic() < target:
                time.sleep(min(0.02, max(0.0, target - time.monotonic())))
            plan_sent = time.monotonic()
            rospy.loginfo(
                "GCS_A START dispatch %s target=%.3f actual=%.3f deviation=%.3f",
                uav_id, target, plan_sent, plan_sent - target)
            request = UavTaskControlRequest()
            request.protocol_version = PROTOCOL_VERSION
            request.operation = "START"
            request.mission_id = state.mission_id
            request.command_id = state.command_id
            request.uav_id = exec_target
            request.exec_target = exec_target
            request.reason = ""
            deadline = plan_sent + self.start_service_budget_s
            for attempt in range(self.confirm_retries + 1):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                proxy = rospy.ServiceProxy(
                    "/{}/{}".format(exec_target, TASK_CONTROL_SERVICE_SUFFIX),
                    UavTaskControl)
                outcome = self._call_service_with_timeout(
                    proxy, request, min(self.confirm_wait_s, remaining))
                if outcome.exception is None:
                    response = outcome.response
                    results[uav_id].accepted = bool(response.accepted)
                    results[uav_id].status = str(response.status or "")
                    results[uav_id].error_code = str(response.error_code or "")
                    results[uav_id].message = str(response.message or "")
                    if response.accepted:
                        with state.lock:
                            if uav_id not in state.started_uavs:
                                state.started_uavs.append(uav_id)
                    return
                results[uav_id].message = str(outcome.exception)
                if attempt < self.confirm_retries:
                    time.sleep(min(self.confirm_retry_interval_s,
                                   max(0.0, deadline - time.monotonic())))
            if results[uav_id].accepted is None and not results[uav_id].message:
                results[uav_id].message = "START dispatch deadline expired"

        threads = [threading.Thread(target=worker, args=item, daemon=True)
                   for item in state.exec_by_uav.items()]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(max(0.0, group_deadline - time.monotonic()))
        return [results[uav] for uav in state.uav_ids]

    def _run_goal(self, goal, state: GroupState) -> None:
        if self._schedule_applies(state.command):
            if not self._compute_and_store_schedule(goal, state):
                return

        confirm_results = self._confirm_participants(goal, state)

        rejections = [item for item in confirm_results if item.accepted is False]
        unresolved = [item for item in confirm_results if item.accepted is None]

        if rejections or unresolved:
            # Fail fast: group ACCEPTED must never be sent until every UAV confirmed.
            failed_uavs = [item.uav_id for item in rejections + unresolved]
            if not state.message:
                if rejections:
                    first = rejections[0]
                    state.message = (
                        f"UAV rejected task: {first.uav_id} "
                        f"({first.error_code}: {first.message})"
                    )
                else:
                    state.message = f"confirm timeout for UAV: {unresolved[0].uav_id}"
            self._control_participants(state, "ABORT", reason="group acceptance failed")
            result = self._build_result(
                final_status="FAILED",
                failed_uavs=failed_uavs,
                error_code="CONFIRM_FAILED",
                message=state.message,
            )
            self._publish_feedback(state, phase="FAILED", detail=state.message)
            self.server.set_aborted(result, text=result.message)
            return

        if self._schedule_applies(state.command) and state.schedule_tau:
            start_results = self._dispatch_start_scheduled(state)
        else:
            start_results = self._control_participants(state, "START")
        start_failures = [item for item in start_results if not item.accepted]
        if start_failures:
            self._control_participants(state, "ABORT", reason="group START failed")
            self._safe_hold(state, reason="group START failed")
            failed = [item.uav_id for item in start_failures]
            result = self._build_result("FAILED", failed_uavs=failed,
                                        error_code="START_FAILED",
                                        message=start_failures[0].message)
            self.server.set_aborted(result, text=result.message)
            return

        # Every participating UAV accepted.
        with state.lock:
            state.accepted_sent = True
            # Replay states that arrived before the group ACCEPTED was formed.
            for buffered in state.pre_accept:
                self._absorb_state_locked(state, buffered)
                if state.terminal_count() >= len(state.uav_ids):
                    state.stop_event.set()
            state.pre_accept.clear()
        feedback = ExecuteGroupTaskFeedback()
        feedback.phase = "ACCEPTED"
        feedback.active_uavs = list(state.uav_ids)
        feedback.detail = "all participating UAVs accepted"
        self.server.publish_feedback(feedback)
        rospy.loginfo(
            "GCS_A group ACCEPTED for %s/%s after %.3fs",
            state.mission_id,
            state.command_id,
            self._elapsed_s(state),
        )

        # Now wait for per-UAV terminal states: all COMPLETED, or any FAILED,
        # timeout, or cancel.
        deadline = state.start_monotonic + float(goal.timeout_s)
        self._wait_for_terminal(state, deadline)

    def _control_participants(self, state, operation, reason=""):
        deadline = (state.start_monotonic + self.confirm_deadline_s
                    if operation == "START"
                    else time.monotonic() + self.confirm_wait_s)
        results = {uav: SingleUavConfirm(uav, target) for uav, target in state.exec_by_uav.items()}

        def worker(uav_id, exec_target):
            request = UavTaskControlRequest()
            request.protocol_version = PROTOCOL_VERSION
            request.operation = operation
            request.mission_id = state.mission_id
            request.command_id = state.command_id
            request.uav_id = exec_target
            request.exec_target = exec_target
            request.reason = reason
            for attempt in range(self.confirm_retries + 1):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                proxy = rospy.ServiceProxy("/{}/{}".format(exec_target, TASK_CONTROL_SERVICE_SUFFIX), UavTaskControl)
                outcome = self._call_service_with_timeout(proxy, request, min(self.confirm_wait_s, remaining))
                if outcome.exception is None:
                    response = outcome.response
                    results[uav_id].accepted = bool(response.accepted)
                    results[uav_id].status = str(response.status or "")
                    results[uav_id].error_code = str(response.error_code or "")
                    results[uav_id].message = str(response.message or "")
                    if operation == "START" and response.accepted:
                        with state.lock:
                            if uav_id not in state.started_uavs:
                                state.started_uavs.append(uav_id)
                    return
                results[uav_id].message = str(outcome.exception)
                if attempt < self.confirm_retries:
                    time.sleep(min(self.confirm_retry_interval_s,
                                   max(0.0, deadline - time.monotonic())))
            if results[uav_id].accepted is None and not results[uav_id].message:
                results[uav_id].message = "group acceptance deadline expired"

        threads = [threading.Thread(target=worker, args=item, daemon=True)
                   for item in state.exec_by_uav.items()]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(max(0.0, deadline - time.monotonic()))
        return [results[uav] for uav in state.uav_ids]

    def _formation_for_goal(self, goal):
        participants = frozenset(item.uav_id for item in goal.assignments)
        matches = [name for name, configured in self.formation_participants.items()
                   if configured == participants]
        return matches[0] if len(matches) == 1 else None

    def _wait_for_terminal(self, state: GroupState, deadline: float) -> None:
        while not rospy.is_shutdown():
            with state.lock:
                if state.stop_event.is_set():
                    break
                if self.server.is_preempt_requested():
                    state.stop_event.set()
                    state.message = "goal cancelled"
                    state.error_code = "CANCELLED"
                    break
                if state.failed_uavs:
                    state.stop_event.set()
                    break
                if state.terminal_count() >= len(state.uav_ids):
                    state.stop_event.set()
                    break
                now = time.monotonic()
                if now >= deadline:
                    state.stop_event.set()
                    state.message = "group task timed out"
                    state.error_code = "TIMEOUT"
                    break

            self._publish_feedback(state, phase="RUNNING", detail="")
            sleep_s = max(0.1, min(0.5, deadline - time.monotonic()))
            time.sleep(sleep_s)

        if self.server.is_preempt_requested():
            self._safe_hold(state, reason="goal cancelled")
            result = self._build_result(
                final_status="CANCELLED",
                failed_uavs=state.failed_uavs or list(state.uav_ids),
                error_code=state.error_code or "CANCELLED",
                message=state.message or "goal cancelled",
            )
            self._publish_feedback(state, phase="CANCELLED", detail=result.message)
            self.server.set_preempted(result, text=result.message)
            return

        if state.error_code == "TIMEOUT":
            self._safe_hold(state, reason="group task timed out")
            result = self._build_result(
                final_status="FAILED",
                failed_uavs=state.failed_uavs or list(state.uav_ids),
                error_code="TIMEOUT",
                message=state.message or "group task timed out",
            )
            self._publish_feedback(state, phase="FAILED", detail=result.message)
            self.server.set_aborted(result, text=result.message)
            return

        if state.failed_uavs:
            # 白名单安全终态（BRAKE_HOLD/MIN_DISTANCE_BREACH/...）→ 组级安全
            # fan-out（一次，直接 BRAKE_HOLD，排除触发者/已 terminal/失联）；
            # 否则沿用通用失败收口 HOLD。
            if not self._maybe_start_group_safety_hold(state):
                self._safe_hold(
                    state,
                    reason="task failed on participating UAV",
                    skip=state.link_lost_uavs,
                )
            result = self._build_result(
                final_status="FAILED",
                failed_uavs=state.failed_uavs,
                error_code=state.error_code or "UAV_TASK_FAILED",
                message=state.message or "one or more UAVs failed",
            )
            self._publish_feedback(state, phase="FAILED", detail=result.message)
            self.server.set_aborted(result, text=result.message)
            return

        # All reached a terminal state.
        with state.lock:
            all_completed = all(
                state.terminal.get(uav_id) == STATE_COMPLETED for uav_id in state.uav_ids
            )
        if all_completed:
            result = self._build_result(
                final_status="COMPLETED",
                completed_uavs=list(state.uav_ids),
                message=state.message or "all UAVs completed",
            )
            self._publish_feedback(state, phase="COMPLETED", detail=result.message)
            self.server.set_succeeded(result, text=result.message)
            return

        self._safe_hold(state, reason="terminal states inconsistent")
        failed_uavs = [
            uav_id
            for uav_id in state.uav_ids
            if state.terminal.get(uav_id) != STATE_COMPLETED
        ]
        result = self._build_result(
            final_status="FAILED",
            failed_uavs=failed_uavs or list(state.uav_ids),
            error_code=state.error_code or "UAV_TASK_FAILED",
            message=state.message or "some UAVs did not complete",
        )
        self._publish_feedback(state, phase="FAILED", detail=result.message)
        self.server.set_aborted(result, text=result.message)

    def _confirm_participants(
        self, goal, state: GroupState
    ) -> List[SingleUavConfirm]:
        """Concurrently confirm every participating UAV.

        Per UAV: at most confirm_retries+1 attempts, each attempt waits up to
        confirm_wait_s, retry interval confirm_retry_interval_s, all inside the
        confirm_deadline_s budget. A clear rejection fails the whole group fast.
        """
        # START and task confirmation share one absolute acceptance budget.
        deadline = state.start_monotonic + self.confirm_deadline_s
        results: Dict[str, SingleUavConfirm] = {}
        results_lock = threading.Lock()
        group_failed = threading.Event()
        threads: List[threading.Thread] = []

        for uav_id, exec_target in state.exec_by_uav.items():
            confirm = SingleUavConfirm(uav_id=uav_id, exec_target=exec_target)
            results[uav_id] = confirm
            thread = threading.Thread(
                target=self._confirm_one,
                args=(goal, state, confirm, results_lock, deadline, group_failed),
                daemon=True,
            )
            thread.start()
            threads.append(thread)

        # All workers share one absolute deadline. Never spend one deadline per
        # thread, otherwise a large group can violate the 4 s group decision.
        for thread in threads:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            thread.join(timeout=remaining)

        return [results[uav_id] for uav_id in state.uav_ids]

    def _confirm_one(
        self,
        goal,
        state: GroupState,
        confirm: SingleUavConfirm,
        results_lock: threading.Lock,
        deadline: float,
        group_failed: threading.Event,
    ) -> None:
        service_name = "/{}/{}".format(confirm.exec_target, TASK_SERVICE_SUFFIX)
        attempts = self.confirm_retries + 1
        for attempt in range(attempts):
            if group_failed.is_set() or state.stop_event.is_set():
                return
            if rospy.is_shutdown():
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            wait = min(self.confirm_wait_s, remaining)
            try:
                request = self._build_uav_task_request(goal, confirm.exec_target)
                proxy = rospy.ServiceProxy(service_name, UavTask)
                outcome = self._call_service_with_timeout(proxy, request, wait)
                if outcome.exception is not None:
                    raise outcome.exception
                response = outcome.response
                with results_lock:
                    if response.accepted:
                        confirm.accepted = True
                        with state.lock:
                            if confirm.uav_id not in state.accepted_uavs:
                                state.accepted_uavs.append(confirm.uav_id)
                        return
                    # Clear rejection -> fail the group fast.
                    confirm.accepted = False
                    confirm.status = str(response.status or "")
                    confirm.error_code = str(response.error_code or "")
                    confirm.message = str(response.message or "")
                group_failed.set()
                return
            except rospy.ServiceException as exc:
                rospy.logwarn(
                    "confirm %s attempt %d failed: %s",
                    confirm.uav_id,
                    attempt + 1,
                    exc,
                )
            if attempt < attempts - 1:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                rospy.sleep(min(self.confirm_retry_interval_s, remaining))
        with results_lock:
            if confirm.accepted is None:
                confirm.message = "no accepted response within confirm budget"

    @staticmethod
    def _call_service_with_timeout(
        proxy, request, timeout_s: float
    ) -> _CallOutcome:
        """Invoke a blocking rospy ServiceProxy in a worker thread.

        rospy ServiceProxy has no native call timeout, so the worker thread
        stays blocked if the peer never replies; we only wait ``timeout_s``
        and treat a late reply as a timeout for this attempt.
        """
        outcome = _CallOutcome()
        done = threading.Event()

        def worker() -> None:
            try:
                outcome.response = proxy(request)
            except Exception as exc:  # noqa: BLE001
                outcome.exception = exc
            finally:
                done.set()

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()
        if not done.wait(timeout=timeout_s):
            outcome.exception = rospy.ServiceException("service call timed out")
        return outcome

    def _absorb_state_locked(self, state: GroupState, msg: UavTaskState) -> None:
        """Apply one UAV state message under ``state.lock`` (idempotent/out-of-order)."""
        if (
            msg.uav_id not in state.exec_by_uav
            or msg.exec_target != state.exec_by_uav[msg.uav_id]
        ):
            return
        last_seq = state.seen_seq.get(msg.uav_id, -1)
        if msg.status_seq <= last_seq:
            return
        state.seen_seq[msg.uav_id] = msg.status_seq
        state.latest_states[msg.uav_id] = msg

        status = str(msg.status or "").upper()
        if status == STATE_COMPLETED:
            state.terminal[msg.uav_id] = STATE_COMPLETED
        elif status == STATE_FAILED:
            state.terminal[msg.uav_id] = STATE_FAILED
            if msg.uav_id not in state.failed_uavs:
                state.failed_uavs.append(msg.uav_id)
            state.message = msg.message or state.message
            state.error_code = msg.error_code or state.error_code

    def _build_uav_task_request(self, goal, exec_target: str) -> UavTaskRequest:
        import copy as _copy
        request = UavTaskRequest()
        request.protocol_version = PROTOCOL_VERSION
        request.mission_id = goal.mission_id
        request.group_id = str(getattr(goal, "group_id", "") or "GroupA")
        request.command_id = goal.command_id
        # 机载 ROS 层统一使用执行层 UAVn 名称；任务层 A01..A15 的换算只在
        # GCS_A 边界发生（identity_map 是唯一翻译点）。
        request.uav_id = str(exec_target)
        request.exec_target = exec_target
        request.command = str(goal.command or "")
        request.timeout_s = float(goal.timeout_s)
        leader_id = str(getattr(goal, "leader_id", "") or "")
        if leader_id:
            try:
                leader_id = self.identity_map.identity(leader_id).exec_target
            except KeyError:
                leader_id = ""
        request.leader_id = leader_id

        # Locate this UAV's assignment in the goal.
        assignment: Optional[TaskAssignment] = None
        for item in goal.assignments:
            ident = self.identity_map.identity(item.uav_id)
            if ident.exec_target == exec_target:
                assignment = item
                break
        if assignment is None:
            raise ValueError(f"no assignment for exec_target {exec_target}")
        # 不修改共享 goal：深拷贝后把 assignment.uav_id 改写为执行层名称，
        # 使 executor 的 identity 校验（uav_id/exec_target）一致。
        translated = _copy.deepcopy(assignment)
        translated.uav_id = str(exec_target)
        task_uav_id = assignment.uav_id
        command = str(getattr(goal, "command", "") or "")
        formation_name = self._formation_for_goal(goal) if command == "FOLLOW_ROUTE" else None
        # Runtime yaw is intentionally ignored. Translate task right/forward
        # coordinates into the frozen local ENU frame before onboard dispatch.
        def translate_pose(pose):
            x, y, z = task_point_to_local_enu(pose.x, pose.y, pose.z,
                                              self.reference_heading, self.mission_anchor)
            pose.x, pose.y, pose.z, pose.yaw = x, y, z, self.reference_heading
        if command in ("MOVE_TO", "FAULT_EXIT"):
            if translated.waypoints:
                for point in translated.waypoints:
                    translate_pose(point)
            else:
                translate_pose(translated.target_pose)
        elif command == "FOLLOW_ROUTE":
            if task_uav_id == self.formation_leader:
                translated.formation_follow = False
                for point in translated.waypoints:
                    translate_pose(point)
            else:
                translated.formation_follow = True
                table = self.formation_offsets.get(formation_name, {})
                offset = table.get(task_uav_id)
                if offset is None:
                    raise ValueError("missing fixed formation slot for %s" % task_uav_id)
                ex, ny = task_vector_to_enu(offset[0], offset[1], self.reference_heading)
                translated.formation_offset.x = ex
                translated.formation_offset.y = ny
                translated.formation_offset.z = float(offset[2])
                translated.formation_offset.yaw = 0.0
                leader_assignment = next(
                    item for item in goal.assignments
                    if item.uav_id == self.formation_leader)
                if leader_assignment.waypoints:
                    translated.target_pose = _copy.deepcopy(leader_assignment.waypoints[-1])
                    translate_pose(translated.target_pose)
        request.assignment = translated
        return request

    def _safe_hold(
        self,
        state: GroupState,
        reason: str,
        skip: Optional[List[str]] = None,
        targets: Optional[List[str]] = None,
    ) -> None:
        """Batched, retrying concurrent HOLD across active (accepted, not terminal) UAVs.

        Fan-out is split into batches of hold_batch_size (default 4) to avoid
        saturating the bridge service forwarding (26082700: 12 concurrent
        /uav_hold calls timed out on A12). Unconfirmed members of each batch
        are retried hold_retry_count times (default 2) with a
        hold_retry_interval_s (0.5s) gap to survive transient bridge timeouts.

        ``targets`` (task-layer Axx subset, optional) restricts fan-out to the
        requested subset when provided (implementation_plan_26090900 §4.2);
        None keeps the historical all-started-active semantics.  Existing
        filters are always retained: started, not terminal, not pending,
        not in ``skip``.
        """
        target_set = None if targets is None else set(targets)
        with state.lock:
            candidates = [
                uav_id
                for uav_id in state.uav_ids
                if uav_id in state.started_uavs
                if state.terminal.get(uav_id) is None
                and uav_id not in state.hold_pending
                and (skip is None or uav_id not in skip)
                and (target_set is None or uav_id in target_set)
            ]
            if not candidates:
                return
            state.hold_pending.extend(candidates)

        def hold_one(uav_id: str, accepted: set, rejected: dict,
                     lock: threading.Lock) -> None:
            exec_target = state.exec_by_uav[uav_id]
            service_name = "/{}/{}".format(exec_target, HOLD_SERVICE_SUFFIX)
            request = UavHoldRequest()
            request.protocol_version = PROTOCOL_VERSION
            request.mission_id = state.mission_id
            request.command_id = state.command_id
            # 机载层统一使用执行层 UAVn 名称。
            request.uav_id = exec_target
            request.exec_target = exec_target
            request.reason = reason
            try:
                proxy = rospy.ServiceProxy(service_name, UavHold)
                outcome = self._call_service_with_timeout(
                    proxy, request, self.confirm_wait_s
                )
                if outcome.exception is not None:
                    raise outcome.exception
                response = outcome.response
                with lock:
                    if response.accepted:
                        accepted.add(uav_id)
                        state.held.append(uav_id)
                    else:
                        rejected[uav_id] = "{}: {}".format(
                            response.error_code, response.message
                        )
            except rospy.ServiceException as exc:
                with lock:
                    rejected[uav_id] = str(exc)

        for start in range(0, len(candidates), self.hold_batch_size):
            batch = candidates[start:start + self.hold_batch_size]
            pending = list(batch)
            for attempt in range(self.hold_retry_count + 1):
                if not pending:
                    break
                lock = threading.Lock()
                accepted: set = set()
                rejected: dict = {}
                threads = [
                    threading.Thread(
                        target=hold_one, args=(uav_id, accepted, rejected, lock),
                        daemon=True,
                    )
                    for uav_id in pending
                ]
                for thread in threads:
                    thread.start()
                for thread in threads:
                    thread.join(timeout=self.confirm_wait_s + 1.0)
                if attempt < self.hold_retry_count:
                    # 0.5s 后重试本批仍未确认的机。
                    time.sleep(self.hold_retry_interval_s)
                    pending = [
                        uav_id for uav_id in pending if uav_id not in accepted
                    ]
            for uav_id, detail in rejected.items():
                state.hold_rejected.append(uav_id)
                rospy.logwarn("HOLD failed for %s after %d retries: %s",
                              uav_id, self.hold_retry_count, detail)

        with state.lock:
            if state.hold_rejected:
                state.message = (
                    f"HOLD not confirmed for: {','.join(state.hold_rejected)}"
                )
                rospy.logwarn(
                    "HOLD unconfirmed for %s, reason=%s", state.hold_rejected, reason
                )

    def _maybe_start_group_safety_hold(self, state: GroupState) -> bool:
        """One-shot group safety fan-out on a whitelisted terminal event.

        implementation_plan_26090900 §4.3：任一参与 UAV 上报 FAILED 且
        error_code ∈ {BRAKE_HOLD, MIN_DISTANCE_BREACH, EMERGENCY_BRAKE} → 只触发
        一次：对同任务仍需收口的其他参与者（started、未 terminal、未失联、非
        触发者）以 reason "GROUP_BRAKE_HOLD:<error_code>:<trigger>" 下发直接
        BRAKE_HOLD。调用方为 Action/run_goal 线程（非 subscriber 回调，无持锁
        跨 master 等待）。返回 True 表示本次失败已由组级安全 fan-out 处理。
        """
        with state.lock:
            if state.group_safety_hold_started:
                return False
            trigger = None
            code = ""
            for uav_id in state.failed_uavs:
                last = state.latest_states.get(uav_id)
                error_code = str(getattr(last, "error_code", "") or "") if last else ""
                if error_code in GROUP_SAFETY_ERROR_CODES:
                    trigger = uav_id
                    code = error_code
                    break
            if trigger is None:
                return False
            state.group_safety_hold_started = True
            state.group_safety_trigger_uav = trigger
            state.group_safety_error_code = code
            targets = [
                uav_id
                for uav_id in state.uav_ids
                if uav_id != trigger
                and uav_id in state.started_uavs
                and state.terminal.get(uav_id) is None
                and uav_id not in state.link_lost_uavs
            ]
        reason = "{}:{}:{}".format(
            GROUP_BRAKE_HOLD_REASON_PREFIX, code, trigger
        )
        rospy.logwarn(
            "GCS_A group safety hold triggered by %s (%s); targets=%s",
            trigger,
            code,
            ",".join(targets) if targets else "none",
        )
        self._safe_hold(state, reason=reason, targets=targets or None)
        return True

    def _handle_emergency_hold(self, request) -> EmergencyHoldResponse:
        with self._active_lock:
            state = self._active
        if state is None:
            return EmergencyHoldResponse(
                accepted=False,
                message="no active group task to hold",
            )
        # Map local mission/uav ids onto the active group if mission matches.
        if request.mission_id and state.mission_id != request.mission_id:
            return EmergencyHoldResponse(
                accepted=False,
                message="active group mission_id mismatch",
            )
        # SimpleActionServer 没有 cancel_goal_at_server（那是 client 的方法）。
        # 设置 active group 的 stop_event 让 _execute/_wait_for_terminal 线程
        # 自行终止并发布终态结果；随后仅对本次请求的有效目标执行直接 BRAKE_HOLD
        # （任务层 Axx 与 group 交集，request 外成员不下发，见 §4.2）。
        raw_ids = getattr(request, "uav_ids", None) or []
        requested = list(dict.fromkeys(raw_ids)) or list(state.uav_ids)
        unknown = [uav_id for uav_id in requested if uav_id not in state.uav_ids]
        effective = [uav_id for uav_id in requested if uav_id in state.uav_ids]
        with state.lock:
            state.stop_event.set()
            state.message = request.reason or "emergency hold"
            state.error_code = "EMERGENCY_HOLD"
        executable = [
            uav_id
            for uav_id in effective
            if uav_id in state.started_uavs
            and state.terminal.get(uav_id) is None
            and uav_id not in state.link_lost_uavs
        ]
        to_hold = [uav_id for uav_id in executable if uav_id not in state.held]
        # runtime 指定安全 HOLD 一律以直接 BRAKE_HOLD 语义下发（§4.1/§3.2）。
        # 无可执行目标（已全部 held / 全部不可执行）时不调用服务（幂等跳过）。
        if to_hold:
            self._safe_hold(
                state,
                reason=RUNTIME_BRAKE_HOLD_REASON,
                targets=to_hold,
            )
        with state.lock:
            unconfirmed = [
                uav_id for uav_id in effective if uav_id not in state.held
            ]
        parts: List[str] = []
        if unknown:
            parts.append("requested outside active group: %s" % ",".join(unknown))
        if unconfirmed:
            parts.append("hold unconfirmed for: %s" % ",".join(unconfirmed))
        if not effective:
            parts.append("no requested UAV in active group")
        detail = "; ".join(parts) if parts else "hold confirmed for all requested active UAVs"
        # 保守语义：只要任一请求目标未确认（含未知/不可执行），整体 accepted=false，
        # TCP 侧按原 assignments 全失败集合返回 FAILED/HOLD_REJECTED（§4.1 step 6）。
        accepted = bool(effective) and not unknown and not unconfirmed
        return EmergencyHoldResponse(
            accepted=accepted,
            message=detail,
        )

    def shutdown(self) -> None:
        if hasattr(self, "_safety_stop"):
            self._safety_stop.set()
        if hasattr(self, "_safety_thread") and self._safety_thread is not threading.current_thread():
            self._safety_thread.join(1.5)

    # ------------------------------------------------------------------ utils

    @staticmethod
    def _elapsed_s(state: GroupState) -> float:
        return time.monotonic() - state.start_monotonic

    def _publish_feedback(self, state: GroupState, phase: str, detail: str) -> None:
        with state.lock:
            active_uavs = [
                uav_id for uav_id in state.uav_ids if state.terminal.get(uav_id) is None
            ]
        feedback = ExecuteGroupTaskFeedback()
        feedback.phase = phase
        feedback.active_uavs = active_uavs
        feedback.detail = detail
        self.server.publish_feedback(feedback)

    def _build_result(
        self,
        final_status: str,
        completed_uavs: Optional[List[str]] = None,
        failed_uavs: Optional[List[str]] = None,
        error_code: str = "",
        message: str = "",
    ) -> ExecuteGroupTaskResult:
        result = ExecuteGroupTaskResult()
        result.final_status = final_status
        result.completed_uavs = list(completed_uavs or [])
        result.failed_uavs = list(failed_uavs or [])
        result.error_code = error_code
        result.message = message
        return result


def main() -> None:
    rospy.init_node("gcs_a_backend_node", anonymous=False)
    GcsABackendNode()
    rospy.spin()


if __name__ == "__main__":
    main()
