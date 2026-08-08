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

import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import actionlib
import rospy

from tcp_to_ros.msg import (
    ExecuteGroupTaskAction,
    ExecuteGroupTaskFeedback,
    ExecuteGroupTaskResult,
)
from tcp_to_ros.srv import (
    EmergencyHold,
    EmergencyHoldResponse,
)
from swarm_uav_interfaces.msg import TaskAssignment, UavTaskState
from swarm_uav_interfaces.srv import (
    UavHold,
    UavHoldRequest,
    UavTask,
    UavTaskRequest,
)

from tcp_to_ros.identity import IdentityMap

PROTOCOL_VERSION = "1.0"

# Static per-UAV cross-master naming convention (documented in CONTEXT.md / docs):
# task Service  : /{exec_target}/uav_task
# hold Service  : /{exec_target}/uav_hold
# state Topic   : /{exec_target}/uav_task_state
TASK_SERVICE_SUFFIX = "uav_task"
HOLD_SERVICE_SUFFIX = "uav_hold"
STATE_TOPIC_SUFFIX = "uav_task_state"

# Confirm plan constants
DEFAULT_CONFIRM_WAIT_S = 1.0
DEFAULT_CONFIRM_RETRIES = 2
DEFAULT_CONFIRM_RETRY_INTERVAL_S = 0.2
DEFAULT_CONFIRM_DEADLINE_S = 4.0

STATE_ACCEPTED = "ACCEPTED"
STATE_COMPLETED = "COMPLETED"
STATE_FAILED = "FAILED"


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

    lock: threading.RLock = field(default_factory=threading.RLock)
    stop_event: threading.Event = field(default_factory=threading.Event)

    accepted: bool = False
    accepted_sent: bool = False
    # UAVs that returned accepted=true and therefore own an action which may
    # require compensation even before group ACCEPTED is formed.
    accepted_uavs: List[str] = field(default_factory=list)
    terminal: Dict[str, str] = field(default_factory=dict)  # uav_id -> COMPLETED/FAILED
    latest_states: Dict[str, UavTaskState] = field(default_factory=dict)
    seen_seq: Dict[str, int] = field(default_factory=dict)  # uav_id -> max status_seq
    pre_accept: List[UavTaskState] = field(default_factory=list)  # terminal states arriving before group ACCEPTED
    failed_uavs: List[str] = field(default_factory=list)
    hold_pending: List[str] = field(default_factory=list)
    held: List[str] = field(default_factory=list)
    hold_rejected: List[str] = field(default_factory=list)
    error_code: str = ""
    message: str = ""

    @property
    def key(self) -> str:
        return f"{self.mission_id}|{self.command_id}"

    def terminal_count(self) -> int:
        return len(self.terminal)


class GcsABackendNode:
    def __init__(self) -> None:
        self.action_name = rospy.get_param("~action_name", "/group_a/execute_task")
        self.emergency_service_name = rospy.get_param(
            "~emergency_hold_service",
            "/group_a/emergency_hold",
        )
        self.identity_map_path = rospy.get_param(
            "~map_file",
            "/home/ub20tg/catkin_swarm6-2/src/tcp_to_ros/config/uav_identity_map.yaml",
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

        self.identity_map = IdentityMap.load(self.identity_map_path)

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

    def _on_uav_state(self, msg: UavTaskState) -> None:
        with self._active_lock:
            state = self._active
        if state is None:
            return
        if msg.command_id != state.command_id or msg.mission_id != state.mission_id:
            return
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
            )
            self._active = state

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

    def _run_goal(self, goal, state: GroupState) -> None:
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
            self._safe_hold(state, reason="group acceptance failed")
            result = self._build_result(
                final_status="FAILED",
                failed_uavs=failed_uavs,
                error_code="CONFIRM_FAILED",
                message=state.message,
            )
            self._publish_feedback(state, phase="FAILED", detail=state.message)
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
        deadline = state.start_time + float(goal.timeout_s)
        self._wait_for_terminal(state, deadline)

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
                now = rospy.Time.now().to_sec()
                if now >= deadline:
                    state.stop_event.set()
                    state.message = "group task timed out"
                    state.error_code = "TIMEOUT"
                    break

            self._publish_feedback(state, phase="RUNNING", detail="")
            sleep_s = max(0.1, min(0.5, deadline - rospy.Time.now().to_sec()))
            rospy.sleep(sleep_s)

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
            self._safe_hold(
                state,
                reason="task failed on participating UAV",
                skip=state.failed_uavs,
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
        deadline = rospy.Time.now().to_sec() + self.confirm_deadline_s
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
            remaining = deadline - rospy.Time.now().to_sec()
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
            remaining = deadline - rospy.Time.now().to_sec()
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
                remaining = deadline - rospy.Time.now().to_sec()
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
        request = UavTaskRequest()
        request.protocol_version = PROTOCOL_VERSION
        request.mission_id = goal.mission_id
        request.group_id = str(getattr(goal, "group_id", "") or "GroupA")
        request.command_id = goal.command_id
        request.uav_id = ""
        request.exec_target = exec_target
        request.command = str(goal.command or "")
        request.timeout_s = float(goal.timeout_s)
        request.leader_id = str(getattr(goal, "leader_id", "") or "")

        # Locate this UAV's assignment in the goal.
        assignment: Optional[TaskAssignment] = None
        for item in goal.assignments:
            ident = self.identity_map.identity(item.uav_id)
            if ident.exec_target == exec_target:
                assignment = item
                request.uav_id = item.uav_id
                break
        if assignment is None:
            raise ValueError(f"no assignment for exec_target {exec_target}")
        request.assignment = assignment
        return request

    def _safe_hold(
        self, state: GroupState, reason: str, skip: Optional[List[str]] = None
    ) -> None:
        """Concurrent HOLD across active (accepted, not terminal) UAVs."""
        with state.lock:
            candidates = [
                uav_id
                for uav_id in state.uav_ids
                if uav_id in state.accepted_uavs
                if state.terminal.get(uav_id) is None
                and uav_id not in state.hold_pending
                and (skip is None or uav_id not in skip)
            ]
            if not candidates:
                return
            state.hold_pending.extend(candidates)

        lock = threading.Lock()

        def hold_one(uav_id: str) -> None:
            exec_target = state.exec_by_uav[uav_id]
            service_name = "/{}/{}".format(exec_target, HOLD_SERVICE_SUFFIX)
            request = UavHoldRequest()
            request.protocol_version = PROTOCOL_VERSION
            request.mission_id = state.mission_id
            request.command_id = state.command_id
            request.uav_id = uav_id
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
                        state.held.append(uav_id)
                    else:
                        state.hold_rejected.append(uav_id)
                        rospy.logwarn(
                            "HOLD rejected for %s: %s %s",
                            uav_id,
                            response.error_code,
                            response.message,
                        )
            except rospy.ServiceException as exc:
                with lock:
                    state.hold_rejected.append(uav_id)
                rospy.logwarn("HOLD failed for %s: %s", uav_id, exc)

        threads = [
            threading.Thread(target=hold_one, args=(uav_id,), daemon=True)
            for uav_id in candidates
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=self.confirm_wait_s + 1.0)

        with state.lock:
            if state.hold_rejected:
                state.message = (
                    f"HOLD not confirmed for: {','.join(state.hold_rejected)}"
                )
                rospy.logwarn(
                    "HOLD unconfirmed for %s, reason=%s", state.hold_rejected, reason
                )

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
        self.server.cancel_goal_at_server()
        self._safe_hold(state, reason=request.reason or "emergency hold")
        with state.lock:
            held_ok = all(uav_id in state.held for uav_id in state.uav_ids)
            if held_ok:
                detail = "hold confirmed for all active UAVs"
            else:
                detail = (
                    f"hold unconfirmed for: {','.join(state.hold_rejected)}"
                    if state.hold_rejected
                    else "no active UAVs to hold"
                )
        return EmergencyHoldResponse(
            accepted=held_ok,
            message=detail,
        )

    # ------------------------------------------------------------------ utils

    @staticmethod
    def _elapsed_s(state: GroupState) -> float:
        return rospy.Time.now().to_sec() - state.start_time

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