#!/usr/bin/env python3
"""Focused regression tests for GCS_A aggregation safety boundaries."""

import importlib.util
import os
import sys
import threading
import time
import unittest
from unittest import mock

from swarm_uav_interfaces.msg import UavTaskState


SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "gcs_a_backend_node.py",
)
SPEC = importlib.util.spec_from_file_location("gcs_a_backend_node", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def make_state(uav_id, exec_target, status, seq, error_code="", message=""):
    msg = UavTaskState()
    msg.mission_id = "mission-1"
    msg.command_id = "command-1"
    msg.uav_id = uav_id
    msg.exec_target = exec_target
    msg.status = status
    msg.status_seq = seq
    msg.error_code = error_code
    msg.message = message
    return msg


class GcsAAggregationTest(unittest.TestCase):
    def setUp(self):
        self.node = MODULE.GcsABackendNode.__new__(MODULE.GcsABackendNode)
        # _safe_hold 分批/重试参数（__new__ 跳过 __init__，此处显式声明）。
        self.node.hold_batch_size = 4
        self.node.hold_retry_count = 2
        self.node.hold_retry_interval_s = 0.5
        self.state = MODULE.GroupState(
            mission_id="mission-1",
            command_id="command-1",
            command="MOVE_TO",
            uav_ids=["A01", "A02"],
            exec_by_uav={"A01": "UAV1", "A02": "UAV2"},
            start_time=0.0,
        )

    def test_ignores_non_participant_and_wrong_exec_target(self):
        self.node._absorb_state_locked(
            self.state, make_state("A03", "UAV3", "COMPLETED", 1)
        )
        self.node._absorb_state_locked(
            self.state, make_state("A01", "UAV2", "COMPLETED", 1)
        )
        self.assertEqual({}, self.state.terminal)
        self.assertEqual({}, self.state.seen_seq)

    def test_ignores_duplicate_and_out_of_order_status(self):
        self.node._absorb_state_locked(
            self.state, make_state("A01", "UAV1", "COMPLETED", 2)
        )
        self.node._absorb_state_locked(
            self.state, make_state("A01", "UAV1", "FAILED", 2)
        )
        self.node._absorb_state_locked(
            self.state, make_state("A01", "UAV1", "FAILED", 1)
        )
        self.assertEqual("COMPLETED", self.state.terminal["A01"])
        self.assertEqual([], self.state.failed_uavs)

    def test_hold_targets_only_accepted_nonterminal_uavs(self):
        self.state.started_uavs = ["A01"]
        captured = []

        def fake_call(_proxy, request, _timeout):
            captured.append(request.uav_id)
            response = type("Response", (), {"accepted": True})()
            return MODULE._CallOutcome(response=response)

        self.node.confirm_wait_s = 0.01
        self.node._call_service_with_timeout = fake_call
        self.node._safe_hold(self.state, "test")
        # 机载层统一使用执行层 UAVn 命名；A01 -> UAV1。
        self.assertEqual(["UAV1"], captured)
        self.assertEqual(["A01"], self.state.held)

    def test_safe_hold_batches_and_retries(self):
        # 26082700 修复：>batch_size 台机分批下发（每批 hold_batch_size），
        # 未确认机按 hold_retry_interval_s 重试 hold_retry_count 次，最终全部确认。
        state = MODULE.GroupState(
            mission_id="mission-1", command_id="command-1", command="MOVE_TO",
            uav_ids=["A01", "A02", "A03", "A04", "A05", "A06"],
            exec_by_uav={"A01": "UAV1", "A02": "UAV2", "A03": "UAV3",
                         "A04": "UAV4", "A05": "UAV5", "A06": "UAV6"},
            start_time=0.0,
        )
        state.started_uavs = ["A01", "A02", "A03", "A04", "A05", "A06"]
        self.node.confirm_wait_s = 0.01
        self.node.hold_batch_size = 4
        self.node.hold_retry_count = 2
        self.node.hold_retry_interval_s = 0.01
        calls = []
        first_fail = {"UAV5"}

        def fake_call(_proxy, request, _timeout):
            calls.append(request.uav_id)
            if request.uav_id in first_fail:
                first_fail.remove(request.uav_id)  # 首次调用 bridge 超时
                return MODULE._CallOutcome(
                    exception=MODULE.rospy.ServiceException("bridge timeout"))
            response = type("Response", (), {"accepted": True})()
            return MODULE._CallOutcome(response=response)

        self.node._call_service_with_timeout = fake_call
        self.node._safe_hold(state, "test")
        # 6 台 → 4+2 两批；UAV5 在第二批首次失败，0.5s 间隔后重试成功。
        self.assertEqual(calls[:4], ["UAV1", "UAV2", "UAV3", "UAV4"])
        self.assertEqual(calls.count("UAV5"), 2)  # 首次超时 + 重试成功
        self.assertIn("UAV6", calls)
        self.assertEqual(
            sorted(state.held),
            ["A01", "A02", "A03", "A04", "A05", "A06"],
        )
        self.assertEqual(state.hold_rejected, [])

    def test_late_service_response_remains_timed_out(self):
        def late_proxy(_request):
            time.sleep(0.05)
            return object()

        outcome = self.node._call_service_with_timeout(late_proxy, object(), 0.01)
        self.assertIsInstance(outcome.exception, MODULE.rospy.ServiceException)
        time.sleep(0.06)
        self.assertIsInstance(outcome.exception, MODULE.rospy.ServiceException)

    def test_confirm_worker_respects_shared_deadline(self):
        self.node.confirm_retries = 20
        self.node.confirm_retry_interval_s = 0.01
        self.node.confirm_wait_s = 0.02
        self.node._build_uav_task_request = lambda _goal, _target: object()
        calls = []

        def always_timeout(_proxy, _request, timeout_s):
            calls.append(timeout_s)
            time.sleep(timeout_s)
            return MODULE._CallOutcome(
                exception=MODULE.rospy.ServiceException("service call timed out")
            )

        self.node._call_service_with_timeout = always_timeout
        confirm = MODULE.SingleUavConfirm("A01", "UAV1")
        deadline = time.monotonic() + 0.08
        started = time.monotonic()
        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            side_effect=lambda: MODULE.rospy.Time.from_sec(time.time()),
        ), mock.patch.object(MODULE.rospy, "sleep", side_effect=time.sleep):
            self.node._confirm_one(
                object(),
                self.state,
                confirm,
                threading.Lock(),
                deadline,
                threading.Event(),
            )
        elapsed = time.monotonic() - started

        self.assertLess(elapsed, 0.14)
        self.assertIsNone(confirm.accepted)
        self.assertEqual("no accepted response within confirm budget", confirm.message)
        self.assertGreaterEqual(len(calls), 1)

    def _active_node(self):
        self.node._active = self.state
        self.node._active_lock = threading.RLock()
        self.node.confirm_wait_s = 0.01
        return self.node

    def _make_hold_request(self, uav_ids=None, reason="operator cancel", mission_id="mission-1"):
        return type(
            "Request",
            (),
            {
                "mission_id": mission_id,
                "reason": reason,
                "uav_ids": list(uav_ids) if uav_ids is not None else None,
            },
        )()

    def test_emergency_hold_targets_subset_with_brake_hold_reason(self):
        # implementation_plan_26090900 §4.2：runtime 指定 HOLD 只向 request 与
        # group 交集的有效目标（started/未 terminal）以 RUNTIME_BRAKE_HOLD 下发；
        # request 外的 group 成员不被波及。
        self.state.started_uavs = ["A01", "A02"]
        self._active_node()
        calls = []

        def fake_hold(state, reason, skip=None, targets=None):
            calls.append((reason, list(targets or [])))
            for uav_id in targets or []:
                state.held.append(uav_id)

        self.node._safe_hold = fake_hold
        response = self.node._handle_emergency_hold(
            self._make_hold_request(uav_ids=["A01"])
        )
        self.assertEqual(calls, [(MODULE.RUNTIME_BRAKE_HOLD_REASON, ["A01"])])
        self.assertTrue(response.accepted)
        self.assertEqual("hold confirmed for all requested active UAVs", response.message)
        self.assertTrue(self.state.stop_event.is_set())
        self.assertEqual("EMERGENCY_HOLD", self.state.error_code)

    def test_emergency_hold_unconfirmed_target_rejects(self):
        # 目标拒绝/未确认 → 整体 accepted=false，绝不构造假成功。
        self.state.started_uavs = ["A01"]
        self._active_node()
        calls = []

        def fake_hold(state, reason, skip=None, targets=None):
            calls.append(list(targets or []))
            state.hold_pending.extend(list(targets or []))
            state.hold_rejected.extend(list(targets or []))

        self.node._safe_hold = fake_hold
        response = self.node._handle_emergency_hold(
            self._make_hold_request(uav_ids=["A01"])
        )
        self.assertEqual(calls, [["A01"]])
        self.assertFalse(response.accepted)
        self.assertEqual("hold unconfirmed for: A01", response.message)
        # stop_event 置位以终止执行线程（EmergencyHold 语义）。
        self.assertTrue(self.state.stop_event.is_set())

    def test_emergency_hold_unknown_target_not_sent_and_rejected(self):
        # request 目标不在 active group：不下发任何 HOLD，整体拒绝并说明原因。
        self.state.started_uavs = ["A01"]
        self._active_node()
        self.node._safe_hold = lambda *args, **kwargs: self.fail(
            "must not hold any UAV for unknown-only request"
        )
        response = self.node._handle_emergency_hold(
            self._make_hold_request(uav_ids=["A03"])
        )
        self.assertFalse(response.accepted)
        self.assertIn("no requested UAV in active group", response.message)

    def test_emergency_hold_mixed_unknown_and_valid(self):
        # 部分请求目标不在 group：有效目标照常 brake hold，但整体保守拒绝。
        self.state.started_uavs = ["A01"]
        self._active_node()
        calls = []

        def fake_hold(state, reason, skip=None, targets=None):
            calls.append(list(targets or []))
            for uav_id in targets or []:
                state.held.append(uav_id)

        self.node._safe_hold = fake_hold
        response = self.node._handle_emergency_hold(
            self._make_hold_request(uav_ids=["A01", "A99"])
        )
        self.assertEqual(calls, [["A01"]])
        self.assertFalse(response.accepted)
        self.assertIn("requested outside active group: A99", response.message)

    def test_emergency_hold_idempotent_when_already_held(self):
        # 已 held 目标重复请求：幂等成功，不重复调用 service。
        self.state.started_uavs = ["A01"]
        self.state.held = ["A01"]
        self._active_node()
        self.node._safe_hold = lambda *args, **kwargs: self.fail(
            "must not re-send HOLD for already-held target"
        )
        response = self.node._handle_emergency_hold(
            self._make_hold_request(uav_ids=["A01"])
        )
        self.assertTrue(response.accepted)
        self.assertEqual("hold confirmed for all requested active UAVs", response.message)

    def test_emergency_hold_non_started_target_is_unconfirmed(self):
        # 未 started/已 terminal 目标不得默认为成功：请求含此类成员时保守拒绝。
        self.state.started_uavs = ["A02"]
        self._active_node()
        calls = []

        def fake_hold(state, reason, skip=None, targets=None):
            calls.append(list(targets or []))
            for uav_id in targets or []:
                state.held.append(uav_id)

        self.node._safe_hold = fake_hold
        response = self.node._handle_emergency_hold(
            self._make_hold_request(uav_ids=["A01", "A02"])
        )
        # A01 未 started → 不下发、不得计为成功 → 整体拒绝。
        self.assertEqual(calls, [["A02"]])
        self.assertFalse(response.accepted)
        self.assertEqual("hold unconfirmed for: A01", response.message)

    def test_mission_mismatch_rejects(self):
        self._active_node()
        response = self.node._handle_emergency_hold(
            self._make_hold_request(uav_ids=["A01"], mission_id="other")
        )
        self.assertFalse(response.accepted)
        self.assertEqual("active group mission_id mismatch", response.message)


    def test_link_loss_affects_only_participating_uav(self):
        self.node._active = self.state
        self.node._active_lock = threading.RLock()
        self.node._inject_link_lost("A03")
        self.assertEqual([], self.state.failed_uavs)
        self.assertFalse(self.state.stop_event.is_set())
        self.node._inject_link_lost("A01")
        self.assertEqual(["A01"], self.state.link_lost_uavs)
        self.assertEqual(["A01"], self.state.failed_uavs)
        self.assertEqual("UAV_LINK_LOST", self.state.error_code)
        self.assertTrue(self.state.stop_event.is_set())

    def test_group_safety_fanout_fires_once_with_whitelist(self):
        # implementation_plan_26090900 §4.3：白名单安全终态只触发一次 fan-out，
        # reason=GROUP_BRAKE_HOLD:<error_code>:<trigger>，目标排除触发者/失联。
        state = MODULE.GroupState(
            mission_id="mission-1", command_id="command-1", command="MOVE_TO",
            uav_ids=["A01", "A02", "A03", "A04", "A05", "A06"],
            exec_by_uav={"A01": "UAV1", "A02": "UAV2", "A03": "UAV3",
                         "A04": "UAV4", "A05": "UAV5", "A06": "UAV6"},
            start_time=0.0,
        )
        state.started_uavs = ["A01", "A02", "A03", "A04", "A05", "A06"]
        state.link_lost_uavs = ["A02"]
        self.node._active = state
        self.node._active_lock = threading.RLock()
        calls = []

        def fake_hold(group_state, reason, skip=None, targets=None):
            calls.append((reason, list(targets or [])))
            for uav_id in targets or []:
                group_state.held.append(uav_id)

        self.node._safe_hold = fake_hold
        # 触发者 A05（UAV5）上报 FAILED/BRAKE_HOLD。
        self.node._absorb_state_locked(
            state,
            make_state("A05", "UAV5", "FAILED", 7, error_code="BRAKE_HOLD"),
        )
        state.failed_uavs = ["A05"]
        first = self.node._maybe_start_group_safety_hold(state)
        self.assertTrue(first)
        self.assertTrue(state.group_safety_hold_started)
        self.assertEqual(state.group_safety_trigger_uav, "A05")
        self.assertEqual(state.group_safety_error_code, "BRAKE_HOLD")
        self.assertEqual(
            calls,
            [(MODULE.GROUP_BRAKE_HOLD_REASON_PREFIX + ":BRAKE_HOLD:A05",
              ["A01", "A03", "A04", "A06"])],
        )
        # 第二次调用不再 fan-out（重复/乱序状态吸收后由一次性字段防重）。
        self.assertFalse(self.node._maybe_start_group_safety_hold(state))
        self.assertEqual(len(calls), 1)

    def test_group_safety_fanout_excludes_terminal_and_trigger(self):
        state = MODULE.GroupState(
            mission_id="mission-1", command_id="command-1", command="MOVE_TO",
            uav_ids=["A01", "A02", "A03"],
            exec_by_uav={"A01": "UAV1", "A02": "UAV2", "A03": "UAV3"},
            start_time=0.0,
        )
        state.started_uavs = ["A01", "A02", "A03"]
        state.terminal["A01"] = MODULE.STATE_COMPLETED
        self.node._active = state
        self.node._active_lock = threading.RLock()
        calls = []

        def fake_hold(group_state, reason, skip=None, targets=None):
            calls.append(list(targets or []))

        self.node._safe_hold = fake_hold
        self.node._absorb_state_locked(
            state,
            make_state("A03", "UAV3", "FAILED", 9, error_code="MIN_DISTANCE_BREACH"),
        )
        state.failed_uavs = ["A03"]
        self.assertTrue(self.node._maybe_start_group_safety_hold(state))
        # A01 已 COMPLETED → 排除；A02 仍执行 → 收口。
        self.assertEqual(calls, [["A02"]])

    def test_plain_failure_does_not_trigger_group_safety_fanout(self):
        state = MODULE.GroupState(
            mission_id="mission-1", command_id="command-1", command="MOVE_TO",
            uav_ids=["A01", "A02"],
            exec_by_uav={"A01": "UAV1", "A02": "UAV2"},
            start_time=0.0,
        )
        state.started_uavs = ["A01", "A02"]
        self.node._active = state
        self.node._active_lock = threading.RLock()
        calls = []

        def fake_hold(group_state, reason, skip=None, targets=None):
            calls.append(reason)

        self.node._safe_hold = fake_hold
        self.node._absorb_state_locked(
            state,
            make_state("A01", "UAV1", "FAILED", 3, error_code="EGO_PLAN_FAILED"),
        )
        state.failed_uavs = ["A01"]
        # 非白名单错误码 → 不 fan-out（generic _safe_hold 由调用方处理）。
        self.assertFalse(self.node._maybe_start_group_safety_hold(state))
        self.assertEqual(calls, [])


    def test_link_lost_uav_is_skipped_by_group_hold(self):
        self.state.started_uavs = ["A01", "A02"]
        self.state.link_lost_uavs = ["A01"]
        captured = []

        def fake_call(_proxy, request, _timeout):
            captured.append(request.uav_id)
            return MODULE._CallOutcome(response=type("Response", (), {"accepted": True})())

        self.node.confirm_wait_s = 0.01
        self.node._call_service_with_timeout = fake_call
        self.node._safe_hold(self.state, "link lost", skip=self.state.link_lost_uavs)
        # 机载层统一使用执行层 UAVn 命名；A02 -> UAV2。
        self.assertEqual(["UAV2"], captured)


if __name__ == "__main__":
    unittest.main()
