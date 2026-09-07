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


def make_state(uav_id, exec_target, status, seq):
    msg = UavTaskState()
    msg.mission_id = "mission-1"
    msg.command_id = "command-1"
    msg.uav_id = uav_id
    msg.exec_target = exec_target
    msg.status = status
    msg.status_seq = seq
    return msg


class GcsAAggregationTest(unittest.TestCase):
    def setUp(self):
        self.node = MODULE.GcsABackendNode.__new__(MODULE.GcsABackendNode)
        # _safe_hold 分批/重试参数（__new__ 跳过 __init__，此处显式声明）。
        self.node.hold_batch_size = 4
        self.node.hold_retry_count = 2
        self.node.hold_retry_interval_s = 0.5
        self.node.group_safety_hold_targets = "all_enabled"
        self.node._enabled_exec_targets = [
            "UAV1", "UAV2", "UAV3", "UAV4", "UAV5", "UAV6", "UAV7", "UAV8",
            "UAV9", "UAV10", "UAV11", "UAV12", "UAV13", "UAV14", "UAV15",
        ]
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

    def test_group_safety_broadcast_includes_all_enabled(self):
        # 变更 F：白名单安全 error_code 终态 → 异步向 all_enabled（含 reserve）广播。
        fail = make_state("A01", "UAV1", "FAILED", 3)
        fail.error_code = "MIN_DISTANCE_BREACH"
        self.node._absorb_state_locked(self.state, fail)
        captured = []
        self.node._broadcast_group_safety_hold = lambda state, targets: captured.extend(targets)
        self.node._maybe_broadcast_group_safety(self.state)
        time.sleep(0.05)
        self.assertEqual(len(captured), 15)  # 含 reserve UAV13..UAV15
        self.assertTrue(self.state.group_safety_broadcasted)
        # 只触发一次。
        self.node._maybe_broadcast_group_safety(self.state)
        time.sleep(0.05)
        self.assertEqual(len(captured), 15)

    def test_group_safety_participants_target(self):
        self.node.group_safety_hold_targets = "participants"
        fail = make_state("A01", "UAV1", "FAILED", 3)
        fail.error_code = "BRAKE_HOLD"
        self.node._absorb_state_locked(self.state, fail)
        captured = []
        self.node._broadcast_group_safety_hold = lambda state, targets: captured.extend(targets)
        self.node._maybe_broadcast_group_safety(self.state)
        time.sleep(0.05)
        self.assertEqual(sorted(captured), ["UAV1", "UAV2"])

    def test_group_safety_whitelist_ignores_normal_failures(self):
        fail = make_state("A01", "UAV1", "FAILED", 3)
        fail.error_code = "EGO_PLAN_FAILED"  # 普通规划失败不在白名单
        self.node._absorb_state_locked(self.state, fail)
        self.node._broadcast_group_safety_hold = lambda state, targets: self.fail(
            "ordinary failure must not broadcast GROUP_SAFETY")
        self.node._maybe_broadcast_group_safety(self.state)
        time.sleep(0.05)
        self.assertFalse(self.state.group_safety_broadcasted)

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

    def test_emergency_hold_reports_rejected_uavs_and_cancels_action(self):
        self.state.accepted_uavs = ["A01"]
        self.node._active = self.state
        self.node._active_lock = threading.RLock()

        def fake_hold(state, reason):
            self.assertEqual("operator cancel", reason)
            state.hold_pending.append("A01")
            state.hold_rejected.append("A01")

        self.node._safe_hold = fake_hold
        request = type(
            "Request", (), {"mission_id": "mission-1", "reason": "operator cancel"}
        )()
        response = self.node._handle_emergency_hold(request)

        self.assertFalse(response.accepted)
        self.assertEqual("hold unconfirmed for: A01", response.message)
        # EmergencyHold must stop the executing group thread so it publishes a
        # terminal result, instead of calling a nonexistent server cancel API.
        self.assertTrue(self.state.stop_event.is_set())
        self.assertEqual("EMERGENCY_HOLD", self.state.error_code)

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
