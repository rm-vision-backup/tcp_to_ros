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
        self.state.accepted_uavs = ["A01"]
        captured = []

        def fake_call(_proxy, request, _timeout):
            captured.append(request.uav_id)
            response = type("Response", (), {"accepted": True})()
            return MODULE._CallOutcome(response=response)

        self.node.confirm_wait_s = 0.01
        self.node._call_service_with_timeout = fake_call
        self.node._safe_hold(self.state, "test")
        self.assertEqual(["A01"], captured)
        self.assertEqual(["A01"], self.state.held)

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
        deadline = time.time() + 0.08
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
        cancelled = []
        self.node.server = type(
            "Server", (), {"cancel_goal_at_server": lambda _self: cancelled.append(True)}
        )()

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
        self.assertEqual([True], cancelled)


if __name__ == "__main__":
    unittest.main()