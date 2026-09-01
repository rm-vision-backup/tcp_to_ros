#!/usr/bin/env python3
"""Tests for the t_off scheduler and its GCS_A backend integration.

Covers implementation_plan_26090106:
- S8.1 scheduler unit tests: parallel / head-on / orthogonal crossing /
  different horizon lengths / negative tau shift / disjoint safe sets /
  pairwise-feasible-but-globally-infeasible / empty safe set / search caps.
  Every successful schedule is re-verified: all pairwise min distances >= d_ij,
  UAV1 is part of the variance, and the post-shift relative offsets and
  variance match the pre-shift values.
- S8.2 GCS integration: missing/stale pose, normal scheduling, no-solution
  rejection, START dispatch deviation logging, derived budget, insufficient
  task timeout, and the non-MOVE_TO/FAULT_EXIT regression path.
"""

from __future__ import annotations

import importlib.util
import math
import os
import sys
import threading
import time
import unittest
from types import SimpleNamespace
from unittest import mock

from swarm_uav_interfaces.msg import Pose3DYaw
from tcp_to_ros.msg import ExecuteGroupTaskGoal

# NOTE: tcp_to_ros.time_offset_scheduler and the generated tcp_to_ros.msg are
# resolved from the devel tree; the scripts/ source package must not shadow the
# generated messages, so no sys.path injection happens here. Run after
# `catkin build tcp_to_ros`.

from tcp_to_ros.time_offset_scheduler import (  # noqa: E402
    GEOMETRY_UNSUITABLE,
    GLOBAL_SCHEDULE_INFEASIBLE,
    PAIR_SAFE_SET_EMPTY,
    SCHEDULE_LIMIT_REACHED,
    SCHEDULE_OK,
    ScheduleParams,
    UavSegment,
    compute_schedule,
    _min_distance,
    _pair_safe_intervals,
    _floyd_point,
    _search_tau,
    _Motion,
)

SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "gcs_a_backend_node.py",
)
SPEC = importlib.util.spec_from_file_location("gcs_a_backend_node", SCRIPT)
BACKEND = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = BACKEND
SPEC.loader.exec_module(BACKEND)


def make_params(**overrides) -> ScheduleParams:
    values = dict(
        horizontal_speed_mps=0.7,
        geometric_distance_m=1.0,
        safety_margin_m=0.5,
        time_epsilon_s=1e-6,
        max_start_delay_s=30.0,
        max_interval_combinations=2000,
        max_compute_time_s=5.0,
    )
    values.update(overrides)
    return ScheduleParams(**values)


class ScheduleModelVerifierMixin:
    """Shared checks from plan 8.1; every test verifies these invariants."""

    def assert_model_safe(self, result, params):
        self.assertTrue(result.success, result.failure_reason)
        d = params.safe_distance_m
        for pair in result.pair_results:
            if pair.predicted_min_distance_m is not None:
                self.assertGreaterEqual(
                    pair.predicted_min_distance_m, d - 1e-6,
                    "pair %s/%s predicted dmin below d_ij" % (pair.uav_i, pair.uav_j))
        # UAV1 participates in the variance (raw tau[0] == 0 reference).
        self.assertAlmostEqual(result.tau_raw[0], 0.0, places=6)
        # Shifting by -min(tau) preserves relative offsets.
        for i in range(len(result.tau_raw)):
            for j in range(len(result.tau_raw)):
                self.assertAlmostEqual(
                    (result.tau[j] - result.tau[i])
                    - (result.tau_raw[j] - result.tau_raw[i]), 0.0, places=6)
        # Shifted variance equals the raw variance.
        mean = sum(result.tau_raw) / len(result.tau_raw)
        expected = sum((t - mean) ** 2 for t in result.tau_raw) / len(result.tau_raw)
        self.assertAlmostEqual(result.variance, expected, places=6)
        # All send times are non-negative after the shift.
        self.assertGreaterEqual(min(result.tau), -1e-9)
        self.assertLessEqual(max(result.tau), params.max_start_delay_s + 1e-6)


class TimeOffsetSchedulerUnitTest(unittest.TestCase, ScheduleModelVerifierMixin):

    def test_parallel_same_direction_synchronous(self):
        # Plan 8.1: same-direction parallel -> tau stays synchronous.
        params = make_params()
        result = compute_schedule([
            UavSegment("A01", (0.0, 0.0), (50.0, 0.0)),
            UavSegment("A02", (0.0, 3.0), (50.0, 3.0)),
        ], params)
        self.assert_model_safe(result, params)
        self.assertEqual(result.schedule_status, SCHEDULE_OK)
        self.assertAlmostEqual(result.variance, 0.0, places=9)
        self.assertEqual(result.tau, [0.0, 0.0])

    def test_head_on_generates_nonzero_offset(self):
        params = make_params()
        result = compute_schedule([
            UavSegment("A01", (0.0, 0.0), (10.0, 0.0)),
            UavSegment("A02", (10.0, 0.0), (0.0, 0.0)),
        ], params)
        self.assert_model_safe(result, params)
        self.assertEqual(result.schedule_status, SCHEDULE_OK)
        delta = result.tau[1] - result.tau[0]
        self.assertGreater(abs(delta), 5.0, "head-on must produce a real offset")

    def test_orthogonal_crossing_safe_boundary(self):
        params = make_params()
        result = compute_schedule([
            UavSegment("A01", (0.0, 0.0), (50.0, 0.0)),
            UavSegment("A02", (25.0, -25.0), (25.0, 25.0)),
        ], params)
        self.assert_model_safe(result, params)
        self.assertEqual(result.schedule_status, SCHEDULE_OK)
        delta = result.tau[1] - result.tau[0]
        self.assertGreater(abs(delta), 1.0, "crossing needs a timing offset")

    def test_different_horizon_lengths(self):
        # Same direction, same speed, different lengths -> delta = 0 is safe.
        params = make_params()
        result = compute_schedule([
            UavSegment("A01", (0.0, 0.0), (50.0, 0.0)),
            UavSegment("A02", (20.0, 0.0), (50.0, 0.0)),
        ], params)
        self.assert_model_safe(result, params)
        self.assertEqual(result.schedule_status, SCHEDULE_OK)
        self.assertAlmostEqual(result.variance, 0.0, places=9)

    def test_negative_tau_shifts_to_nonnegative(self):
        params = make_params()
        result = compute_schedule([
            UavSegment("A01", (0.0, 0.0), (10.0, 0.0)),
            UavSegment("A02", (10.0, 0.0), (0.0, 0.0)),
            UavSegment("A03", (0.0, 2.0), (10.0, 2.0)),
        ], params)
        self.assert_model_safe(result, params)
        self.assertEqual(result.schedule_status, SCHEDULE_OK)
        self.assertLess(min(result.tau_raw), -1e-6)

    def test_disjoint_safe_sets_preserved(self):
        params = make_params()
        motions = [
            _Motion("A01", (0.0, 0.0), (0.7, 0.0), 40.0 / 0.7),
            _Motion("A02", (20.0, -20.0), (0.0, 0.7), 40.0 / 0.7),
        ]
        intervals = _pair_safe_intervals(
            motions[0], motions[1], params.safe_distance_m,
            -params.max_start_delay_s, params.max_start_delay_s,
            params.time_epsilon_s)
        self.assertGreaterEqual(len(intervals), 2, "expected disjoint intervals")
        # The unsafe middle gap (delta ~ 0) must be excluded.
        self.assertTrue(all(not (l - 1e-6 <= 0.0 <= r + 1e-6) for (l, r) in intervals))
        for (l, r) in intervals:
            for sample in (l, (l + r) / 2.0, r):
                md = _min_distance(motions[0], motions[1], sample)
                self.assertTrue(md is None or md >= params.safe_distance_m - 1e-6)

    def test_pairwise_feasible_globally_infeasible(self):
        # Plan 8.1: every pair has a non-empty safe set yet no global tau
        # vector exists. Real geometry almost never triggers this (two-sided
        # symmetric safe sets plus delta_13 = delta_12 + delta_23 always leave
        # a same-side combination), so the mechanism is verified directly: the
        # difference-constraint feasibility check must detect the negative
        # cycle of a narrow two-sided band, and the interval-combination search
        # must exhaust every combination without a solution.
        n = 3
        constraints = [
            (0, 1, 14.29, 15.0),
            (0, 2, 14.29, 15.0),
            (1, 2, 14.29, 15.0),
        ]
        for (i, j, l, r) in constraints:
            point, feas = _floyd_point(n, [(i, j, l, r)])
            self.assertTrue(feas, "pair (%d,%d) must be feasible" % (i, j))
        # Together they form a conflicting cycle: delta12 = delta02 - delta01
        # lands in (-0.71, 0.71) or the far band, never in [14.29, 15].
        point, feas = _floyd_point(n, constraints)
        self.assertFalse(feas)

        # compute_schedule's GLOBAL_SCHEDULE_INFEASIBLE branch: exhaustive DFS
        # over the two-sided band combinations finds no verified tau.
        params = make_params(max_interval_combinations=1000)
        motions = [
            _Motion("A01", (0.0, 0.0), (0.7, 0.0), 10.0 / 0.7),
            _Motion("A02", (10.0, 0.0), (-0.7, 0.0), 10.0 / 0.7),
            _Motion("A03", (0.0, 2.0), (0.7, 0.0), 10.0 / 0.7),
        ]
        bands = [(-15.0, -14.29), (14.29, 15.0)]
        intervals = {(0, 1): bands, (0, 2): bands, (1, 2): bands}
        best_tau, _best_var, limited, _combos = _search_tau(
            3, motions, [(0, 1), (0, 2), (1, 2)], intervals,
            params, time.monotonic())
        self.assertIsNone(best_tau)
        self.assertFalse(limited)
    def test_empty_safe_set(self):
        params = make_params(max_start_delay_s=10.0)
        result = compute_schedule([
            UavSegment("A01", (0.0, 0.0), (50.0, 0.0)),
            UavSegment("A02", (50.0, 0.0), (0.0, 0.0)),
        ], params)
        self.assertFalse(result.success)
        self.assertEqual(result.schedule_status, PAIR_SAFE_SET_EMPTY)
        self.assertEqual(sorted(result.fail_ids), ["A01", "A02"])

    def test_search_cap_hit_without_solution(self):
        params = make_params(max_compute_time_s=1e-9)
        result = compute_schedule([
            UavSegment("A01", (0.0, 0.0), (10.0, 0.0)),
            UavSegment("A02", (10.0, 0.0), (0.0, 0.0)),
        ], params)
        self.assertFalse(result.success)
        self.assertEqual(result.schedule_status, SCHEDULE_LIMIT_REACHED)

    def test_search_cap_hit_with_feasible_solution(self):
        params = make_params(max_interval_combinations=1)
        result = compute_schedule([
            UavSegment("A01", (0.0, 0.0), (10.0, 0.0)),
            UavSegment("A02", (10.0, 0.0), (0.0, 0.0)),
        ], params)
        self.assertTrue(result.success)
        self.assertEqual(result.schedule_status, SCHEDULE_OK)
        self.assertTrue(result.non_global_optimal)
        self.assert_model_safe(result, params)

    def test_geometry_unsuitable_zero_displacement(self):
        params = make_params()
        result = compute_schedule([
            UavSegment("A01", (3.0, 4.0), (3.0, 4.0)),
        ], params)
        self.assertFalse(result.success)
        self.assertEqual(result.schedule_status, GEOMETRY_UNSUITABLE)

    def test_single_uav_is_trivial(self):
        params = make_params()
        result = compute_schedule([
            UavSegment("A01", (0.0, 0.0), (50.0, 0.0)),
        ], params)
        self.assertTrue(result.success)
        self.assertEqual(result.schedule_status, SCHEDULE_OK)
        self.assertEqual(result.tau, [0.0])

    def test_params_validation(self):
        with self.assertRaises(ValueError):
            make_params(horizontal_speed_mps=-1.0).validate()
        with self.assertRaises(ValueError):
            make_params(geometric_distance_m=0.0).validate()
        with self.assertRaises(ValueError):
            make_params(safety_margin_m=-0.1).validate()
        with self.assertRaises(ValueError):
            make_params(max_interval_combinations=0).validate()
        with self.assertRaises(ValueError):
            make_params(max_compute_time_s=float("nan")).validate()



class GcsBackendScheduleIntegrationTest(unittest.TestCase):
    """Plan 8.2 integration tests against the GCS_A backend methods."""

    def setUp(self):
        self.node = BACKEND.GcsABackendNode.__new__(BACKEND.GcsABackendNode)
        self.node.move_to_schedule_enabled = True
        self.node.fault_exit_schedule_enabled = True
        self.node.schedule_params = make_params()
        self.node.schedule_pose_timeout_s = 1.0
        self.node.start_service_budget_s = 1.5
        self.node.start_dispatch_jitter_margin_s = 0.5
        self.node.confirm_wait_s = 0.05
        self.node.confirm_retries = 0
        self.node.confirm_retry_interval_s = 0.01
        self.node.confirm_deadline_s = 4.0
        self.node.reference_heading = 0.0
        self.node.mission_anchor = (0.0, 0.0, 0.0)
        self.node._pose_cache = {}
        self.node._pose_lock = threading.Lock()
        self.node.identity_map = SimpleNamespace(
            resolve=lambda ids: [
                (i, SimpleNamespace(exec_target="UAV%s" % i[1:])) for i in ids])
        self.node.server = mock.Mock()
        self.node.server.is_preempt_requested.return_value = False
        self.node._wait_for_terminal = mock.Mock()

    def _make_state(self, command="MOVE_TO"):
        return BACKEND.GroupState(
            mission_id="m", command_id="c", command=command,
            uav_ids=["A01", "A02"],
            exec_by_uav={"A01": "UAV1", "A02": "UAV2"},
            start_time=0.0, start_monotonic=time.monotonic())

    def _make_goal(self, command="MOVE_TO", timeout_s=60.0, a1_goal=(1.0, 0.0),
                   a2_goal=(0.0, 0.0)):
        goal = ExecuteGroupTaskGoal()
        goal.mission_id = "m"
        goal.command_id = "c"
        goal.command = command
        goal.timeout_s = float(timeout_s)
        a1 = BACKEND.TaskAssignment()
        a1.uav_id = "A01"
        a1.target_pose = Pose3DYaw(x=a1_goal[0], y=a1_goal[1], z=12.0, yaw=0.0)
        a2 = BACKEND.TaskAssignment()
        a2.uav_id = "A02"
        a2.target_pose = Pose3DYaw(x=a2_goal[0], y=a2_goal[1], z=12.0, yaw=0.0)
        goal.assignments = [a1, a2]
        return goal

    def _inject_pose(self, uav1_xy=(0.0, 0.0), uav2_xy=(1.0, 0.0)):
        now = time.monotonic()
        self.node._pose_cache = {
            "UAV1": {"x": uav1_xy[0], "y": uav1_xy[1], "recv_monotonic": now},
            "UAV2": {"x": uav2_xy[0], "y": uav2_xy[1], "recv_monotonic": now},
        }

    def _stub_services(self, calls):
        def fake_call(proxy, request, timeout_s):
            calls.append((time.monotonic(), request))
            response = SimpleNamespace(
                accepted=True, status="ACCEPTED", error_code="", message="")
            return BACKEND._CallOutcome(response=response)

        self.node._call_service_with_timeout = fake_call

    def test_pose_missing_prevents_prepare_with_fail_id(self):
        # Plan 8.2: pose topic missing -> no PREPARE, FAILED with fail ids.
        goal = self._make_goal()
        state = self._make_state()
        self._inject_pose(uav1_xy=(0.0, 0.0), uav2_xy=(1.0, 0.0))
        self.node._pose_cache["UAV1"]["recv_monotonic"] = time.monotonic() - 100.0
        self.node._confirm_participants = mock.Mock(
            side_effect=AssertionError("must not PREPARE on schedule failure"))
        self.node._run_goal(goal, state)
        result = self.node.server.set_aborted.call_args[0][0]
        self.assertEqual(result.final_status, "FAILED")
        self.assertEqual(result.error_code, "SCHEDULE_POSE_UNAVAILABLE")
        # Only UAV1's pose was made stale; the fail id is the stale UAV.
        self.assertEqual(result.failed_uavs, ["A01"])
        self.node._confirm_participants.assert_not_called()

    def test_move_to_schedules_start_by_tau(self):
        # Plan 8.2: PREPARE concurrency happens, START is sent by tau offsets
        # (head-on 1 m -> A02 first, A01 ~1.43 s later).
        goal = self._make_goal()
        state = self._make_state()
        self._inject_pose(uav1_xy=(0.0, 0.0), uav2_xy=(1.0, 0.0))

        def fake_confirm(_goal, _state):
            return [
                BACKEND.SingleUavConfirm("A01", "UAV1", accepted=True),
                BACKEND.SingleUavConfirm("A02", "UAV2", accepted=True),
            ]

        self.node._confirm_participants = fake_confirm
        calls = []
        self._stub_services(calls)
        with mock.patch.object(BACKEND.rospy, "ServiceProxy", return_value=object()):
            self.node._run_goal(goal, state)

        start_calls = [item for item in calls if item[1].operation == "START"]
        self.assertEqual(len(start_calls), 2)
        times = {item[1].uav_id: item[0] for item in start_calls}
        self.assertIn("UAV1", times)
        self.assertIn("UAV2", times)
        self.assertGreater(times["UAV1"], times["UAV2"],
                           "scheduled START must respect tau offsets")
        self.assertAlmostEqual(times["UAV1"] - times["UAV2"], 1.0 / 0.7,
                               delta=0.7, msg="START interval ~ tau difference")

    def test_no_solution_prevents_task_send(self):
        # Plan 8.2: infeasible geometry -> no task sent, FAILED result.
        # task frame -> local ENU: (x, y) -> (y, -x). Goals (0,50)/(0,0) with
        # local starts (0,0)/(50,0) form a true 50 m head-on pair, which has an
        # empty safe set at max_start_delay_s=30 (needs |delta| >= 71.4 s).
        goal = self._make_goal(a1_goal=(0.0, 50.0), a2_goal=(0.0, 0.0))
        state = self._make_state()
        self._inject_pose(uav1_xy=(0.0, 0.0), uav2_xy=(50.0, 0.0))
        self.node._confirm_participants = mock.Mock(
            side_effect=AssertionError("must not PREPARE on infeasible schedule"))
        self.node._run_goal(goal, state)
        result = self.node.server.set_aborted.call_args[0][0]
        self.assertEqual(result.final_status, "FAILED")
        self.assertEqual(result.error_code, "SCHEDULE_PAIR_SAFE_SET_EMPTY")
        self.node._confirm_participants.assert_not_called()

    def test_timeout_insufficient_rejected_upfront(self):
        # Plan 8.2: task timeout too small -> rejected before PREPARE instead of
        # a mid-run local timeout on the UAV.
        goal = self._make_goal(timeout_s=1.0)
        state = self._make_state()
        self._inject_pose(uav1_xy=(0.0, 0.0), uav2_xy=(1.0, 0.0))
        self.node._confirm_participants = mock.Mock(
            side_effect=AssertionError("must not PREPARE on timeout rejection"))
        self.node._run_goal(goal, state)
        result = self.node.server.set_aborted.call_args[0][0]
        self.assertEqual(result.final_status, "FAILED")
        self.assertEqual(result.error_code, "SCHEDULE_TIMEOUT_INSUFFICIENT")
        self.node._confirm_participants.assert_not_called()

    def test_derived_start_budget(self):
        # Plan 8.2: derived deadline = max(tau') + budget + jitter, and it is
        # checked against the task timeout before PREPARE.
        goal = self._make_goal(timeout_s=60.0)
        state = self._make_state()
        self._inject_pose(uav1_xy=(0.0, 0.0), uav2_xy=(1.0, 0.0))
        ok = self.node._compute_and_store_schedule(goal, state)
        self.assertTrue(ok)
        self.assertEqual(sorted(state.schedule_tau), ["A01", "A02"])
        max_tau = max(state.schedule_tau.values())
        self.assertGreater(max_tau, 0.0)
        # Derived deadline (plan 5.4): max(tau') + max horizontal exec time
        # + start_service_budget_s + start_dispatch_jitter_margin_s.
        max_horizontal_time_s = 1.0 / 0.7  # 1 m segment at 0.7 m/s
        needed = (max_tau + max_horizontal_time_s
                  + self.node.start_service_budget_s
                  + self.node.start_dispatch_jitter_margin_s)
        self.assertLessEqual(needed, float(goal.timeout_s))
        # state.schedule_status is log-only and mirrors the scheduler result.
        self.assertEqual(state.schedule_status, SCHEDULE_OK)

    def test_start_dispatch_records_deviation(self):
        # Plan 8.2: every scheduled START logs plan/actual/deviation.
        goal = self._make_goal()
        state = self._make_state()
        self._inject_pose(uav1_xy=(0.0, 0.0), uav2_xy=(1.0, 0.0))

        def fake_confirm(_goal, _state):
            return [
                BACKEND.SingleUavConfirm("A01", "UAV1", accepted=True),
                BACKEND.SingleUavConfirm("A02", "UAV2", accepted=True),
            ]

        self.node._confirm_participants = fake_confirm
        calls = []
        self._stub_services(calls)
        logged = []
        with mock.patch.object(BACKEND.rospy, "ServiceProxy", return_value=object()), \
                mock.patch.object(
                    BACKEND.rospy, "loginfo",
                    side_effect=lambda msg, *args: logged.append((msg, args))):
            self.node._run_goal(goal, state)
        dispatch_logs = [item for item in logged
                         if "START dispatch" in item[0]]
        self.assertEqual(len(dispatch_logs), 2)

    def test_non_move_to_keeps_existing_flow(self):
        # Plan 8.2: non MOVE_TO/FAULT_EXIT never enters the scheduler.
        goal = self._make_goal(command="FOLLOW_ROUTE")
        state = self._make_state(command="FOLLOW_ROUTE")
        self.node._compute_and_store_schedule = mock.Mock(
            side_effect=AssertionError("scheduler must not run for FOLLOW_ROUTE"))

        def fake_confirm(_goal, _state):
            return [
                BACKEND.SingleUavConfirm("A01", "UAV1", accepted=True),
                BACKEND.SingleUavConfirm("A02", "UAV2", accepted=True),
            ]

        self.node._confirm_participants = fake_confirm
        self.node._control_participants = mock.Mock(return_value=[
            BACKEND.SingleUavConfirm("A01", "UAV1", accepted=True),
            BACKEND.SingleUavConfirm("A02", "UAV2", accepted=True),
        ])
        self.node._run_goal(goal, state)
        self.node._compute_and_store_schedule.assert_not_called()
        self.node._control_participants.assert_called_once()


if __name__ == "__main__":
    unittest.main()
