#!/usr/bin/env python3
"""Deterministic tests for the GCS_A monotonic link-health state machine."""

import unittest

from tcp_to_ros.link_health import (
    DISCOVERED,
    HEALTHY,
    LOST,
    RECOVERED,
    SUSPECT,
    UNDISCOVERED,
    UavLinkMonitor,
)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def set(self, value):
        self.now = float(value)


class UavLinkMonitorTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.monitor = UavLinkMonitor(
            [("A01", "UAV1"), ("A02", "UAV2")],
            suspect_after_s=3.0,
            lost_after_s=5.0,
            healthy_window_s=3.0,
            clock=self.clock,
        )

    def heartbeat(self, at, connected=True):
        self.clock.set(at)
        self.monitor.record_state("A01", "UAV1", connected)
        return self.monitor.evaluate()

    def test_undiscovered_member_never_times_out(self):
        self.clock.set(100.0)
        self.assertEqual([], self.monitor.evaluate())
        self.assertEqual(UNDISCOVERED, self.monitor.observation("A02").state)

    def test_discovered_requires_stability_window(self):
        transitions = self.heartbeat(0.0)
        self.assertEqual([DISCOVERED], [item.current for item in transitions])
        self.heartbeat(1.0)
        self.heartbeat(2.0)
        transitions = self.heartbeat(3.0)
        self.assertEqual([HEALTHY], [item.current for item in transitions])
        self.assertTrue(self.monitor.is_stably_healthy("A01"))

    def test_monotonic_suspect_and_lost_boundaries(self):
        self.heartbeat(0.0)
        self.heartbeat(3.0)
        self.clock.set(5.999)
        self.assertEqual([], self.monitor.evaluate())
        self.clock.set(6.0)
        self.assertEqual([SUSPECT], [item.current for item in self.monitor.evaluate()])
        self.clock.set(8.0)
        self.assertEqual([LOST], [item.current for item in self.monitor.evaluate()])

    def test_connected_false_is_immediately_suspect_and_never_renews(self):
        self.heartbeat(0.0)
        self.heartbeat(3.0)
        transitions = self.heartbeat(3.5, connected=False)
        self.assertEqual([SUSPECT], [item.current for item in transitions])
        self.assertAlmostEqual(0.5, self.monitor.snapshot("A01", 10.0).age_s)
        self.clock.set(8.0)
        self.assertEqual([LOST], [item.current for item in self.monitor.evaluate()])

    def test_lost_recovery_is_latched_until_stable(self):
        self.heartbeat(0.0)
        self.heartbeat(3.0)
        self.clock.set(8.0)
        self.monitor.evaluate()
        transitions = self.heartbeat(9.0)
        self.assertEqual([RECOVERED], [item.current for item in transitions])
        self.assertFalse(self.monitor.is_stably_healthy("A01"))
        self.heartbeat(10.0)
        self.heartbeat(11.0)
        transitions = self.heartbeat(12.0)
        self.assertEqual([HEALTHY], [item.current for item in transitions])

    def test_snapshot_uses_valid_age_and_session_metadata(self):
        self.heartbeat(2.0, connected=False)
        snapshot = self.monitor.snapshot(
            "A01", 123.5, mission_id="mission", session_epoch="epoch"
        )
        self.assertEqual(-1.0, snapshot.age_s)
        self.assertFalse(snapshot.connected)
        self.assertEqual(SUSPECT, snapshot.state)
        self.assertEqual("mission", snapshot.mission_id)
        self.assertEqual("epoch", snapshot.session_epoch)
        self.assertEqual(123.5, snapshot.reported_at)

    def test_rejects_unknown_or_mismatched_identity(self):
        with self.assertRaises(KeyError):
            self.monitor.record_state("A03", "UAV3", True)
        with self.assertRaises(ValueError):
            self.monitor.record_state("A01", "UAV2", True)

    def test_rejects_invalid_thresholds(self):
        with self.assertRaises(ValueError):
            UavLinkMonitor([], suspect_after_s=0.0)
        with self.assertRaises(ValueError):
            UavLinkMonitor([], suspect_after_s=3.0, lost_after_s=3.0)
        with self.assertRaises(ValueError):
            UavLinkMonitor([], healthy_window_s=-1.0)


if __name__ == "__main__":
    unittest.main()