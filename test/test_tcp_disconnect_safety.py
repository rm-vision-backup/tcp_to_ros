#!/usr/bin/env python3
"""Regression test for TCP connection-loss compensation."""

import importlib.util
import os
import sys
import unittest

from tcp_to_ros.protocol import Assignment, ParsedCommand


SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "tcp_server_node.py",
)
SPEC = importlib.util.spec_from_file_location("tcp_server_node", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class TcpDisconnectSafetyTest(unittest.TestCase):
    def test_disconnect_cancels_action_and_requests_group_hold(self):
        node = MODULE.TcpToRosServerNode.__new__(MODULE.TcpToRosServerNode)
        command = ParsedCommand(
            mission_id="mission-1",
            group_id="GroupA",
            command_id="command-1",
            command="MOVE_TO",
            timeout_s=10.0,
            assignments=[Assignment(uav_id="A01"), Assignment(uav_id="A02")],
        )
        active = type("Active", (), {"command": command})()
        node.tasks = type("Tasks", (), {"suppress_current": lambda _self: active})()
        cancelled = []
        node.action_client = type(
            "Action", (), {"cancel_all_goals": lambda _self: cancelled.append(True)}
        )()
        holds = []
        node.emergency_hold = lambda request: holds.append(request)
        finals = []
        node.cache = type(
            "Cache", (), {"set_final": lambda _self, key, payload: finals.append((key, payload))}
        )()
        states = []
        node._publish_state = lambda *args, **kwargs: states.append((args, kwargs))

        node._handle_disconnect("runtime link lost")

        self.assertEqual([True], cancelled)
        self.assertEqual(1, len(holds))
        self.assertEqual(["A01", "A02"], list(holds[0].uav_ids))
        self.assertEqual("runtime link lost", holds[0].reason)
        self.assertEqual("CONNECTION_CLOSED", finals[0][1]["error_code"])
        self.assertEqual("DISCONNECTED", states[0][1]["ros_status"])


if __name__ == "__main__":
    unittest.main()