#!/usr/bin/env python3
"""Regression tests for the runtime TCP/ROS XML-RPC port collision."""

import importlib.util
import os
import sys
import tempfile
import unittest
from unittest import mock

from tcp_to_ros.server import (
    linux_port_is_ephemeral_and_unreserved,
    parse_port_range,
    parse_reserved_ports,
)


SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "tcp_server_node.py",
)
SPEC = importlib.util.spec_from_file_location("tcp_server_node_port_order", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class TcpPortPolicyTest(unittest.TestCase):
    def test_parses_linux_port_policy(self):
        self.assertEqual((32768, 60999), parse_port_range("32768\t60999\n"))
        self.assertEqual({39001, 40000, 40001, 40002}, parse_reserved_ports("39001,40000-40002\n"))

    def test_detects_unreserved_ephemeral_port(self):
        with tempfile.NamedTemporaryFile(mode="w") as range_file, tempfile.NamedTemporaryFile(mode="w") as reserved_file:
            range_file.write("32768 60999\n")
            range_file.flush()
            reserved_file.write("40000-40010\n")
            reserved_file.flush()
            self.assertTrue(
                linux_port_is_ephemeral_and_unreserved(
                    39001, range_file.name, reserved_file.name
                )
            )

    def test_reserved_port_is_safe(self):
        with tempfile.NamedTemporaryFile(mode="w") as range_file, tempfile.NamedTemporaryFile(mode="w") as reserved_file:
            range_file.write("32768 60999\n")
            range_file.flush()
            reserved_file.write("39001\n")
            reserved_file.flush()
            self.assertFalse(
                linux_port_is_ephemeral_and_unreserved(
                    39001, range_file.name, reserved_file.name
                )
            )

    def test_listener_is_claimed_before_ros_backend_wait(self):
        events = []
        listener = mock.Mock()
        action_client = mock.Mock()
        action_client.wait_for_server.side_effect = lambda: events.append("wait")
        params = {
            "~host": "0.0.0.0",
            "~port": 39001,
            "~action_name": "/group_a/execute_task",
            "~state_topic": "/group_a/group_task_state",
            "~emergency_hold_service": "/group_a/emergency_hold",
        }

        with mock.patch.object(MODULE.rospy, "get_param", side_effect=lambda key, default=None: params.get(key, default)), mock.patch.object(
            MODULE, "create_listener", side_effect=lambda *_args: events.append("bind") or listener
        ), mock.patch.object(
            MODULE, "linux_port_is_ephemeral_and_unreserved", return_value=False
        ), mock.patch.object(
            MODULE.rospy, "Publisher", return_value=mock.Mock()
        ), mock.patch.object(
            MODULE.actionlib, "SimpleActionClient", return_value=action_client
        ), mock.patch.object(
            MODULE.rospy, "wait_for_service"
        ), mock.patch.object(
            MODULE.rospy, "ServiceProxy", return_value=mock.Mock()
        ):
            MODULE.TcpToRosServerNode()

        self.assertEqual(["bind", "wait"], events)


if __name__ == "__main__":
    unittest.main()