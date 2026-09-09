#!/usr/bin/env python3
"""Pure unit tests for runtime HOLD terminal semantics (implementation_plan_26090900 §4.1/§7.1).

No roscore/TCP listener required: exercises protocol parsing plus the
``_hold_terminal_payload`` decision helper used by ``tcp_server_node``.
"""
from __future__ import annotations

import importlib.util
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..")))

SERVER_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "tcp_server_node.py",
)
SPEC = importlib.util.spec_from_file_location("tcp_server_node", SERVER_PATH)
SERVER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SERVER
SPEC.loader.exec_module(SERVER)


def _hold_command(uav_ids=("A01", "A02"), command_id="hold-1"):
    return SERVER.parse_json_line({
        "version": "1.0",
        "type": "COMMAND",
        "mission_id": "mission-1",
        "group_id": "GroupA",
        "command_id": command_id,
        "command": "HOLD",
        "timeout_s": 5.0,
        "assignments": [{"uav_id": uav_id} for uav_id in uav_ids],
        "reason": "manual hold",
    })


class TcpHoldSemanticsTest(unittest.TestCase):
    def test_hold_json_parses(self):
        command = _hold_command(["A03"])
        self.assertEqual(command.command, "HOLD")
        self.assertEqual(command.uav_ids, ["A03"])
        self.assertEqual(command.reason, "manual hold")

    def test_hold_rejects_duplicate_uav(self):
        with self.assertRaises(SERVER.CommandValidationError) as ctx:
            _hold_command(["A01", "A01"])
        self.assertEqual(ctx.exception.error_code, "DUPLICATE_UAV")

    def test_accepted_true_builds_completed(self):
        command = _hold_command(["A01", "A02"])
        payload = SERVER._hold_terminal_payload(command, True, "all good")
        self.assertEqual(payload["status"], "COMPLETED")
        self.assertEqual(payload["completed_uavs"], ["A01", "A02"])
        self.assertNotIn("failed_uavs", payload)
        self.assertEqual(payload["message"], "all good")

    def test_accepted_false_builds_failed_hold_rejected(self):
        command = _hold_command(["A01", "A02"])
        payload = SERVER._hold_terminal_payload(
            command, False, "hold unconfirmed for: A02"
        )
        self.assertEqual(payload["status"], "FAILED")
        self.assertEqual(payload["error_code"], "HOLD_REJECTED")
        self.assertEqual(payload["failed_uavs"], ["A01", "A02"])
        self.assertNotIn("completed_uavs", payload)
        # backend 原因保留在 message/detail（§4.1 step 3）。
        self.assertIn("hold unconfirmed for: A02", payload["message"])

    def test_service_exception_builds_failed_hold_service_failed(self):
        command = _hold_command(["A01"])
        payload = SERVER._hold_terminal_payload(
            command, False, "backend down", service_error="service call timed out"
        )
        self.assertEqual(payload["status"], "FAILED")
        self.assertEqual(payload["error_code"], "HOLD_SERVICE_FAILED")
        self.assertEqual(payload["failed_uavs"], ["A01"])
        # 同一 command_id 的缓存终态不可被后续回调覆盖（set_final 一次性）。
        cache = SERVER.CommandCache()
        cache.set_final("hold-1", payload)
        self.assertEqual(cache.get("hold-1").final_response, payload)

    def test_completed_uses_original_assignments_only(self):
        # completed_uavs/failed_uavs 只能来自该 HOLD 命令原 assignments 的 Axx。
        command = _hold_command(["A01"])
        payload = SERVER._hold_terminal_payload(command, True, "")
        self.assertEqual(payload["completed_uavs"], ["A01"])


if __name__ == "__main__":
    unittest.main()
