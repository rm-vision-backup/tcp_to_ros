#!/usr/bin/env python3
"""Canonical parameter / topology / identity wiring tests (implementation_plan_26082916 §7/§9).

验证：
- uav_identity_map.yaml 为唯一 Axx↔UAVn 映射（A01-A15 / UAV1-UAV15，无重复/缺失/跳号）；
- backend_defaults.yaml 含生产确认 link/confirm/hold 默认；transport_defaults.yaml 含 TCP 默认；
- gcs_a_backend*.launch 共用 backend_defaults.yaml；tcp_to_ros_mock.launch 加载 transport；
- Group A sim topology 唯一 owner = swarm_topology_bridge，tcp_to_ros 不再保存重复内容。
"""
import os
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml

TCP_DIR = Path(__file__).resolve().parents[1]
CONFIG = TCP_DIR / "config"
LAUNCH = TCP_DIR / "launch"
BRIDGE_DIR = TCP_DIR.parents[0] / "swarm_topology_bridge"


def _load_yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


class ParameterWiringTest(unittest.TestCase):
    def test_identity_map_is_complete_and_uniform(self):
        data = _load_yaml(CONFIG / "uav_identity_map.yaml")
        uavs = data["uavs"]
        self.assertEqual(len(uavs), 15)
        uav_ids = [item["uav_id"] for item in uavs]
        self.assertEqual(uav_ids, [f"A{i:02d}" for i in range(1, 16)],
                         "A01..A15 必须连续无重复/跳号")
        self.assertEqual(len(set(uav_ids)), 15, "uav_id 不得重复")
        exec_targets = [item["exec_target"] for item in uavs]
        self.assertEqual(exec_targets, [f"UAV{i}" for i in range(1, 16)],
                         "exec_target 必须 UAV1..UAV15")
        self.assertEqual(len(set(exec_targets)), 15, "exec_target 不得重复")

    def test_backend_defaults_hold_production_link_values(self):
        data = _load_yaml(CONFIG / "backend_defaults.yaml")
        self.assertEqual(data["link_suspect_after_s"], 10.0)
        self.assertEqual(data["link_lost_after_s"], 15.0)
        self.assertEqual(data["link_healthy_window_s"], 3.0)
        for key in ("confirm_wait_s", "confirm_retries", "confirm_retry_interval_s",
                    "confirm_deadline_s", "hold_batch_size", "hold_retry_count",
                    "hold_retry_interval_s", "action_name",
                    "emergency_hold_service", "require_calibration_ready"):
            self.assertIn(key, data, f"backend_defaults.yaml 缺少 {key}")

    def test_transport_defaults(self):
        data = _load_yaml(CONFIG / "transport_defaults.yaml")
        self.assertEqual(data["port"], 39001)
        self.assertEqual(data["host"], "0.0.0.0")
        self.assertEqual(data["action_name"], "/group_a/execute_task")
        self.assertEqual(data["state_topic"], "/group_a/group_task_state")
        self.assertEqual(data["emergency_hold_service"], "/group_a/emergency_hold")

    def test_backend_launches_load_canonical_yaml(self):
        for name in ("gcs_a_backend.launch", "gcs_a_backend_test.launch",
                     "gcs_a_backend_fail.launch"):
            root = ET.parse(str(LAUNCH / name)).getroot()
            node = next(node for node in root.findall(".//node")
                        if node.get("type") == "gcs_a_backend_node.py")
            loads = [item.get("file", "") for item in node.findall("rosparam")
                     if item.get("command") == "load"]
            self.assertTrue(any("backend_defaults.yaml" in f for f in loads),
                            f"{name} 必须加载 backend_defaults.yaml")

    def test_mock_launch_loads_transport_defaults(self):
        root = ET.parse(str(LAUNCH / "tcp_to_ros_mock.launch")).getroot()
        node = next(node for node in root.findall(".//node")
                    if node.get("type") == "tcp_server_node.py")
        loads = [item.get("file", "") for item in node.findall("rosparam")
                 if item.get("command") == "load"]
        self.assertTrue(any("transport_defaults.yaml" in f for f in loads))

    def test_topology_single_owner_in_bridge(self):
        # 唯一 Group A sim topology 在 swarm_topology_bridge；tcp_to_ros 不再保存重复。
        self.assertTrue((BRIDGE_DIR / "config" / "topology_group_a_sim.yaml").exists())
        self.assertFalse((CONFIG / "topology_group_a_sim.yaml").exists())
        bridge_data = _load_yaml(BRIDGE_DIR / "config" / "topology_group_a_sim.yaml")
        names = [item["name"] for item in bridge_data["uavs"]]
        self.assertIn("GCS_A", names)
        self.assertEqual(len(names), 16)
        self.assertTrue(all(f"UAV{i}" in names for i in range(1, 16)))

    def test_tcp_topology_launch_includes_bridge_wrapper(self):
        root = ET.parse(str(LAUNCH / "topology_group_a_sim.launch")).getroot()
        includes = [item.get("file", "") for item in root.findall("include")]
        self.assertTrue(any("swarm_topology_bridge" in f for f in includes),
                        "tcp wrapper 必须 include bridge launch")


if __name__ == "__main__":
    unittest.main()
