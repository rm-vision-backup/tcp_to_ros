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

try:
    import yaml
except ImportError:
    raise ImportError(
        "PyYAML (python3-yaml) 未安装；test_parameter_wiring.py 依赖它解析 "
        "canonical YAML（implementation_plan_26083018 §6）。请先安装："
        "sudo apt install python3-yaml"
    ) from None

TCP_DIR = Path(__file__).resolve().parents[1]
CONFIG = TCP_DIR / "config"
LAUNCH = TCP_DIR / "launch"
BRIDGE_DIR = TCP_DIR.parents[0] / "swarm_topology_bridge"


def _load_yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _param_if(node, name):
    for item in node.findall("param"):
        if item.attrib.get("name") == name:
            return item.attrib.get("if")
    return None


class ParameterWiringTest(unittest.TestCase):
    def test_identity_map_is_complete_and_uniform(self):
        data = _load_yaml(CONFIG / "uav_identity_map.yaml")
        uavs = data["uavs"]
        self.assertEqual(len(uavs), 16)
        uav_ids = [item["uav_id"] for item in uavs]
        self.assertEqual(uav_ids, [f"A{i:02d}" for i in range(1, 17)],
                         "A01..A16 必须连续无重复/跳号")
        self.assertEqual(len(set(uav_ids)), 16, "uav_id 不得重复")
        exec_targets = [item["exec_target"] for item in uavs]
        self.assertEqual(exec_targets, [f"UAV{i}" for i in range(1, 17)],
                         "exec_target 必须 UAV1..UAV16")
        self.assertEqual(len(set(exec_targets)), 16, "exec_target 不得重复")
        self.assertEqual(len({item["ip"] for item in uavs}), 16, "实机 IP 不得重复")

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

    def test_backend_launches_use_conditional_business_params(self):
        # implementation_plan_26083018 §3：业务参数（action_name/emergency_hold_service/
        # confirm_*）arg 默认空，param 仅在命令行显式传入时条件写入；YAML 是唯一默认来源。
        for name in ("gcs_a_backend.launch", "gcs_a_backend_test.launch",
                     "gcs_a_backend_fail.launch"):
            root = ET.parse(str(LAUNCH / name)).getroot()
            node = next(node for node in root.findall(".//node")
                        if node.get("type") == "gcs_a_backend_node.py")
            args = {item.get("name"): item.get("default", "")
                    for item in root.findall("arg")}
            for key in ("action_name", "emergency_hold_service",
                        "confirm_wait_s", "confirm_retries",
                        "confirm_retry_interval_s", "confirm_deadline_s"):
                self.assertEqual(args.get(key), "", f"{name} arg {key} 应空默认")
                self.assertIn(f"arg('{key}')", _param_if(node, key) or "",
                              f"{name} param {key} 应条件写入")
            if name == "gcs_a_backend.launch":
                self.assertEqual(args.get("require_calibration_ready"), "")
                self.assertIn("arg('require_calibration_ready')",
                              _param_if(node, "require_calibration_ready") or "")

    def test_mock_launch_transport_params_conditional(self):
        # transport 默认来自 transport_defaults.yaml；host/port/action/state/hold
        # 仅命令行显式传入时条件写入。
        root = ET.parse(str(LAUNCH / "tcp_to_ros_mock.launch")).getroot()
        node = next(node for node in root.findall(".//node")
                    if node.get("type") == "tcp_server_node.py")
        args = {item.get("name"): item.get("default", "")
                for item in root.findall("arg")}
        for key in ("host", "port", "action_name", "state_topic",
                    "emergency_hold_service"):
            self.assertEqual(args.get(key), "", f"arg {key} 应空默认")
            self.assertIn(f"arg('{key}')", _param_if(node, key) or "",
                          f"param {key} 应条件写入")

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
        self.assertEqual(len(names), 17)
        self.assertTrue(all(f"UAV{i}" in names for i in range(1, 17)))
        offsets = {item["name"]: int(item.get("port_offset", 0))
                   for item in bridge_data["uavs"]}
        self.assertEqual(offsets["UAV16"], 1600,
                         "仿真 UAV16 必须独立 port_offset（不与 GCS_A 同 offset）")
        self.assertEqual(len(set(offsets.values())), len(offsets),
                         "仿真 port_offset 不得重叠")

    def test_tcp_topology_launch_includes_bridge_wrapper(self):
        root = ET.parse(str(LAUNCH / "topology_group_a_sim.launch")).getroot()
        includes = [item.get("file", "") for item in root.findall("include")]
        self.assertTrue(any("swarm_topology_bridge" in f for f in includes),
                        "tcp wrapper 必须 include bridge launch")

    def test_calibration_defaults_use_execution_identity(self):
        # implementation_plan_26091101 §3.1：校准集合参数使用执行层 UAVn。
        data = _load_yaml(CONFIG / "calibration_defaults.yaml")
        self.assertEqual(data["calibration_uavs"],
                         [f"UAV{i}" for i in range(1, 7)])
        self.assertEqual(data["all_uavs"],
                         [f"UAV{i}" for i in range(1, 17)])
        self.assertTrue(set(data["calibration_uavs"]).issubset(set(data["all_uavs"])))
        self.assertTrue(all(item.startswith("UAV") for item in data["all_uavs"]),
                        "校准集合必须使用执行层 UAVn，不得使用 Axx")

    def test_backend_launch_wires_calibration_params(self):
        # implementation_plan_26091101 §3.3：集合/身份文件显式加载到校准节点。
        root = ET.parse(str(LAUNCH / "gcs_a_backend.launch")).getroot()
        node = next(node for node in root.findall(".//node")
                    if node.get("type") == "gcs_a_calibration_node.py")
        loads = [item.get("file", "") for item in node.findall("rosparam")
                 if item.get("command") == "load"]
        self.assertTrue(any("calibration_defaults.yaml" in f for f in loads),
                        "gcs_a_calibration 必须显式加载 calibration_defaults.yaml")
        params = {item.get("name"): item.get("value", "")
                  for item in node.findall("param")}
        self.assertIn("identity_file", params, "校准节点必须显式加载身份文件路径")

    def test_production_topology_is_real_deployment(self):
        # implementation_plan_26091101 §4.2：实机生产拓扑（GCS_A + UAV1..UAV16，offset=0）。
        data = _load_yaml(BRIDGE_DIR / "config" / "topology.yaml")
        names = [item["name"] for item in data["uavs"]]
        self.assertEqual(names[0], "GCS_A")
        self.assertEqual(len(names), 17)
        self.assertTrue(all(f"UAV{i}" in names for i in range(1, 17)))
        self.assertTrue(all(int(item.get("port_offset", 0)) == 0
                            for item in data["uavs"]), "实机所有节点 port_offset=0")
        self.assertEqual(data["base_port"], 4200)
        self.assertEqual(data["service_base_port"], 14000)
        services = [item["name"] for item in data["services"]]
        for i in range(1, 17):
            for suffix in ("uav_task", "uav_task_control", "uav_hold"):
                self.assertIn(f"/UAV{i}/{suffix}", services)
        self.assertFalse(any("/test_srv" in name for name in services),
                         "生产拓扑不得包含 /test_srv")
        self.assertEqual(set(data["topology"]), set(names),
                         "topology 接收矩阵必须覆盖全部节点")

    def test_test_launch_isolated_from_production_topology(self):
        # implementation_plan_26091101 §8：测试拓扑独立，生产替换不影响 /test_srv。
        root = ET.parse(str(BRIDGE_DIR / "launch" / "test.launch")).getroot()
        loads = [item.get("file", "") for item in root.findall(".//rosparam")
                 if item.get("command") == "load"]
        self.assertTrue(any("topology_test.yaml" in f for f in loads))
        self.assertFalse(any(f.endswith("config/topology.yaml") for f in loads),
                         "test.launch 不得加载生产 topology.yaml")
        test_topology = _load_yaml(BRIDGE_DIR / "config" / "topology_test.yaml")
        self.assertTrue(any(item["name"].startswith("/test_srv/")
                            for item in test_topology["services"]))

    def test_real_entry_launches_separate_from_sim(self):
        # implementation_plan_26091101 §5/§7：实机入口与仿真入口分离并实际接线。
        exec_dir = TCP_DIR.parents[0] / "swarm_uav_executor"
        real = ET.parse(str(exec_dir / "launch" / "uav_offboard_ego_real.launch")).getroot()
        includes = [item.get("file", "") for item in real.findall("include")]
        self.assertTrue(any("bridge_group_a_real.launch" in f for f in includes))
        real_args = {item.get("name"): item.get("default", "")
                     for item in real.findall("arg")}
        self.assertIn("topology_config", real_args)
        # FCU 链路（fcu_url / tgt_system）由机载 MAVROS launch 现场配置，实机入口不传。
        self.assertNotIn("fcu_url", real_args)
        self.assertNotIn("tgt_system", real_args)
        mavros_includes = [item for item in real.findall("include")
                           if item.get("file", "").endswith("mavros)/launch/px4.launch")]
        self.assertEqual(len(mavros_includes), 1)
        mavros_args = {arg.get("name") for arg in mavros_includes[0].findall("arg")}
        self.assertNotIn("fcu_url", mavros_args)
        self.assertNotIn("tgt_system", mavros_args)
        sim = ET.parse(str(exec_dir / "launch" / "uav_offboard_ego.launch")).getroot()
        sim_includes = [item.get("file", "") for item in sim.findall("include")]
        self.assertTrue(any("topology_group_a_sim.launch" in f for f in sim_includes))
        gcs_real = ET.parse(str(LAUNCH / "gcs_a_real.launch")).getroot()
        gcs_includes = [item.get("file", "") for item in gcs_real.findall("include")]
        self.assertTrue(any("bridge_group_a_real.launch" in f for f in gcs_includes))


if __name__ == "__main__":
    unittest.main()
