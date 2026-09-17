# 测试记录 26091103：实机部署适配（implementation_plan_26091101）实施与验收

时间：2026-09-11 03:30–03:56 CST（实施 + 自动测试 + 单轮 15-UAV prepare SITL 回归）
记录人：cline-deepseek
对应计划：`plans/implementation_plan_26091101.md`（原点校准集合参数化与实机多主部署适配）
工作空间：`/home/yjq/catkin_swarm6-2`

## 0. 摘要

- **实施范围**：校准集合参数化（执行层 UAVn + 启动校验 + ready 仅按必要集合 + 逐机 ready 日志）、
  身份映射扩展 A16→UAV16、实机生产 topology（GCS_A + UAV1–UAV16、offset 0、冻结 4200/14000）、
  测试拓扑隔离（`topology_test.yaml` + `test.launch`）、独立实机 launch
  （`uav_offboard_ego_real.launch` / `gcs_a_real.launch` / `bridge_group_a_real.launch`）、
  诊断日志路径参数化、机载 systemd service 文档（**2026-09-14 已删除**，见 §5 第 2 条）。
- **自动测试**：全工作区 `catkin_test_results --all build` = **367 tests, 0 errors, 0 failures, 0 skipped**。
- **SITL 回归**：官方 15 机（GCS_A + UAV1–UAV15）`prepare` **SUCCESS**，`failed=[] ticks=1648`（≈82s）。
- 未验证项：实机链路、A16 备用机的 runtime roster 扩展（属上游）、实机 FCU 参数现场核对。

## 1. 实施 commit（4 个嵌套仓库，均未 push）

| 仓库 | commit | 内容 |
|---|---|---|
| swarm_topology_bridge | `9ee6f58` | 生产 topology.yaml（GCS_A+UAV1–16、port_offset 0、4200/14000）；新增 topology_test.yaml；test.launch 改载测试拓扑；sim 拓扑加 UAV16（offset 1600、不启动）；新增 bridge_group_a_real.launch；README |
| swarm_uav_executor | `fc7c3b6` | 新增 uav_offboard_ego_real.launch（hostname 身份、实机 topology、显式 fcu_url）；offboard/executor launch 日志目录参数化；services/ systemd 单元 + README（**2026-09-14 删除**）；README |
| ego_planner_driver | `42bf262` | ego_planner_driver.launch diagnostic_log_dir 参数化 |
| tcp_to_ros | `db130c3` | 校准集合参数化/启动校验/必要集合 ready/逐机 ready 日志；identity map 扩 A16；backend map 路径去工作区绝对路径；calibration_defaults.yaml + gcs_a_real.launch；README/CONTEXT；测试 +13 |

## 2. 身份与配置来源

- 任务层 Axx：仅 `tcp_to_ros/config/uav_identity_map.yaml`（A01–A16 → UAV1–UAV16）。
- 机载执行层：UAVn（`uav_id`/`exec_target`/hostname/topic-service 路径）；未新增 Axx 反向映射。
- 校准集合：`tcp_to_ros/config/calibration_defaults.yaml`（`calibration_uavs`=UAV1–UAV6、
  `all_uavs`=UAV1–UAV16），启动校验（非空/无重复/子集/身份存在且 enabled）；ready 仅按 `calibration_uavs`。
- 实机拓扑：`swarm_topology_bridge/config/topology.yaml`（GCS_A 192.168.5.241、UAVn 192.168.5.(10n+1)）。
- 端口：`base_port 4200` / `service_base_port 14000`（实机 offset 0）；仿真 offset：GCS_A 0、UAVn 100n、UAV16 1600。

## 3. 构建与自动测试

```bash
source /opt/ros/noetic/setup.bash
catkin build tcp_to_ros swarm_topology_bridge swarm_uav_executor ego_planner_driver -j4   # 5 包成功
catkin run_tests -j3 -p1
catkin_test_results --all build
```

- 结果：**367 tests, 0 errors, 0 failures, 0 skipped**（基线 354 + 新增 13：校准校验 8 + 接线 5）。
- targeted：`PYTHONPATH=scripts pytest test/test_calibration.py test/test_parameter_wiring.py` → 26 passed。
- 日志：`.tmp/logs/build_26091101.log`、`.tmp/logs/tests_26091101.log`、`.tmp/logs/test_results_26091101.log`。

## 4. 15-UAV prepare SITL 回归

```bash
bash src/swarm_simulation/bashs/round_prepare_15sitl.sh \
  --round 1 --task prepare --timeout-s 150 --keep-ros-home-on-fail 1
```

- 结果：`status=SUCCESS failed=[] ticks=1648`（≈82s；成功轮 ticks 正常区间）。
- 阶段：sim 15 px4 → onboard 15/15 → gcs bridge→backend → `calibration_ready=true`（UAVn 校准集合）
  → tcp 39001 → takeoff 15/15 → prepare SUCCESS。
- 证据归档（新留档规则）：`.ros_home_round1_s`（482M，含 PX4 ulog/rosout/GCS 日志/07_task JSON）；
  编排日志 `.tmp/logs/round1_26091101.log`。
- 收口：测试后进程 0、端口（39001/11300/11310/11345/1131x）0；默认 `.ros_home` 已清空。

## 5. 未验证项

1. 实机链路：FCU 串口/UDP 参数、现场 hostname=UAVn、地址可达性未在本工作区验证（计划规定现场核对）。
2. systemd：该方案未在实机安装/启用，已于 2026-09-14 随 `swarm_uav_executor/services/` 一并删除（不设开机自启）；机载启停改用 `bashs/start_onboard.sh` / `bashs/stop_onboard.sh` 与 `tools/onboard_ctl.sh`。
3. A16 备用机：runtime roster/配置扩展属上游责任；本工作区仅完成 A16→UAV16 映射、解析与路由能力。
4. 实机多机分批上电：仿真以“拓扑含 UAV16 但不启动”覆盖缺席机不阻塞 ready 的同一轮；实机分批流程需现场演练。
5. 实机 arm/OFFBOARD/起飞：按计划由现场遥控器完成，未纳入本工作区自动测试。

## 6. 关联

- 计划：`plans/implementation_plan_26091101.md`；状态登记：`plans/README.md`。
- 日志留档规则：`test_records/README.md` §8。
