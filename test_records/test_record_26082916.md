# 功能包参数配置整理与规范化测试记录

时间：2026-08-29 17:00-18:30 CST
记录人：cline-deepseek
对应实施计划：`plans/implementation_plan_26082916.md`（功能包参数配置整理与规范化）

实施前基线（所有嵌套仓库 HEAD）：

| 仓库 | HEAD |
|---|---|
| ego_planner_driver | 44c2b7e |
| swarm_uav_executor | eb753e1 |
| safe_valley_exp | 2add6f5 |
| swarm_topology_bridge | 56aa4e8 |
| tcp_to_ros | 8549d69（工作区含未提交 plans/README.md + 新计划文件） |
| swarm_uav_interfaces | 6ab1619 |

## 1. 参数真源（inventory/schema）

完整 inventory：`.tmp/notes/parameter_inventory_26082916.md`。来源优先级：
**canonical YAML → launch deployment override → 源码校验**。

| canonical 文件 | owner 包 | 内容 |
|---|---|---|
| `config/planner_defaults.yaml` | ego_planner_driver | 算法/执行/yield 默认（lambda_feasibility 统一 0.5） |
| `config/safety_defaults.yaml` | ego_planner_driver | Safety/TTC/stale/emergency 默认 |
| `config/resource_limits.yaml` | ego_planner_driver | 四项资源上限 + 方向 epsilon |
| `config/executor_defaults.yaml` | swarm_uav_executor | 仅通用 executor 参数（ego_swarm/mavros_position 段迁出） |
| `config/ego_swarm_defaults.yaml` | swarm_uav_executor | EGO/PI/densify 参数（waypoint_densify_spacing authoritative 3.0） |
| `config/mavros_position_legacy.yaml` | swarm_uav_executor | legacy MAVROS 参数 |
| `config/gps_bias_defaults.yaml` | safe_valley_exp | window_s/publish_rate_hz/lockout_s |
| `config/topology_group_a_sim.yaml` | swarm_topology_bridge | Group A sim topology 唯一 owner（自 tcp_to_ros 迁移） |
| `config/backend_defaults.yaml` | tcp_to_ros | backend/link health 默认（link 10/15/3 生产值） |
| `config/transport_defaults.yaml` | tcp_to_ros | TCP host/port/action/state/hold 默认 |

## 2. 修改文件（对应计划 §4/§5/§6/§7）

### ego_planner_driver
- 新增 `config/{planner,safety,resource_limits}_defaults.yaml`；`launch/ego_planner_driver.launch`
  只加载三份 YAML + identity/path override（删除全部重复业务默认与假配置）。
- `ego_planner_core.h`：删无效 `planning_horizon` 字段；`lambda_feasibility` 默认统一 0.5。
- `ego_planner_driver_node.h/.cpp`：接入 `order`/`lambda_terminal`/`feasibility_tolerance`/
  `plan_fail_timeout_s`/`replan_failure_fatal`/`yield_hovering_velocity_mps`/
  `emergency_minimum_duration_s`/`emergency_maximum_duration_s`/`neighbor_intent_stale_s`；
  `kNeighborStaleS=30` 硬编码替换为 `neighbor_intent_stale_s_`；集中语义校验
  （finite/正数/TTC 单调/emergency max>=min/枚举白名单）fail-fast，不 clamp、不降级。
- `yield_candidates.h/.cpp`：`YieldCandidateConfig.swarm_clearance`；gate 用统一 clearance
  （`clearance_factor*2*swarm_clearance`，不再写死 0.5）。
- `CMakeLists.txt`：加 `install(DIRECTORY launch/ config/)`。

### swarm_uav_executor
- `config/executor_defaults.yaml` 删 ego_swarm/mavros_position 段；新增 `ego_swarm_defaults.yaml`
  （含 waypoint_densify_spacing=3.0）、`mavros_position_legacy.yaml`。
- `launch/uav_executor_ego.launch`：planner 三份 YAML + executor/ego_swarm YAML 加载；
  删重复默认与 `neighbor_stale_policy`/`neighbor_missing_policy` 假配置。
- `launch/uav_executor_mavros.launch`：加载 `mavros_position_legacy.yaml`。
- `scripts/setpoint_relay_node.py`：rate/timeout finite/正数校验 fail-fast。
- `test/test_ego_launch_config.py`：改为 canonical YAML 来源断言 + 假配置/launch 重复检查。

### safe_valley_exp
- 删除 flock 示例（用户确认边界）：`flock.yaml`/`obstacles.yaml`/`flock_*.py`/
  `safe_flock_main.py`/`safe_flock_{sim,real}.launch`/`uav_offboard_{sim,real}.launch`/
  `startup_safe_valley.sh`；CMakeLists/README 删 flock 引用。保留 wait_mavros/submode_publisher
  /rosbag_record 与 GPS bias/offboard EGO/15-UAV 链路。
- 新增 `config/gps_bias_defaults.yaml`；`gps_bias_node.py` 读取/校验参数；
  `launch/uav_offboard_ego.launch` 加载 YAML、删错误 watchdog 注释；wiring 测试更新。

### swarm_topology_bridge
- `config/topology_group_a_sim.yaml` 新增（自 tcp_to_ros 迁移，唯一 Group A sim owner）；
  `launch/topology_group_a_sim.launch` 新增。

### tcp_to_ros
- 删 `config/topology_group_a_sim.yaml`（迁移 bridge）；`launch/topology_group_a_sim.launch`
  改为 include bridge wrapper（保持 run.sh/uav_offboard_ego.launch/README 兼容）。
- 新增 `config/backend_defaults.yaml`、`config/transport_defaults.yaml`；
  `launch/gcs_a_backend*.launch` 共用 backend_defaults.yaml（差异仅故障注入）；
  `launch/tcp_to_ros_mock.launch` 加载 transport_defaults.yaml。
- `scripts/gcs_a_backend_node.py`：link fallback 同步生产值 10/15/3。
- 新增 `test/test_parameter_wiring.py`（identity/topology/backend/transport 接线）；
  CMakeLists 注册。

## 3. 测试与验收（计划 §9）

| 层级 | 命令/方法 | 结果 |
|---|---|---|
| 参数单测 | `catkin run_tests ego_planner_driver swarm_uav_executor safe_valley_exp tcp_to_ros` | 各包全过 |
| launch 静态 | `test_ego_launch_config.py`/`test_parameter_wiring.py`/`test_gps_bias_wiring.py` | 通过 |
| 静态 lint | grep 残留：planning_horizon/stale 假配置/flock 引用 | 已清理 |
| runtime smoke | roscore + `ego_planner_driver.launch`；`rosparam dump` 快照 | 15 项参数全 OK（见 §4） |
| 非法值 fail-fast | `_emergency_ttc_s:=5.0 _warning_ttc_s:=3.0` 启动 | FATAL `refusing to start` |
| 全量回归 | `catkin build/run_tests -j1 -p1`（6 包） | **317 tests 0 failures** |
| 受保护 prepare 回归 | headless 15-UAV 全栈（sim/onboard/gcs/tcp/takeoff/calibration/task） | **SUCCESS**（failed=[]、ticks=1621） |

## 4. runtime 参数快照（planner smoke）

文件：`.tmp/parameter_snapshots/planner_smoke.yaml`（rosparam dump）。
关键值全部来自 canonical YAML：swarm_clearance=0.5、lambda_feasibility=0.5、
lambda_terminal=1.0、neighbor_intent_stale_s=30.0、yield_hovering_velocity_mps=0.2、
emergency_min/max_duration=0.4/12.0、setpoint_rate_hz=30.0、protected_distance_m=1.0、
plan_fail_timeout_s=10.0、replan_failure_fatal=true、max_arc_samples=4096、order=3、
collision_check_rate_hz=10.0、safety_supervisor_mode=active。
`neighbor_stale_policy`/`planning_horizon` 参数树中不存在。

## 5. 提交

| 仓库 | 提交 | 内容 |
|---|---|---|
| ego_planner_driver | （实施提交） | canonical YAML + 参数接入/校验 |
| swarm_uav_executor | （实施提交） | EGO/mavros YAML 拆分 + relay 校验 |
| safe_valley_exp | （实施提交） | flock 清理 + gps_bias YAML |
| swarm_topology_bridge | （实施提交） | topology 迁移 |
| tcp_to_ros | （实施提交） | backend/transport/topology 接线 |

## 6. 未验证项

- **受保护 prepare 回归**：headless 15-UAV 全栈执行 **SUCCESS**（failed=[]、ticks=1621，
  planner RSS 11.6-12.4 MB、calibration ready 15/15、UAV1 COMPLETED），1.0m 三维门禁 /
  Safety active / 30Hz / 0.2s / 0.1s / bridge-GPS-intent 语义无回归。
- PX4 参数 `COM_RCL_EXCEPT`/`NAV_RCL_ACT` 实机生效（仿真不产生 RC 告警差异）。
- headless 流程备注：gcs 阶段 bridge/backend 两 launch 在空 master 上并发 auto-start
  存在竞态（bridge 可能 port-in-use 死亡）→ 需先起一个建立 master 再起另一个，或
  gcs 后检查 `swarm_bridge` 存在并用 `gcs_bridge_only` 重启；calibration 要求 UAV
  disarm 时完成（takeoff 前），否则 backend `require_calibration_ready=true` 拒单。
