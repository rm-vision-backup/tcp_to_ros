# 功能包参数配置整理与规范化计划

**计划日期：** 2026-08-29 16:00（文件生成时间，24 小时制）  
**状态：** 计划已建立，未实施  
**适用工作区：** `/home/ub20tg/catkin_swarm6-2`

## 1. 背景问题

当前参数来源分散在源码 fallback、多个 launch、YAML、wrapper launch 和文档中。同一参数存在多份默认值，导致不同启动路径行为漂移；部分参数在 launch 中声明但源码不读取；部分源码字段和安全常量没有配置入口，调参必须改源码并重新编译；部分 YAML 会被 launch 二次覆盖。

已确认的代表性问题：

- `ego_planner_driver` 源码、独立 launch、`uav_executor_ego.launch` 重复保存 planner 默认值；`lambda_feasibility` 存在 1.0/0.5/未声明漂移。
- `neighbor_intent_stale_s`、`neighbor_stale_policy` 在 launch 中存在，但源码未读取，实际仍使用硬编码 stale 时间。
- `order`、`lambda_terminal`、`plan_fail_timeout_s`、`replan_failure_fatal`、EmergencyBrake 时长、yield 悬停阈值等没有完整 ROS 参数入口。
- `executor_defaults.yaml` 与 EGO launch 的 `waypoint_densify_spacing` 不一致；legacy MAVROS 参数混入 EGO 默认文件。
- `safe_valley_exp` 含已完成的 flock 示例算法，相关源码/配置/launch/文档与当前工作区项目无关；GPS bias 参数仍需保留并规范化。
- topology、backend、TCP server 和 identity 配置存在重复维护；runtime JSON 与 ROS tuning 的职责边界不清。
- 普通数值参数缺少统一 finite、范围和枚举校验，错误值可能依赖运行时 clamp 或静默 fallback。

## 2. 参数真源和命名规则

### 要改什么

建立每个功能域唯一的 authoritative default，清理重复默认值和无效参数声明。

### 怎么改

1. 建立参数 inventory/schema，总表记录：参数名、类型、单位、默认值、范围、namespace、canonical 文件、读取节点、实际消费者、安全等级、是否允许 launch override。
2. 按四类登记参数：部署参数、算法调参、安全参数、冻结内部常量。
3. 规定来源优先级：

   ```text
   canonical YAML → 明确的 launch deployment override → 源码校验
   ```

4. 源码不再保存与 YAML 不同的业务默认；若必须保留 fallback，必须与 YAML 同值并由检查阻止漂移。
5. 统一单位后缀：`_s`、`_m`、`_hz`、`_mps`、`_mps2`、`_mps3`、`_rad`。
6. 明确 `uav_id`/`exec_target` 的公开 identity namespace 与 planner/safety 私有 tuning namespace。
7. 对每个 launch 参数标明：是否只是 identity/path/environment override；其它默认值从 YAML 加载。
8. 将当前实验边界写入 schema 校验：A01–A15/UAV1–UAV15，较小编队从 1 连续编号。

## 3. Canonical 配置文件

### 要改什么

为各功能域建立唯一配置文件；launch 不再保存第二套业务默认值。

### 文件级改动

| 文件 | 动作 | 内容 |
|---|---|---|
| `src/ego_planner_driver/config/planner_defaults.yaml` | 新增 | planner、replan、到达、yield 默认值 |
| `src/ego_planner_driver/config/safety_defaults.yaml` | 新增 | Safety、TTC、stale、emergency 默认值 |
| `src/ego_planner_driver/config/resource_limits.yaml` | 新增 | 采样、轨迹、intent 资源上限 |
| `src/swarm_uav_executor/config/executor_defaults.yaml` | 整理 | 通用 executor 默认值 |
| `src/swarm_uav_executor/config/ego_swarm_defaults.yaml` | 新增 | EGO/PI/densify 默认值 |
| `src/swarm_uav_executor/config/mavros_position_legacy.yaml` | 新增/迁移 | legacy MAVROS 默认值 |
| `src/safe_valley_exp/config/gps_bias_defaults.yaml` | 新增 | GPS bias 三个运行参数 |
| `src/swarm_topology_bridge/config/topology_<environment>.yaml` | 整理/新增 | 各环境唯一 topology |
| `src/tcp_to_ros/config/backend_defaults.yaml` | 新增 | backend/link health 默认值 |
| `src/tcp_to_ros/config/transport_defaults.yaml` | 新增 | TCP server 默认值 |

### 处理规则

- `swarm_topology_bridge` 拥有完整 topology；`tcp_to_ros` 删除重复 topology 内容。
- `tcp_to_ros/config/uav_identity_map.yaml` 保留为唯一 Axx↔UAVn 映射文件。
- `references/` 不修改；runtime JSON 不承载 ROS tuning。
- 具体 YAML 路径先核对 catkin install，再写 launch。

## 4. `ego_planner_driver` 参数接线

### 要改什么

补齐 planner 参数入口，删除无效入口，统一默认值，增加语义校验。

### 文件级改动

| 文件 | 动作 | 内容 |
|---|---|---|
| `ego_planner_driver/config/planner_defaults.yaml` | 新增/填写 | `ts`、速度/加速度、planning range、代价权重、LBFGS、replan、arrival、takeoff、yield |
| `ego_planner_driver/config/safety_defaults.yaml` | 新增/填写 | Safety mode、protected distance、TTC、stale/pose timeout、emergency |
| `ego_planner_driver/config/resource_limits.yaml` | 新增/填写 | 四项资源上限 |
| `src/ego_planner_driver/src/ego_planner_driver_node.cpp/.h` | 修改 | 读取新增参数、集中校验、传递配置 |
| `src/ego_planner_driver/include/.../ego_planner_core.h` | 修改 | 接入 `order`/相关 schema，删除无效 `planning_horizon` 运行入口 |
| `src/ego_planner_driver/src/ego_planner_core.cpp` | 修改 | 使用明确配置或登记冻结常量 |
| `src/ego_planner_driver/src/yield_candidates.cpp`及配置头文件 | 修改 | yield gate 使用统一 clearance |
| `src/ego_planner_driver/src/emergency_brake.cpp`及配置头文件 | 修改 | 接入 emergency 最小/最大时长 |
| `src/ego_planner_driver/launch/ego_planner_driver.launch` | 修改 | 只加载三份 YAML，保留 identity/path override |
| `src/ego_planner_driver/test/*` | 新增/修改 | 参数入口、非法值、stale、yield、emergency 回归 |

### 参数动作

- 接入 ROS 参数：`order`、`lambda_terminal`、`replan_failure_fatal`、`plan_fail_timeout_s`、`yield_hovering_velocity_mps`、`emergency_minimum_duration_s`、`emergency_maximum_duration_s`。
- 删除 `planning_horizon` 的运行配置和 launch 声明；在迁移表标记旧版字段。
- 接入 `neighbor_intent_stale_s` 实际 stale 判断；删除未实现的 `neighbor_stale_policy`、`neighbor_missing_policy` 声明。
- 保留 `peer_data_age_cap_s` 为诊断参数，schema/README 写明不参与安全决策。
- 统一 `lambda_feasibility` authoritative default；standalone 与 wrapper 使用同一值。
- `waypoint`/采样等内部硬编码逐项登记：可调项进入 YAML，保护性常量保留在代码并写入 schema。

### 校验规则

- 所有浮点参数先检查 finite。
- 时间、频率、速度、距离、TTC 按语义检查非负/正数；允许 0 的参数单独登记。
- TTC 检查 `emergency_ttc_s <= warning_ttc_s`。
- 资源参数检查整数、非零、编译上限和 checked arithmetic。
- 枚举只接受白名单；不使用无依据的统一最大值。
- 非法配置启动 fail-fast。

## 5. `swarm_uav_executor` 参数接线

### 要改什么

统一 EGO wrapper 参数来源，拆分通用 executor 与 legacy MAVROS 配置。

### 文件级改动

| 文件 | 动作 | 内容 |
|---|---|---|
| `swarm_uav_executor/config/executor_defaults.yaml` | 修改 | 只保留通用 executor 参数 |
| `swarm_uav_executor/config/ego_swarm_defaults.yaml` | 新增/填写 | EGO、PI、densify 参数 |
| `swarm_uav_executor/config/mavros_position_legacy.yaml` | 新增/迁移 | legacy driver 参数 |
| `swarm_uav_executor/launch/uav_executor_ego.launch` | 修改 | 加载 executor/EGO 及 planner canonical YAML，删除重复默认 |
| `swarm_uav_executor/launch/uav_executor_mavros.launch` | 修改 | 只加载 legacy 配置和部署 override |
| `swarm_uav_executor/src/.../ego_swarm.py` | 修改 | 读取 canonical 参数，删除不同业务 fallback |
| `swarm_uav_executor/src/.../setpoint_relay_node.py` | 修改 | rate/timeout 参数校验 |
| `swarm_uav_executor/test/*` | 新增/修改 | 参数接线、默认一致、非法值回归 |

### 参数动作

- `waypoint_densify_spacing` authoritative default 定为 `3.0 m`；写入 `ego_swarm_defaults.yaml`，删除 launch 重复值，同步源码 fallback。
- EGO wrapper 加载 planner 的 `max_acc`、`acceleration_limit`、time reparam、feasibility、LBFGS、emergency 参数。
- `configure_px4_params`、PX4 参数超时、`COM_RCL_EXCEPT`、`NAV_RCL_ACT` 放入部署配置，登记读取节点。
- `require_armed`、`require_offboard`、`auto_arm`、`auto_offboard` 放入明确的实验/实机配置入口。
- relay rate、candidate timeout、pose timeout 增加 finite/范围校验。
- follower/monitor 的 10/20/30 Hz：需要调节的进入 YAML；不需要调节的登记为冻结常量。

## 6. `safe_valley_exp` 参数接线与示例清理

### 要改什么

保留 GPS bias/offboard 支持；删除与当前项目无关的 flock 示例。

### 文件级改动

| 文件/路径 | 动作 |
|---|---|
| `safe_valley_exp/config/flock.yaml` | 删除 |
| `safe_valley_exp/config/obstacles.yaml` | 删除 |
| `safe_valley_exp/scripts/flock_*.py` | 删除 |
| `safe_valley_exp/scripts/safe_flock_main.py` | 删除 |
| `safe_valley_exp/launch/safe_flock_*.launch` | 删除 |
| `safe_valley_exp/startup_safe_valley.sh` | 删除 |
| `safe_valley_exp/CMakeLists.txt`、README、安装规则 | 删除 flock 引用 |
| `safe_valley_exp/config/gps_bias_defaults.yaml` | 新增 |
| `safe_valley_exp/launch/uav_offboard_ego.launch` | 加载 GPS bias YAML、删除错误 watchdog 注释 |
| `safe_valley_exp/scripts/gps_bias_node.py` | 读取/校验 GPS bias 参数 |
| `safe_valley_exp/test/test_gps_bias*.py` | 增加 YAML 接线测试 |

### 参数动作

```text
window_s
publish_rate_hz
lockout_s
```

唯一来源为 `gps_bias_defaults.yaml`。不得删除 `gps_bias_node.py`、offboard EGO launch、15-UAV 启动脚本或 GPS bias 测试。

## 7. topology、identity、backend、runtime 配置

### 要改什么

为 topology、identity、backend、TCP transport 各指定唯一 owner。

### 文件级改动

| 文件/路径 | 动作 | 最终 owner/内容 |
|---|---|---|
| `swarm_topology_bridge/config/topology_<environment>.yaml` | 整理/新增 | 完整环境 topology |
| `swarm_topology_bridge/launch/*` | 修改 | 加载 bridge topology |
| `tcp_to_ros/config/topology_group_a_sim.yaml` | 删除重复内容/迁移引用 | 不再作为完整 topology owner |
| `tcp_to_ros/config/uav_identity_map.yaml` | 保留/校验 | 唯一 Axx↔UAVn 映射 |
| `tcp_to_ros/config/backend_defaults.yaml` | 新增 | backend/link health 默认 |
| `tcp_to_ros/config/transport_defaults.yaml` | 新增 | TCP host/port/action/state/hold 默认 |
| `tcp_to_ros/launch/gcs_a_backend*.launch` | 修改 | 共用 backend 基础配置，差异仅故障注入 |
| `tcp_to_ros/launch/tcp_to_ros_mock.launch` | 修改 | 加载 transport 配置 |
| `tcp_to_ros/scripts/gcs_a_backend_node.py` | 修改 | 读取 canonical backend 参数 |
| `tcp_to_ros/scripts/tcp_server_node.py` | 修改 | 读取 canonical transport 参数 |
| `tcp_to_ros/test/*` | 新增/修改 | identity/topology/backend/transport 接线测试 |

### 参数动作

- topology schema 固定字段：环境、UAV、IP/master、端口、topic、service、拓扑关系。
- identity 只从 `uav_identity_map.yaml` 读取；检查 A01–A15/UAV1–UAV15 无重复、缺失、跳号。
- backend 默认统一为当前生产配置确认值：`link_suspect_after_s=10.0`、`link_lost_after_s=15.0`、`link_healthy_window_s=3.0`。
- TCP server 的 host、port、action、state、hold 从 `transport_defaults.yaml` 加载。
- production/test/fail launch 共用基础配置；故障注入参数单独命名。
- `references/` 不修改；runtime CLI 的 tick/TCP/timeout/max ticks 只登记，不写入任务 JSON。

## 8. Launch 与源码校验

### 要改什么

让 launch 只负责加载配置和部署差异；让源码集中校验并报告实际生效值。

### 怎么改

1. standalone planner、EGO wrapper、test launch 加载同一 planner/safety/resource 文件。
2. 删除 launch 中重复的算法默认值；保留 identity、路径、环境和明确 deployment override。
3. supported 参数未被源码读取 -> 启动 fail-fast。
4. deprecated 参数 -> 一次明确迁移 warning，不宣称已生效。
5. 集中参数读取/校验逻辑；保存启动参数摘要或 `rosparam dump`。
6. 静态检查源码读取表、schema、YAML、launch 和 README 的对应关系。
7. 原版参数迁移表固定分类：保留、重命名、替代、废弃、不适用；grid map/A*/prediction/object 不进入当前 planner 配置。

## 9. 测试与验收

### 要改什么

为每类参数增加“来源正确、实际生效、非法失败”证据。

### 怎么改

1. **参数单测**：默认加载、YAML override、缺失、NaN/Inf、边界、超界、枚举、依赖关系。
2. **launch/rostest**：standalone/wrapper 公共参数最终值一致；namespace、identity、YAML 加载正确。
3. **静态 lint**：重复 YAML key、未登记读取、未消费声明、launch 重复默认、fallback 漂移、README 缺项、单位错误、legacy 误加载、topology/identity 重复。
4. **runtime smoke**：保存实际参数快照；验证 planner、executor、relay、Safety、GPS bias 的关键参数确实被对应节点消费。
5. **当前配置矩阵**：完整检查 Group A A01–A15/UAV1–UAV15，以及从 1 开始的连续子集。
6. **系统回归**：运行相关包测试和受保护 prepare；记录配置快照、命令、结果和未验证项。

每个参数测试记录必须能够回答：

```text
改哪个文件 → ROS 参数名 → 哪个节点读取 → 哪个行为改变 → 非法值如何失败
```

## 10. 文档、提交与回滚

### 要改什么

同步参数文档、迁移表、实际值记录和嵌套仓库提交边界。

### 怎么改

1. 新增参数总表：名称、单位、范围、默认、来源、namespace、消费者、安全等级。
2. 各包 README 增加 YAML 调参示例；说明 identity/path 的 launch 入口。
3. 更新 `src/tcp_to_ros/plans/README.md` 和测试记录；验证完成后才更新状态。
4. 实施前检查所有嵌套 Git 仓库 HEAD/status；生成 inventory/schema 后再改接线。
5. 按顺序实施：`ego_planner_driver` → `swarm_uav_executor` → `safe_valley_exp`/topology/identity/backend → lint/test/文档。
6. 删除 fallback 或 launch 默认前，先用实际参数快照确认 canonical YAML 生效。
7. 按功能域在对应嵌套仓库分批提交，不 push；不覆盖用户已有修改。
8. 回归失败时回到最后一个已验证配置提交，保留可逆迁移入口。

## 11. 完成标准

计划完成必须同时满足：

- 已支持的调参行为无需修改源码即可通过 canonical YAML 或明确 launch override 调整。
- 不存在 launch 声明但源码不读取的已支持参数。
- planner standalone、executor wrapper、test launch 的公共默认一致，差异有记录。
- 每个参数只有一个 authoritative default。
- 安全关键参数有范围校验和实际生效测试。
- legacy 和不适用参数不会误加载。
- A01–A15/UAV1–UAV15 当前配置可完整加载，identity/topology 无冲突。
- 当前 1.0 m 三维门禁、Safety active、30 Hz、0.2 s candidate timeout、0.1 s TimedTrajectory、bridge/GPS/协议语义无回归。
- 测试记录包含实际参数快照、命令、结果、提交和未验证项。
