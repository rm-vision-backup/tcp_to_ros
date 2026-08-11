# Implementation Plan

[Overview]
修复 `ego_planner_driver` 状态机缺陷，使其在真实 MAVROS + PX4 SITL 多 Master 环境下稳定可执行，并配套解决 OFFBOARD setpoint 流缺失与多机坐标系统一问题。保留 bspline 轨迹优化内核。

依据 `docs/test_record_26081123.md`（场景 3 无头 15 机端到端测试）：链路层（TCP → GCS_A → bridge → executor → MAVROS → PX4）已贯通，但六批次应答仅 5-11/12 机成功，根因是 `ego_planner_driver` 状态机缺陷导致部分无人机（如 A05/A09）连续失败。

本计划三个缺陷与修复目标：
1. **IDLE 不发布 setpoint** → OFFBOARD 前无 setpoint 流，无人机无法进入 OFFBOARD 或切出后无法保持。参考 `safe_valley_exp` 的 `execute_hover()` 持续发布冻结位姿的做法，但改为**每秒更新一次 hover 目标**（而非仅模式切换时），以防从其他模式切 OFFBOARD 时 setpoint 剧烈波动。
2. **COMPLETED/HOLD 后不复位 IDLE** → 部分无人机（A05/A09）连续失败。采纳方案 A：HOLD/COMPLETED 保持 `steady_target_` 冻结发布 2 秒后自动回 IDLE，所有状态下都不停止发布 setpoint。
3. **多机坐标系不一致** → 各机以上电点为 local origin，不适用于编队需求。由 GCS_A 设置 GPS 全局原点并周期发布给所有无人机，每机起飞前持续设置 `set_gp_origin`（与 `safe_valley_exp/flock_comm.py::sync_origin()` 流程类似但更稳定：以 GCS_A 统一下发原点，而非依赖 leader 位置）。

启动顺序（参考 `safe_valley_exp` 算法先启动模式，去掉保活脚本）：
```
仿真层启动 → 机载层启动（MAVROS + ego driver + executor + bridge，ego driver 启动即 30Hz 发布当前位姿 setpoint）→ 全局原点统一 → 操作员/GCS 批量 arm + takeoff + 切 OFFBOARD → GCS_A 下发任务
```

arm+takeoff+OFFBOARD 自动化方案仅调研、本次不实现（见附录 A）。

[Types]

单句描述：`ego_planner_driver` 节点新增 IDLE hover 目标缓存与 terminal→IDLE 转换计时字段，`swarm_uav_interfaces` 新增全局原点消息。

### `ego_planner_driver_node.cpp` 新增成员

```cpp
// IDLE hover 目标（每秒更新为当前位姿，避免切 OFFBOARD 时 setpoint 跳变）
boost::optional<Eigen::Vector3d> idle_hover_target_;
bool idle_hover_target_valid_ = false;
ros::Time idle_hover_updated_at_;

// terminal(COMPLETED/HOLD) → IDLE 自动转换窗口
double terminal_to_idle_s_ = 2.0;   // 可配置，默认 2 秒
std::string scheduled_revert_state_; // "" 表示不调度
ros::Time terminal_entered_at_;
```

### 常量/配置

```cpp
constexpr double kSetpointRateHz = 30.0;          // 现有
constexpr double kIdleHoverRefreshHz = 1.0;       // 新增：IDLE hover 目标每秒刷新一次
constexpr double kTerminalToIdleS = 2.0;          // 新增：terminal → IDLE 自动转换窗口
```

### 全局原点消息

优先复用 `mavros_msgs/GeoPointStamped`（`geographic_msgs/GeoPoint` + header）作为全局原点消息，避免新增 `swarm_uav_interfaces` 消息。GCS_A 发布 `/group_a/gp_origin`（`GeoPointStamped`），机载侧接收后调用 `/mavros/global_position/set_gp_origin`。若实施中需要多机区分目标，可在自定义 `GpOrigin.msg`（含 `uav_id` 字段）与 GeoPointStamped 之间二选一，后者默认。

[Files]

单句描述：修改 `ego_planner_driver` C++ 节点实现状态机修复，新增 GCS_A 全局原点发布与机载原点接收两个轻量节点，更新拓扑桥配置与测试。

### 新建文件

- `src/tcp_to_ros/scripts/gp_origin_publisher.py`（GCS_A 侧）
  - 发布 `/group_a/gp_origin`（`mavros_msgs/GeoPointStamped`）
  - 读取 ROS 参数 `origin_lat/origin_lon/origin_alt`（默认湖南大学科技园附近 `28.2/112.9/50`）
  - 1Hz 周期发布，无人机起飞前持续设置
- `src/swarm_uav_executor/scripts/gp_origin_receiver.py`（机载侧，每机一个实例）
  - 经 bridge 订阅 `GCS_A/group_a/gp_origin`（跨 Master 桥接后命名）
  - 收到后调用 `/mavros/global_position/set_gp_origin`
  - 起飞前每秒重复设置（规避 RTK 延迟坐标突变）

### 修改文件

- `src/ego_planner_driver/src/ego_planner_driver_node.cpp`（核心修复）
  - `publishSetpoint()`：所有状态下持续 30Hz 发布（IDLE 时发布 `idle_hover_target_`，无 fresh pose 时不发布）
  - 新增 `refreshIdleHoverTarget()`：1Hz 更新 `idle_hover_target_` 为当前 local_pose
  - `checkArrival()` / `replanLoop()`：COMPLETED/HOLD 后调度 `terminal_to_idle_s_` 窗口，窗口结束自动 `setState(IDLE)` + 清理
  - 构造函数读取 `terminal_to_idle_s` / `idle_hover_refresh_hz` 参数
- `src/tcp_to_ros/config/topology_group_a_sim.yaml`
  - 新增 `GCS_A → UAV*` 的 `group_a/gp_origin` topic 桥接（`mavros_msgs/GeoPointStamped`）
- `src/safe_valley_exp/launch/uav_offboard_ego.launch`
  - 装配 `gp_origin_receiver` 节点（属于机载层）
- `src/ego_planner_driver/launch/ego_planner_driver.launch`
  - 暴露 `terminal_to_idle_s` / `idle_hover_refresh_hz` 参数
- `src/ego_planner_driver/test/test_trajectory.cpp`
  - 新增 IDLE setpoint 发布、COMPLETED→IDLE 转换、HOLD→IDLE 窗口用例
- `verification/ego_smoke/offboard_takeoff_15.py`
  - 移除 `--hold` 保活逻辑（仅做 arm+takeoff+OFFBOARD）
- `src/tcp_to_ros/plans/README.md`：新增本次计划索引 + 更新当前状态
- `src/tcp_to_ros/CONTEXT.md`：新增"全局原点"术语

### 删除/移动

- 不删除现有文件；`verification/ego_smoke/offboard_takeoff_15.py` 的 `--hold` 分支移除但保留 takeoff 脚本。

[Functions]

单句描述：修改 `publishSetpoint` 使其在所有状态下持续发布，新增 IDLE hover 目标刷新与 terminal→IDLE 自动转换；新增 GCS_A 原点发布与机载原点接收两个节点函数。

### 修改 `ego_planner_driver_node.cpp`

- `void publishSetpoint(const ros::TimerEvent&)`（核心修改）
  - 原有 `steady_target_` / `current_` 分支保留
  - 新增 IDLE 分支：`!steady_target_ && !current_` 时发布 `idle_hover_target_`（若 fresh pose 可用）
  - 检查 `scheduled_revert_state_`：若 terminal→IDLE 窗口到期，`steady_target_.reset()` + `setState(IDLE)` + 清空调度
- `void refreshIdleHoverTarget()`（新增，1Hz timer 或 `publishSetpoint` 内按时间戳限频）
  - 读取最新 `local_pose_`，更新 `idle_hover_target_`
  - 与 `safe_valley_exp` 的区别：**每秒更新一次 hover 目标**（而非仅模式切换时），防止从其他模式切 OFFBOARD 时 setpoint 剧烈波动
- `void scheduleTerminalToIdle(const std::string& terminal_state)`（新增）
  - COMPLETED/HOLD 进入时调用，记录 `terminal_entered_at_`
- 构造函数：读取 `~terminal_to_idle_s`（默认 2.0）与 `~idle_hover_refresh_hz`（默认 1.0）参数

### 新建 `gp_origin_publisher.py`

- `publish_origin_loop()`：1Hz 周期发布全局原点到 `/group_a/gp_origin`
- `main()`：初始化节点、读取原点参数、开始发布

### 新建 `gp_origin_receiver.py`

- `on_gp_origin(msg)`：收到原点 → 调用 `/mavros/global_position/set_gp_origin`
- `main()`：初始化节点、订阅 `GCS_A/group_a/gp_origin`、循环设置直到 disarm（起飞前持续）

[Classes]

单句描述：`EgoPlannerDriverNode` 增加 IDLE hover 目标缓存与 terminal→IDLE 自动恢复逻辑；新增 `GpOriginPublisher` 与 `GpOriginReceiver` 两个轻量 ROS 节点类。

### 修改 `EgoPlannerDriverNode`（`ego_planner_driver_node.cpp`）

- 新增成员：`idle_hover_target_`（boost::optional<Eigen::Vector3d>）、`idle_hover_updated_at_`、`scheduled_revert_state_`、`terminal_entered_at_`、`terminal_to_idle_s_`、`idle_hover_refresh_hz_`
- 新增方法：`refreshIdleHoverTarget()`、`scheduleTerminalToIdle()`
- 修改方法：`publishSetpoint()`（所有状态 30Hz 发布）、`checkArrival()`（COMPLETED 时调度→IDLE）、`replanLoop()`（HOLD 分支调度→IDLE）

### 新建 `GpOriginPublisher`（`src/tcp_to_ros/scripts/gp_origin_publisher.py`）

- 构造函数：读取 `~origin_lat/~origin_lon/~origin_alt`
- 方法：`publish_origin_loop()`（1Hz）

### 新建 `GpOriginReceiver`（`src/swarm_uav_executor/scripts/gp_origin_receiver.py`）

- 构造函数：订阅 `GCS_A/group_a/gp_origin`，注册 `/mavros/global_position/set_gp_origin` service proxy
- 方法：`on_gp_origin(msg)`

[Testing]

单句描述：新增 C++ 单元测试覆盖 IDLE setpoint 发布与 terminal→IDLE 转换窗口，重跑 catkin 全量测试与单 Master 冒烟；本次不做场景 3 重跑。

### 单元测试（`test_trajectory.cpp` 扩充）

- IDLE 状态下 `publishSetpoint` 以 ≥29Hz 发布 local_pose 位姿
- COMPLETED 后 2 秒内持续发布冻结位姿；>2 秒后 `exec_state` 回到 IDLE 且继续发布当前位姿
- HOLD 后 2 秒内持续发布冻结位姿；>2 秒后回 IDLE
- 无 fresh pose 时 IDLE 不发布 setpoint（fail-safe）

### 回归测试

- `catkin build ego_planner_driver swarm_uav_executor tcp_to_ros`
- `catkin_test_results --all build` 全量 0 failure
- 单 Master 冒烟（`smoke_single_master.py`）：IDLE→EXECUTING→COMPLETED→IDLE→HOLD→IDLE 完整生命周期

### 场景 3 重跑（后续，由用户确认）

- 15 机无头 SITL + 6 批次回测，确认 prepare/coverage/fault-exit/recovery/hold 全绿

[Dependencies]

单句描述：无新增第三方包；`tcp_to_ros` 与 `swarm_uav_executor` 需声明 `mavros_msgs`/`geographic_msgs` 依赖（若未声明）。

- `mavros_msgs`：`gp_origin_publisher.py` 发布 `GeoPointStamped`、`gp_origin_receiver.py` 调用 `set_gp_origin` service
- `geographic_msgs`：`GeoPointStamped` 依赖（`mavros_msgs` 传递引入，确认 `tcp_to_ros` CMake/package.xml 已声明）
- `swarm_uav_interfaces`：优先复用 `mavros_msgs/GeoPointStamped`，不新增消息
- 无 pip 全局包、无 Mavlink2/MavSDK 依赖（本次不实现 arm+takeoff+OFFBOARD 自动化）

[Implementation Order]

单句描述：按"状态机修复 → 坐标系统一 → 配置/launch → 测试"顺序推进，先单 Master 验证再回归。

1. `ego_planner_driver_node.cpp`：修改 `publishSetpoint()` 支持 IDLE hover 目标；新增 `refreshIdleHoverTarget()` 与 `scheduleTerminalToIdle()`
2. `ego_planner_driver.launch`：暴露 `terminal_to_idle_s` / `idle_hover_refresh_hz` 参数
3. `test_trajectory.cpp`：新增 IDLE setpoint、COMPLETED→IDLE、HOLD→IDLE 用例；`catkin build` + 单测通过
4. `gp_origin_publisher.py`（GCS_A 侧）与 `gp_origin_receiver.py`（机载侧）新建
5. `topology_group_a_sim.yaml`：新增 `GCS_A → UAV*` 的 gp_origin topic 桥
6. `uav_offboard_ego.launch`：装配 `gp_origin_receiver`
7. 全量 `catkin_test_results --all build` 回归 + 单 Master 冒烟
8. `verification/ego_smoke/offboard_takeoff_15.py`：移除 `--hold` 保活逻辑
9. 文档落盘：更新 `plans/README.md` 与 `CONTEXT.md`
10. （后续，由用户确认）场景 3 无头 15 机重跑验证

## 附录 A：arm+takeoff+OFFBOARD 自动化方案调研（非优先，本次不实现）

背景：仿真中手动启动 15 架飞机过于繁琐；实机中远程登录每台机载电脑操作不是最优解。调研以下四种方案：

| 方案 | 方式 | 适用性 | 备注 |
|---|---|---|---|
| **MAVROS service** | `rosservice call /mavros/cmd/arming` + `/mavros/set_mode custom_mode:=OFFBOARD` | SITL 仿真首选 | 已有 bridge 跨 Master 代理，GCS_A 可经 `/UAV{n}/mavros/cmd/arming` 远程调用；无需 SSH |
| **pymavlink** | `pymavlink.mavutil` 构造 Mavlink 2.0 `COMMAND_LONG`（MAV_CMD_COMPONENT_ARM_DISARM=400、MAV_CMD_NAV_TAKEOFF=22、MAV_CMD_DO_SET_MODE=176） | 仿真 + 实机通用 | 需要每架机 target_system 与 UDP 端口；Mavlink 2.0 支持多机（不同 system id） |
| **MAVSDK-Python** | `drone.action.arm()` → `takeoff()` → `offboard.start()` | 实机首选 | 高层 API，抽象 MAVLink 细节，带异步/重试；每机独立 gRPC 端口（50040 起）+ UDP 端口（14540 起） |
| **GCS SSH 群发** | 从 GCS_A 设备 `ssh UAV{n} 'rosservice call ...'` | 实机备用 | 依赖局域网连通；批量脚本循环遍历 UAV1..15 |

**建议路径**：仿真用 MAVROS service（已有 bridge 跨 Master 代理）；实机用 MAVSDK-Python（高层 API，无需 SSH，从 GCS_A 所在设备统一控制多机）。可结合 MAVSDK 官方文档（`taking_off_landing.html`）与 PX4 社区多机 offboard 示例。

**本次不实现**：以上仅为调研记录，arm+takeoff+OFFBOARD 自动化脚本/指令列为本计划后续阶段，待状态机修复与坐标系统一验证通过后再实现。
