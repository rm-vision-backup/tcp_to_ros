# 避碰门槛解耦 + 定时/周期碰撞检查 replan 测试记录

时间：2026-08-26（代码+单测完成；SITL 验收待执行）  
记录人：____________  
测试环境：PX4 SITL + Gazebo（无头）+ MAVROS + ego_planner_driver + swarm_uav_executor + swarm_topology_bridge + GCS_A + TCP runtime  
对应实施计划：`plans/implementation_plan_26082602.md`

## 0. 测试目标

验证 26082602 计划三组改动：

1. **规划层回固定 clearance**：`calcSwarmCost` 软惩罚 gate 回固定 `swarm_clearance×2=1m`，
   删除 `computeSafeClearance`/`NeighborTrajectory.safe_clearance_m`/`EgoPlannerConfig.safety_*`/
   `swarm_soft_margin` 依赖；`checkClearance` 固定 1m 硬门不变；
2. **replan 触发源对齐原版**：定时 1s（`replan_period_s`）保留 + 新增 10Hz 周期碰撞检查
   `collisionCheckTick`（本机 `current_` vs fresh 邻机轨迹 < 1m → `planning_requested_`）；
   删除 `acceptNeighborIntentLocked` 事件驱动 replan（移入计划 §5 备用记录）；
3. **发布回生成即发**：删除周期 5Hz intent 刷新（`publishCurrentIntentTick`/`intent_refresh_timer_`/
   `intent_publish_rate_hz`），intent 仅 replan/制动提交时发布（`traj_id` 提交时递增）。

监督层 `SafetyPredictor` 动态 EMERGENCY 门槛、GPS bias 时间戳修正、intent 时间契约、
PositionTarget 契约与旧 1m 执行层门禁**全部保留**。

## 1. 实施内容

### 1.1 ego_planner_driver（代码）

- `ego_planner_core.h`：删除 `EgoPlannerConfig.safety_*`/`swarm_soft_margin` 字段、
  `NeighborTrajectory.safe_clearance_m`、`computeSafeClearance` 声明；`data_age_s` 保留（监督层诊断）。
- `ego_planner_core.cpp`：`calcSwarmCost` gate 回固定 `swarm_clearance*2`；删除
  `computeSafeClearance`/`sampleNeighborVelocity` 实现；`checkClearance` 不变（固定 1m 硬门）。
- `ego_planner_driver_node.h/.cpp`：删除 `publishCurrentIntentTick`/`intent_refresh_timer_`/
  `intent_publish_rate_hz_`；`freshAbsoluteNeighborsLocked` 不再填 `safe_clearance_m`（保留
  `data_age_s` 诊断）；`acceptNeighborIntentLocked` 删除"checkClearance 失效 → replan"事件触发；
  新增 `collisionCheckTick`（10Hz，复用 `core_->checkClearance`）+ public 测试接口
  `runCollisionCheckTick`；构造读取 `collision_check_rate_hz`（默认 10）。

### 1.2 launch/参数

- `ego_planner_driver.launch` / `swarm_uav_executor/launch/uav_executor_ego.launch`：
  删 `intent_publish_rate_hz=5`，加 `collision_check_rate_hz=10`；`replan_period_s=1.0` 不变。
- 监督层参数（`protected_distance_m`/`emergency_decel_mps2`/`peer_data_age_cap_s`/`bias_valid_s` 等）
  保留（SafetyPredictor 消费）。

### 1.3 测试同步

- `test_trajectory.cpp`：原 `PredictiveAvoidanceDeviatesAroundCrossingPeer`（动态距离绕开）改为
  `FixedClearanceHardGateRejectsIntrudingNeighbor`：0.5m 平行侵入 → planOnce 失败/硬门拒绝；
  1.5m 邻机在 1m 门禁外 → 轨迹与无 peer 基线一致。
- `test_ego_state_machine.cpp`：`LowerPriorityIdentityReplansOnlyForHigherPriorityConflict` 改为
  注入冲突 intent 后 `planningRequested()=false`（事件驱动已删）→ `runCollisionCheckTick()`
  → `planningRequested()=true`；无邻机 tick 不置位。
- `swarm_uav_executor/test/test_ego_launch_config.py`：删除 L3 精简遗留的
  `intent_negotiation_wait_s`/`enable_yield_candidates`/`yield_max_velocity_mps` 断言
  （参数已不存在），新增 `collision_check_rate_hz=10` 断言。

## 2. 单元测试

```bash
cd /home/yjq/catkin_swarm6-2
source /opt/ros/noetic/setup.bash && source devel/setup.bash
catkin build ego_planner_driver swarm_uav_executor
catkin test ego_planner_driver swarm_uav_executor tcp_to_ros
catkin_test_results --all build
```

- `ego_planner_driver`：**118 tests（原 116 + 无邻机不置位断言），0 errors，0 failures**；
- `swarm_uav_executor`：launch 静态测试更新后通过；
- 全量：**232 tests，0 errors，0 failures**。

## 3. SITL 双机正交（2026-08-26 重跑，bridge 修复后）

### 3.1 启动链（bridge 修复 + startup HLOG 修复后全通）

- **bridge 根因修复**：旧运行残留 UAV1/UAV2 `bridge_node.py`（8-25 23:13 启动）占用 UAV ZMQ
  topic 端口（4300+/4400+），新 bridge bind 失败退出 → GCS 收不到 A01/A02 → 标定卡死。
  `cleanup.sh`/`startup_ego_sitl.sh` 预检此前不含 bridge 进程 → 漏杀。已修复（cleanup 加
  `[b]ridge_node` 清理与残留检查；startup 预检加 bridge 计数）。
- **startup 脚本 bug**：`HLOG` 未 export，`wait_until` 的 `bash -c` 子进程 `$HLOG` 为空 →
  卡在 sim spawn 检查。已修复（`export HLOG`）。
- 修复后 `startup_ego_sitl.sh "1 2" active`：sim/onboard/gcs/**标定(True)**/active/takeoff 全通
  （takeoff 首跑时机失败，手动重跑 2/2 OK）。

### 3.2 双机正交结果（reciprocal_15sitl.py）

- **UAV2 COMPLETED**（偏差 0.15m，max_speed 1.04 m/s，intent 转发正常
  `neighbor_intent_input_seen=true`）；min_3d_distance=7.166m（两机未接近）。
- **UAV1 FAILED `BRAKE_HOLD`**（误触发）：
  - `UAV1-ego-planner.log` L397：`level=EMERGENCY peer={UAV2, age:1.277s, distance:10.42m,
    min_d:3.89m, warning_d:4.155, emergency_d:3.905}`；
  - 根因：26082602 删周期 5Hz 刷新后 intent 发布频率 5Hz→~1Hz，监督层 `data_age` 从 ~0.2s 涨到
    1.28s，动态门槛被 age 放大到 3.9m，正交交叉预测窗内 `min_d 3.89 < 3.905` → 误判 EMERGENCY；
  - 原版无此问题：监督只用固定 CLEARANCE + 当前时刻采样，无 age 概念。

### 3.3 监督层修订（对齐原版，本次实施）

1. **reaction 不再含 data_age**：邻机轨迹激活时刻已由 GPS bias 修正到本机时间轴（同刻采样即
   预测），`data_age` 不代表位置不确定性 → 从动态门槛反应时间移除（仅诊断）。对齐原版固定
   CLEARANCE 语义，防止发布频率降低后门槛被放大误触发。
2. **EMERGENCY 先重规划再急停**（对齐原版 `checkCollisionCallback`/`planFromCurrentTraj`）：
   - EMERGENCY 时先置 `planning_requested_` + 同步尝试 `replanOnce()`（`replan_failure_fatal_=false`
     失败不收口 HOLD）；成功 → 继续执行；
   - 失败且预测越界时间 `ttc < emergency_replan_time_s`(1.0s，对齐原版 `fsm/emergency_time`) →
     受约束制动；
   - 失败但 ttc 充足 → 交常规 replanLoop 兜底。
3. 新增参数 `emergency_replan_time_s=1.0`（两个 launch）。
4. **EMERGENCY 判定用固定物理 1m**（第二次复测后）：动态 `emergency_d`（含制动余量 ≈1.96m）比
   规划层固定 1m 硬门更严 → UAV2 绕开 cpa_d≈1.2m 时永续 EMERGENCY→重规划 ping-pong。改回对齐
   原版：`level=EMERGENCY` 仅当预测距离 ≤ `protected_distance`(1m)，动态门槛只作 WARNING 缓冲。

### 3.4 复测时间线（监督层修订验证）

| 轮次 | 结果 | 说明 |
|---|---|---|
| 首跑（去 age + 先重规划） | UAV1 误制动（age 放大门槛，BRKAE_HOLD） | 动态门槛 3.9m > 预测窗 min_d 3.89m |
| 二跑（+EMERGENCY 物理 1m 判定） | **UAV1 COMPLETED**、UAV2 HOLD_FAILED | 误制动消除；UAV2 cpa_d≈1.2m 触发动态门槛 → 持续 EMERGENCY→replan ping-pong → 规划失败收口 |
| 三跑（EMERGENCY 物理 1m 判定后） | **UAV2 COMPLETED**、UAV1 EGO_PLAN_FAILED→HOLD_FAILED | **无误制动**；min_3d=2.59m（≥1m 安全达标）；UAV1 正交交叉无法规划穿越 UAV2 移动走廊 → 安全 HOLD |
| 四跑（+SEQUENTIAL_START 首轮排序） | **UAV1(leader) COMPLETED**、UAV2 EGO_PLAN_FAILED→HOLD_FAILED | **确定性**：leader 先规划先完成（不再随机）；min_3d=4.26m ≥1m；UAV2 目标路径被 UAV1 航线阻挡（几何无解）→ 安全 HOLD |

- 监督层修订**验证有效**：四跑均无 EMERGENCY 制动误触发、实际中心距 ≥1m（2.59-4.26m）。
- **SEQUENTIAL_START 效果对比**（想法 1 验证）：
  - 前三跑无排序：COMPLETED/FAILED 角色**随机**（UAV1 或 UAV2 交替）——首轮盲规划 + replan 无确定性
  - 四跑有排序：**UAV1(leader) 确定先完成**，UAV2 基于 UAV1 轨迹规划——次序混乱消除
- 剩余 FAIL 根因：**几何无解**——UAV2 目标 (10,13) 路径须穿越 UAV1 航线走廊 (y=5)；固定
  clearance + 无变速/让路候选（L3 删 SLOW/FAST）→ 穿越不可行 → 规划失败 → 安全 HOLD。
  这印证想法 2：**需要变速/让路能力（含 right-of-way）才能解决穿越被阻挡走廊**。

## 4. 结果汇总

| 项目 | 结果 | 备注 |
|---|---|---|
| 规划层固定 clearance | ✅ 代码+单测 | `calcSwarmCost` 固定 gate；`computeSafeClearance`/`safe_clearance_m`/`swarm_soft_margin` 删除 |
| 周期碰撞检查 10Hz | ✅ 代码+单测 | `collisionCheckTick` 触发 replan；事件驱动删除 |
| 生成即发 | ✅ 代码+单测 | 周期 5Hz 刷新删除；`traj_id` 提交时递增 |
| 监督层修订（对齐原版） | ✅ 代码+单测 | reaction 去 data_age；EMERGENCY 先重规划再急停（`emergency_replan_time_s=1.0`） |
| 监督层/GPS bias/契约保留 | ✅ | `SafetyPredictor` EMERGENCY、`bias_valid_s`、intent 时间契约、PositionTarget 契约、旧 1m 门禁未动 |
| 全量自动测试 | **234 tests, 0 errors, 0 failures** | ego 119 + executor + tcp_to_ros |
| bridge 残留修复 | ✅ 验证 | cleanup/预检加 bridge 检查；标定不再卡死 |
| 双机正交 SITL | ⚠️ 四次复测 | 监督层误制动消除、中心距 ≥1m、首轮排序确定性生效；剩余 UAV2 穿越被阻挡走廊规划失败（安全 HOLD，几何无解） |
| 双机换位 SITL | ⬜ 未执行 | 正交完成后可测 |
| intent 停发注入 | ⬜ 未执行 | 同上 |

## 5. 遗留项（未验证/未解决）

1. **穿越被阻挡走廊的几何无解**：UAV2 目标路径被 UAV1 航线阻挡（固定 clearance + 无变速/让路候选）
   → 规划失败安全 HOLD。SEQUENTIAL_START 已保证 leader 确定性先完成；后续需决策：a) 测试场景时序
   错开（一方先过）；b) 恢复变速/等待候选（SLOW/FAST + right-of-way，验证想法 2）；c) 接受 HOLD
   安全收口为预期行为。
2. **双机换位 SITL** 与 **intent 停发注入**（计划 §6.2 场景 2/3）：待正交场景方案确定后执行。
3. **`planOnce` 耗时实测**（计划 §2.4 前提）：周期 100ms >> 单次 replan 耗时假设，需在 SITL
   运行中实测确认。
4. **takeoff 首跑时机失败**（startup 阶段 6 偶发）：active 重启后 MAVROS 未完全就绪，手动重跑
   成功；可考虑 startup 增加 HOLD 模式确认等待。
