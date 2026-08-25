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
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash && source devel/setup.bash
catkin build ego_planner_driver swarm_uav_executor
catkin test ego_planner_driver swarm_uav_executor tcp_to_ros
catkin_test_results --all build
```

- `ego_planner_driver`：**118 tests（原 116 + 无邻机不置位断言），0 errors，0 failures**；
- `swarm_uav_executor`：launch 静态测试更新后通过；
- 全量：**232 tests，0 errors，0 failures**。

## 3. SITL 双机正交/换位（§6.2 专项，**未完成——环境阻塞**）

### 3.1 尝试

- 用 `.tmp/scripts/startup_ego_sitl.sh "1 2" active` 一键启动（sim/onboard/gcs/标定/active/takeoff）；
- sim（15 px4 + 15 spawn）PASS；onboard 15/15 ready PASS；GCS bridge→backend PASS（服务代理
  `/UAVn/uav_task` 15 机齐全）；**卡在阶段 4 标定**（`/group_a/calibration_ready=False` 持续 ~30 分钟）。

### 3.2 根因（既有环境链路问题，非本次代码改动）

- GCS_A（11310）侧 `/UAVn/mavros/state`、`/UAVn/mavros/global_position/global` 有 bridge
  publisher，但**无数据流动**（`rostopic echo` 无消息）；
- UAV1（11311）本地 `/mavros/state` 有数据且 publishers 正常，但 **UAV 侧 swarm_bridge 未订阅
  本地 `/mavros/state`**（topic 转发未激活）→ GCS 标定节点 `_try_freeze` 数据不足恒 return，
  无 blocked/ready 日志；
- bridge TCP 链路 ESTAB（4201 等）、service 代理正常（`dealer connected`/`proxying`），
  仅 topic 数据流未建立；疑似 bridge topic 按需转发注册/建链时序问题（既有行为）。

### 3.3 影响与结论

- 标定不 ready → 无法 takeoff → 双机正交/换位 SITL 未执行；
- 与 26082602 改动无关（仅影响 `ego_planner_driver` 内部与 launch 参数，不涉及 bridge/标定链路）；
- 需按既有流程整体干净重启（`cleanup.sh` 后重跑 `startup_ego_sitl.sh`）或人工介入
  bridge topic 转发排障后，再执行 §6.2 双机正交/换位与 intent 停发注入。

## 4. 结果汇总

| 项目 | 结果 | 备注 |
|---|---|---|
| 规划层固定 clearance | ✅ 代码+单测 | `calcSwarmCost` 固定 gate；`computeSafeClearance`/`safe_clearance_m`/`swarm_soft_margin` 删除 |
| 周期碰撞检查 10Hz | ✅ 代码+单测 | `collisionCheckTick` 触发 replan；事件驱动删除 |
| 生成即发 | ✅ 代码+单测 | 周期 5Hz 刷新删除；`traj_id` 提交时递增 |
| 监督层/GPS bias/契约保留 | ✅ | `SafetyPredictor` 动态 EMERGENCY、`bias_valid_s`、intent 时间契约、PositionTarget 契约、旧 1m 门禁未动 |
| 全量自动测试 | **232 tests, 0 errors, 0 failures** | ego 118 + executor + tcp_to_ros |
| 双机正交 SITL | ⬜ 未执行 | 标定阻塞（bridge topic 转发链路，既有问题） |
| 双机换位 SITL | ⬜ 未执行 | 同上 |
| intent 停发注入 | ⬜ 未执行 | 同上 |

## 5. 遗留项（未验证/未解决）

1. **双机同层正交/换位 SITL**（计划 §6.2 场景 1/2）与 **intent 停发注入**（场景 3）：待 bridge
   topic 转发链路恢复后执行；验收判据：实际中心距 ≥1m、速度 ≤1.5m/s、无双方反复重规划振荡、
   replan 次数无 10Hz 高频重复。
2. **bridge topic 转发链路问题**：GCS 侧 bridge 有 publisher 但无数据流，UAV 侧 bridge 未订阅
   本地 `/mavros/state`——既有环境行为，需单独排障（bridge_node.py 按需订阅注册时序）。
3. **`planOnce` 耗时实测**（计划 §2.4 前提）：周期 100ms >> 单次 replan 耗时假设，需在恢复后的
   SITL 运行中实测确认；若 >100ms 需降频或异步化。
