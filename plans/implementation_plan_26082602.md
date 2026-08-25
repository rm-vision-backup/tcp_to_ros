# 避碰门槛解耦 + 定时/周期碰撞检查 replan 机制实施计划

时间：2026-08-26 02:57
状态：**待实施**

## 1. 目标与成功判据

在既有 min-snap 单段滚动规划、PositionTarget 控制链路、GPS bias 时间戳修正和 intent 采样广播之上：

- **解耦避碰门槛**：规划层 `calcSwarmCost` 由"动态安全距离（含 data_age 反应项）"回退为
  **固定 clearance**（`swarm_clearance × 2 = 1m`），删除规划层对 `data_age` / `computeSafeClearance` /
  `swarm_soft_margin` 的依赖；
- **replan 机制对齐原版**：由"周期 replan + 邻机 intent 失效触发"改为
  **定时 replan（1s）+ 周期碰撞检查（10Hz）** 两个触发源；删除"接收邻机轨迹即检查碰撞 replan"的
  事件驱动触发（作用与周期检查重复，移入 §5 备用记录）；
- **发布回"生成即发"**：删除周期 5Hz intent 刷新（其唯一目的——保持 `data_age` 小——随动态门槛
  解耦而消失），replan 提交时发布带时间戳的 intent 轨迹。

成功判据：

- 规划层避碰门槛为固定 1m，不再随接近速度/意图年龄变化；监督层 SafetyPredictor **保留**动态
  EMERGENCY 门槛作为制动兜底；
- replan 触发 = 定时 1s（`replan_period_s`）+ 周期碰撞检查（`collision_check_rate_hz=10`）；
  两触发源均不产生高频重复 replan（周期 100ms >> 单次 replan 耗时）；
- 双机正交/换位 SITL：实际中心距 ≥1m、速度 ≤1.5m/s、无双方反复重规划振荡；
- 全量自动测试 0 failure；既有 GPS bias 修正、intent 时间契约、PositionTarget 契约不回归。

## 2. 背景与决策依据

### 2.1 为什么可以解耦（固定 clearance）

- `calcSwarmCost` 的**同刻采样**（`t_abs = glb_time - neighbor.start_time`，经 GPS bias 修正到本机
  时间轴）本身就是"预测邻机未来位置"——采样点即 peer 未来时刻位置；
- 因此**预测能力来自"轨迹 + 时间戳对齐"**，不来自 `data_age` 动态门槛；
- 原版即"固定 clearance + 时间同步采样"（`glb_time = t_now + 控制点时刻`），证明该组合足以互惠避碰；
- `data_age` 动态门槛是当前添加的"对意图过时的保守容错"，非预测能力来源；
- 时间戳修正（GPS bias）已保证同刻采样位置可靠 → 固定 clearance 即可。

### 2.2 为什么删周期发布、回"生成即发"

- 周期 5Hz 刷新（固定 epoch + traj_id 递增）的唯一目的是保持邻机 `data_age` 小 → 动态门槛小；
- 动态门槛解耦后该目的消失 → 周期刷新可删除，回原版"replan 提交即发布"；
- 实机自组网电台**有缓存可任意时刻发布**（不必对齐 publish 时隙），`replan 后立即发布`可直接用；
- 多次 replan 的轨迹会在 publish 时隙集中发布，**必须带时间戳**（`stamp` + `traj_id` 已具备，接收方
  用 `(epoch, traj_id)` 去重/对齐）——保留现有时间戳机制。

### 2.3 为什么删除"接收邻机轨迹即检查碰撞 replan"（事件驱动）

- 该触发（`acceptNeighborIntentLocked` 中 `checkClearance` 失效 → `planning_requested_`）与
  **周期碰撞检查**作用重复：两者都做"本机轨迹 vs 邻机轨迹距离 < clearance → replan"；
- 周期检查用最新缓冲的邻机轨迹，邻机轨迹更新后下一个周期（≤100ms）即检查到同一碰撞；
- 事件驱动只省最多 100ms 检测延迟，在 1m/s 下 = 10cm 位移，可忽略；
- 移入 §5 备用记录（未来高速/即时响应需求可启用）。

### 2.4 周期碰撞检查频率：10Hz（已确认）

- 周期 100ms >> 单次 replan 完成时间（min-snap 单段 + L-BFGS 估算 5-20ms），无并发堆积；
- 1m/s 低速下 100ms 检测延迟 = 10cm，相对固定 1m clearance 余量充足；
- 10Hz 比原版 20Hz CPU 减半（每次检查全部邻机轨迹的前 2/3 采样）；
- 前提：实施后实测 `planOnce` 耗时，若 >100ms 再降频或异步化 replan。

## 3. 方案

### 3.1 规划层回固定 clearance

`ego_planner_core`：

- `calcSwarmCost` 软惩罚 gate 回 `config_.swarm_clearance * 2.0`（固定），删除
  `neighbor.safe_clearance_m + config_.swarm_soft_margin` 分支；
- 删除 `computeSafeClearance`（规划层动态距离函数）及其调用（`freshAbsoluteNeighborsLocked` 不再
  填充 `safe_clearance_m`）；
- `EgoPlannerConfig` 删除 `safety_*`（`safety_control_delay_s` 等）与 `swarm_soft_margin`、
  `safety_peer_data_age_cap_s` 的**规划层使用**（`NeighborTrajectory.safe_clearance_m` 保留字段但
  不再由规划层消费）；
- `checkClearance` 保持固定 `swarm_clearance × 2`（不变，提交前硬校验）。

### 3.2 监督层保留动态 EMERGENCY 门槛（兜底）

- `SafetyPredictor` 的 `emergency_distance`（`dynamicMargin` + `data_age` cap）**保留**；
- `runSafetySupervisor` 仍仅在 `level == EMERGENCY` 触发制动（`BRAKE_HOLD`）；
- `data_age_s` 字段保留（监督层填充/消费），周期刷新删除后 age 由 replan 频率决定（0-1s），
  `peer_data_age_cap_s`（2s）限制其无界增长，监督层更保守而非失效。

### 3.3 replan 触发源

| 触发源 | 频率 | 实现 |
|---|---|---|
| 定时 replan | 1s（`replan_period_s`） | 现有 `replanLoop` 周期（保留） |
| **周期碰撞检查**（新） | **10Hz**（`collision_check_rate_hz`） | 新增 `collisionCheckTick`：本机 `current_`（前 2/3）与全部 fresh 邻机轨迹同刻距离 < 固定 clearance → `planning_requested_ = true`；复用 `core_->checkClearance`（固定 1m）或对前 2/3 采样 |
| ~~接收邻机轨迹即检查碰撞~~ | — | **删除**（§2.3）；移入 §5 备用记录 |

`replanLoop` 每 1s 检查 `planning_requested_`；10Hz tick 置位 → 最迟 1s 内执行 replan（与定时合并）。

### 3.4 发布回"生成即发"

- 删除 `publishCurrentIntentTick`（周期 5Hz）与其 Timer/`intent_pub_mutex_` 依赖（`traj_id` 递增
  由 `publishIntent` 保持，仅 replan 提交时递增）；
- `publishIntent` 在 replan 提交/制动提交时调用（现状已具备），intent 带 `stamp`（激活 epoch）+
  `traj_id`，接收方按 GPS bias 修正后对齐时间轴。

## 4. 文件级改动预期

- `ego_planner_driver/include/ego_planner_driver/ego_planner_core.h`：
  - 删除 `computeSafeClearance` 声明、`EgoPlannerConfig.safety_*`（或标注规划层不再消费）、
    `swarm_soft_margin`、`safety_peer_data_age_cap_s`；
  - `NeighborTrajectory.safe_clearance_m` 保留（兼容字段，标注仅监督层诊断用或一并删除）。
- `ego_planner_driver/src/ego_planner_core.cpp`：
  - `calcSwarmCost` 回固定 gate；删除 `computeSafeClearance`、`sampleNeighborVelocity` 若不再使用；
  - `checkClearance` 不变。
- `ego_planner_driver/include/ego_planner_driver/ego_planner_driver_node.h`：
  - 删除 `publishCurrentIntentTick`、`intent_refresh_timer_`、`intent_publish_rate_hz_`；
  - 新增 `collisionCheckTick`（10Hz）声明 + `collision_check_rate_hz_`。
- `ego_planner_driver/src/ego_planner_driver_node.cpp`：
  - 删除周期刷新 Timer/回调；`freshAbsoluteNeighborsLocked` 不再填 `safe_clearance_m`；
  - `acceptNeighborIntentLocked` 删除"checkClearance 失效 → planning_requested_"事件触发
    （移入 §5 备用记录）；
  - 新增 `collisionCheckTick`（10Hz）：`current_` 有效时对 fresh 邻居做固定 clearance 检查，
    碰撞 → `planning_requested_ = true`；
  - 构造读取 `collision_check_rate_hz`（默认 10）；`bias_valid_s`/GPS bias 修正保留。
- `ego_planner_driver/launch/ego_planner_driver.launch` / `swarm_uav_executor/launch/uav_executor_ego.launch`：
  - 删除 `intent_publish_rate_hz`；新增 `collision_check_rate_hz=10`；`replan_period_s=1.0` 不变。
- `ego_planner_driver/test/test_trajectory.cpp` / `test_ego_state_machine.cpp`：
  - `calcSwarmCost` 固定 clearance 用例；`collisionCheckTick` 触发 replan 用例；
  - 删除周期刷新相关测试；事件触发删除后回归。

## 5. 备用记录（不实施，仅记录）：事件驱动 replan 触发源

> **本方案不实施**以下事件驱动触发，仅记录设计，供未来高速/即时响应需求参考启用。

### 5.1 触发源：接收邻机轨迹即检查碰撞 replan

```
[当前版本（拟删除）]
  acceptNeighborIntentLocked 内：
    if (current_ && 邻机新轨迹使 current_ 在固定 1m clearance 下失效)
      planning_requested_ = true;   // 事件驱动 replan

[原版对应]
  BroadcastBsplineCallback：收到邻机 B-spline → 0.25s 时间窗 → 填缓冲
    → checkCollision(id)（本机轨迹 vs 该邻机轨迹，0.03s 步长，< swarm_clearance）
    → 碰撞 → changeFSMExecState(REPLAN_TRAJ)
```

### 5.2 为何不实施（当前决策）

- **作用重复**：与周期碰撞检查（10Hz）做同一件事（本机轨迹 vs 邻机轨迹距离 < clearance → replan）；
  周期检查用最新缓冲，邻机轨迹更新后 ≤100ms 即检查到同一碰撞；
- **即时性收益可忽略**：事件驱动只省 ≤100ms 检测延迟，1m/s 下 = 10cm 位移；
- **复杂度**：事件驱动在 `acceptNeighborIntentLocked` 内耦合"意图接收 → 碰撞 → replan"，分散 replan
  触发逻辑；周期检查集中在一处（`collisionCheckTick`），更清晰。

### 5.3 启用条件（未来）

- 速度显著提升（>2m/s）且 100ms 检测延迟不可接受；
- 需要"邻机轨迹更新即刻响应"的时序（如密集编队、紧耦合场景）；
- 启用时：恢复 `acceptNeighborIntentLocked` 的事件触发（仅对**新版本** intent 检查碰撞，配合
  `(epoch, traj_id)` 去重避免每帧触发），与 10Hz 周期检查并存（事件 = 即时加速，周期 = 兜底）；
- 启用后需重跑 §6 测试与 SITL 验收。

## 6. 测试计划

### 6.1 单元测试（ego_planner_driver）

- `calcSwarmCost`：邻居距离在固定 clearance 内/外时软惩罚正确；无 `safe_clearance_m` 时回固定 gate；
- `checkClearance` 固定 1m 不回归；
- `collisionCheckTick`：注入邻机轨迹使 `current_` 前 2/3 段 < 1m → `planning_requested_` 置位；
  无碰撞/邻机太远 → 不置位；
- 删除周期刷新后：`publishIntent` 仅在 replan 提交时递增 `traj_id`；无 5Hz 定时发布；
- 删除事件触发后：`acceptNeighborIntentLocked` 不再直接置 `planning_requested_`（回归验证
  `NeighborIntentValidationAndSequenceReset` 等既有用例）；
- GPS bias 修正（`GpsBiasCorrectsNeighborIntentEpoch`）与固定 clearance 并存时 epoch 修正不回归。

### 6.2 PX4 SITL 专项

1. 双机同层正交：实际中心距 ≥1m，无双方反复重规划振荡；
2. 双机同层换位：持续冲突下周期碰撞检查驱动 replan 收敛；
3. 注入邻机 intent 停发（模拟失联）：周期检查依赖缓冲轨迹仍可检测碰撞；
4. 每轮录制 rosbag / exec_state / intent / ULog，统计 replan 次数（验证无 10Hz 高频重复）。

### 6.3 回归

- 全量 `catkin_test_results --all build` 0 failure；
- `replan_period_s=1.0`、PositionTarget 契约、relay 唯一出口、GPS bias、intent 时间契约不回归。

## 7. 实施顺序与提交边界

1. 规划层：`calcSwarmCost` 回固定 clearance，删 `computeSafeClearance`/`safe_clearance_m` 消费；
   C++ 单测；
2. 发布层：删除周期 5Hz 刷新，回"生成即发"；单测；
3. replan 触发：新增 `collisionCheckTick`（10Hz），删除 `acceptNeighborIntentLocked` 事件触发；
   单测；
4. launch/参数：删 `intent_publish_rate_hz`，加 `collision_check_rate_hz=10`；
5. 全量回归 + 双机 SITL 正交/换位；
6. 文档：本计划状态更新、`plans/README.md` 登记、测试记录归档（`test_records/test_record_2608xxxx.md`）；
7. 按仓库提交（`ego_planner_driver`、`swarm_uav_executor`；Conventional Commit，不 push）。

## 8. 回滚方案

- 实施前记录 `ego_planner_driver`/`swarm_uav_executor` 基线 commit；
- 回滚需同时恢复：`calcSwarmCost` 动态 gate + `computeSafeClearance`、周期 5Hz 刷新、事件触发 replan、
  launch 参数与测试，禁止只回滚部分；
- 若固定 clearance 导致 SITL 实测中心距 <1m 或监督层误制动增加，先恢复动态软惩罚（`swarm_soft_margin`）
  作为中间态，再评估；紧急制动与旧 1m 门禁始终保留兜底；
- 不以启用 rebound、增大 clearance 或改 PX4 增益临时掩盖失败。

## 9. 完成定义

- `calcSwarmCost` 固定 clearance，规划层无 `safe_clearance_m`/`swarm_soft_margin` 依赖；
- replan 触发 = 定时 1s + 周期碰撞检查 10Hz，无事件驱动 replan（仅 §5 备用记录）；
- 周期发布删除，intent 生成即发（带 `stamp`+`traj_id` 时间戳）；
- 监督层 EMERGENCY 动态门槛与 GPS bias 修正保留；
- 双机正交/换位 SITL 中心距 ≥1m、无振荡；全量自动测试 0 failure。

