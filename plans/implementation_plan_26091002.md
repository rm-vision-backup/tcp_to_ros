# 实施计划：统一紧急邻机选择与 yield 锁存键

## 1. 目标

修正 yield 避碰流程中的两个优先级一致性问题：

1. `SafetyPredictor` 已确定的 `emergency_peer_id_` 与 `tryYieldReplan()` 实际选择的 `primary_peer` 可能不一致。
2. yield 候选锁存仅比较 `peer_traj_id`，缺少邻机身份，可能把不同邻机的相同轨迹版本误认为同一候选。

修正后，紧急事件触发的让路决策使用同一邻机；yield 锁存以“邻机身份 + 邻机轨迹版本”为判定键。两项修改只修正对象一致性和重复判定，不改变当前 `STAND_ON/YIELD` 规则、候选动作或候选代价。

## 2. 当前实现事实

### 2.1 紧急邻机与 yield 邻机可能不一致

`runSafetySupervisor()` 预测到 `EMERGENCY` 时，在 `emergency_peer_id_` 保存 `prediction.primary_peer_id`，置位 `emergency_pending_` 并唤醒 replan 线程。

`tryYieldReplan(const ReplanSnapshot&)` 当前仍重新遍历 `inbox_`，按邻机轨迹到本机 `start → goal` 线段的最近距离选择 `primary_peer`。因此可能出现：

```text
SafetyPredictor 记录 peer=A
tryYieldReplan 重新选择 peer=B
```

随后 `STAND_ON/YIELD` 角色依据 B 的身份计算，候选动作未必针对触发紧急事件的 A。

### 2.2 yield 锁存键缺少邻机身份

当前字段包括：

```cpp
bool yield_committed_;
YieldDirection yield_direction_;
int32_t yield_peer_traj_id_;
```

`tryYieldReplan()` 在 `yield_committed_` 为真时，仅比较当前 `primary_peer` 的 `traj_id` 与 `yield_peer_traj_id_`。锁存键没有记录 `primary_peer` 身份。

## 3. 修改方案

### 3.1 紧急事件优先复用 `emergency_peer_id_`

**文件：**

- `src/ego_planner_driver/src/ego_planner_driver_node.cpp`
- `src/ego_planner_driver/include/ego_planner_driver/ego_planner_driver_node.h`（仅在需要补充最小测试可见性时修改）
- `src/ego_planner_driver/test/test_ego_state_machine.cpp`

**修改步骤：**

1. 在 `tryYieldReplan()` 选择 `primary_peer` 时，在同一把 mutex 下读取 `emergency_pending_`、`emergency_peer_id_` 和 `inbox_` 中对应的邻机快照；不能先解锁再分别读取，避免使用跨时刻的邻机状态。
2. 当 `emergency_pending_` 为真、`emergency_peer_id_` 非空，且该邻机存在、intent 未过期、轨迹数据有效时，直接使用该邻机作为 `primary_peer`。
3. 直接复用紧急邻机时，仍使用该邻机当前快照中的 `traj_id`、轨迹点和 `peer_hovering` 生成候选；不得继续使用旧快照或只替换 peer 名称。
4. 当紧急邻机不存在、intent 已过期、轨迹无效或身份解析失败时，沿用当前几何最近邻选择逻辑；若回退选择也找不到有效邻机，保持现有 `NOT_AVAILABLE` 结果。
5. 保持现有 `STAND_ON/YIELD` 角色判定和候选生成接口；角色仍基于最终选定的 `primary_peer` 计算。
6. 保持 `emergency_peer_id_` 在普通 replan 成功、yield 提交、HOLD/BRAKE_HOLD 和 `resetRun()` 中的既有清理语义。

### 3.2 将 yield 锁存键扩展为“邻机身份 + 轨迹版本”

**文件：**

- `src/ego_planner_driver/include/ego_planner_driver/ego_planner_driver_node.h`
- `src/ego_planner_driver/src/ego_planner_driver_node.cpp`
- `src/ego_planner_driver/test/test_ego_state_machine.cpp`

**修改步骤：**

1. 在节点状态中增加已锁存邻机身份，例如：

   ```cpp
   std::string yield_peer_id_;
   ```

   空字符串表示当前没有已锁存的邻机。

2. yield 候选重复判定改为同时比较：

   ```text
   yield_committed_
   且 yield_peer_id_ == primary_peer
   且 yield_peer_traj_id_ == primary_traj
   ```

3. 候选提交成功后，同时保存：
   - `yield_peer_id_ = primary_peer`
   - `yield_peer_traj_id_ = primary_traj`
4. 在现有 `resetRun()` 中清空 `yield_peer_id_`，并保持 `yield_peer_traj_id_ = -1`。
5. 冲突解除时同步清理 `yield_committed_`、`yield_peer_id_`、`yield_peer_traj_id_` 和 `yield_direction_`，使锁存字段完整回到“无活动 yield episode”状态；这不改变现有 `yield_committed_` 对重复提交的门控行为。
6. 候选提交失败或未提交时，不更新上述锁存字段。
7. 不改变候选生成顺序、cost 排序、轨迹提交路径和 `YieldCandidateGenerator` 接口。

## 4. 测试计划

### 4.1 紧急邻机一致性

在 `test_ego_state_machine.cpp` 增加或调整用例，验证：

1. 构造两个有效邻机，使几何最近邻与 `emergency_peer_id_` 不同；设置 `emergency_pending_` 和 `emergency_peer_id_=UAV2` 后，`tryYieldReplan()` 使用 UAV2，而不是重新选择几何最近的 UAV3。
2. `emergency_peer_id_` 对应 intent 过期、缺失、轨迹无效或身份不可解析时，回退到现有几何选择；没有有效回退邻机时保持 `NOT_AVAILABLE`。
3. 紧急邻机有效时，传给候选生成的邻机轨迹版本、轨迹数据和 `peer_hovering` 来自同一份当前快照。
4. 紧急邻机有效时，角色判定和候选提交针对该邻机完成。

如需直接断言选择结果，增加最小测试 getter 或测试辅助入口；测试接口只暴露验证所需的邻机身份，不改变生产协议。

### 4.2 yield 锁存键

增加用例验证：

1. 同一 `peer_id + traj_id` 重复请求返回已有锁存语义，不重复提交。
2. `peer_id` 不同但 `traj_id` 相同，可以重新选择并提交候选。
3. 同一邻机轨迹版本变化，可以重新进入候选选择。
4. 冲突解除后，四个 yield episode 字段均清理；随后同一邻机再次出现相同 `traj_id` 时可以重新选择候选。
5. `resetRun()` 后邻机身份和轨迹版本均恢复为空/初始值。
6. 现有 `YieldEpisodeLocksCommit`、高 ID 横向候选、低 ID `slow_only`、低 clearance 触发候选等回归用例继续通过。

## 5. 验收标准

- `emergency_pending_` 期间，只要 `emergency_peer_id_` 对应 intent fresh 且轨迹有效，yield 使用该邻机及其同一份当前快照作为 `primary_peer`。
- 紧急邻机无效时，现有几何选择仍可工作；没有有效回退邻机时保持现有 `NOT_AVAILABLE` 语义。
- yield 重复判定严格使用 `(primary_peer_id, primary_peer_traj_id)`，不同邻机不会因轨迹版本相同而错误跳过。
- 冲突解除或 `resetRun()` 后，yield episode 的身份、轨迹版本、方向和 committed 标志一致清理。
- 现有候选动作、轨迹提交、emergency 清理和冲突 episode 行为无非预期变化。
- `ego_planner_driver` 相关单元测试通过，新增测试覆盖上述分支。
- 修改过程不改变 ROS msg/srv、跨包身份映射和 TCP/backend 行为。

## 6. 修改文件汇总

```text
src/ego_planner_driver/include/ego_planner_driver/ego_planner_driver_node.h
src/ego_planner_driver/src/ego_planner_driver_node.cpp
src/ego_planner_driver/test/test_ego_state_machine.cpp
```
