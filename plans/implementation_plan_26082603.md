# plan-exec 循环重构 + 单一 replan 入口 + EMERGENCY 走 replan（方案 Y）实施计划

时间：2026-08-26
状态：**待实施**

## 1. 背景与决策依据

### 1.1 SITL 七轮复测复盘（26082602 后）

| 轮 | 结果 | 暴露问题 |
|---|---|---|
| 1-4 | UAV 误制动 / ping-pong / 规划失败 HOLD | 监督层门槛、SEQUENTIAL_START 缺失 |
| 5 | UAV2 首轮规划失败（yield 未触发） | 候选只在 `collisionCheckTick`（基于 current_），检测不到首轮规划失败 |
| 6 | UAV2 超速 2.06、min_3d 0.68 | 候选偏移方向/长度错误 + 速度约束缺失 |
| 7 | UAV2 让路成功（min_3d 3.5）但未完成穿越 | **INSUFFICIENT_DATA 564 次**（候选让路破坏监督预测窗）+ **多路径提交竞争** |

### 1.2 核心结论（用户审查确认）

1. **多路径竞争**：`replanLoop`（1s 线程）/ `collisionCheckTick`（10Hz）/ `runSafetySupervisor`（25Hz）
   三处都改 `current_`/`plan_start_`/发布 intent——mutex 只防数据竞争，不防**逻辑覆盖**（一处提交候选，
   另一处立刻 replan 覆盖 → 轨迹不稳定）。
2. **候选破坏监督预测**：UAV2 SLOW 让路（时间延长）→ 与 UAV1 共同预测窗不足 → `INSUFFICIENT_DATA` →
   EMERGENCY 保护失效，纯靠候选轨迹裸奔。
3. **复杂度失控**：规划层 + 10Hz 检测 + 候选层 + right-of-way + episode + 监督层 + SEQUENTIAL_START +
   GPS bias 多层叠加，每轮修一个暴露下一个。

### 1.3 决策方向（用户确认）

- **plan-exec 循环对齐原版**（分支 A 航点推进 / 分支 B 到达 hold / 分支 C 定时 replan）：3m 密化 keypoints，
  目标 = 相邻下一个 keypoints，替换"horizon 窗口滚动"；
- **单一 replan 入口**（方案 A）：所有触发只置位/唤醒，replanLoop 唯一提交者；
- **EMERGENCY 制动走 replan**（方案 Y）：监督层只检测置位，制动由 replan 线程统一提交；
- **候选保留**：Yield 非对称优先级 + SLOW2/LEFT3/RIGHT3，作为 replan 失败分支；
- **状态机简化**：删除 POSE_STALE（HOLD 锁存位置）；
- **启动流程**：保持 shadow 默认 + 个别机 active 重启（不取消）。

## 2. 目标与成功判据

- plan-exec 循环按"3m 密化 keypoints + 相邻目标推进"滚动，无 horizon 窗口逻辑；
- replan 触发 = 1s 定时 + 10Hz 冲突事件 + checkArrival 推进事件 + EMERGENCY 事件，**全部只置位**，
  replanLoop 为唯一轨迹提交者；
- EMERGENCY 制动（BRAKE_HOLD）由 replan 线程在"常规 replan 失败且 ttc 紧急"时提交，不再由监督回调直接改轨迹；
- 候选（SLOW/LEFT/RIGHT + right-of-way）仅在 replan 规划失败分支生成；
- 双机正交/悬停 SITL：中心距 ≥1m、速度 ≤1.5、双方 COMPLETED；
- 全量自动测试 0 failure；PositionTarget 契约、relay 唯一出口、GPS bias、固定 clearance 不回归。

## 3. plan-exec 循环重构（对齐原版 A/B/C）

### 3.1 keypoints 与任务航点

- 上层 `EgoSwarmDriver`：分层关键点按 **3m 间距** 均匀密化（`waypoint_densify_spacing=3.0`）→ `/waypoints`；
- keypoints 序列 = 全局规划参考序列（含 start、层点、弓字转折、目标）；
- **任务航点 = 相邻下一个 keypoints(n+1)**（替换当前 horizon 窗口末点）。

### 3.2 执行循环（对照原版分支）

```
[EXECUTING 每 tick]
  ① 推进（原版分支 A）：
     ||pos - keypoints(n)|| < 0.5m（到当前目标 (n+1) 距离 <0.5m）→ 切换到 keypoints(n+2) 为目标
     （最远使用 3.5m 远的 keypoint；目标距当前位置 <0.5m 时丢弃推进）
  ② 到达（原版分支 B）：
     ||pos - 最终 keypoints|| < 0.1m → COMPLETED → hold（终点坐标为 setpoint）
  ③ 定时 replan（原版分支 C）：
     elapsed >= replan_period_s(1.0) → 置 planning_requested_（唤醒 replanLoop）
```

### 3.3 planOnce/replanOnce（统一）

```
① 起点 = 旧轨迹同刻 P/V/A（滚动连续）；首次 = 实际位置 + 零导数
② 终点 = 当前任务航点 keypoints(n+1)
③ end_vel = 当前位置 → (n+1) 方向 × min(cruise_velocity, max_vel)；最终段 = 0
④ 生成 = min-snap 单段（0.4m 弧长采样）→ B-spline → L-BFGS（固定 1m swarm 软惩罚 + feasibility + terminal）
⑤ 校验 = checkClearance 固定 1m 硬门 + 提交发布（生成即发）
```

## 4. 单一 replan 入口（方案 A）

### 4.1 触发 → 只置位/唤醒

```
① 1s 定时 replan（原版分支 C）      → planning_requested_ = true
② collisionCheckTick(10Hz) 冲突    → planning_requested_ = true（不再直接提交候选）
③ checkArrival 推进/到达           → 推进 n 或 COMPLETED；推进时置 planning_requested_
④ runSafetySupervisor(25Hz) EMERGENCY → emergency_pending_ 置位 + 唤醒 replanLoop（不直接提交/制动）
```

### 4.2 replanLoop（唯一提交者，事件唤醒）

- 常态 1s 定时；`planning_requested_`/`emergency_pending_` 置位时**事件唤醒**立即执行（条件变量）；
- `replanOnce()` 内部按优先级处理：
  1. `emergency_pending_` 且常规 replan 失败且 primary ttc < `emergency_replan_time_s` → **制动**；
  2. 常规 replan（绕开）成功 → 提交轨迹，清除 `emergency_pending_`；
  3. 常规 replan 失败 → 候选（§6）→ 候选成功提交；全失败 → `PLAN_FAILED`；
- **所有轨迹变更（常规 replan / 候选 / 制动）都在 `replanOnce()` 内提交**——提交唯一化。

### 4.3 collisionCheckTick 简化

- 仅"检测 `current_` vs fresh 邻机同刻最小距离 < 1.3×clearance → 置 `planning_requested_`"；
- **删除直接候选生成/提交**（`commitYieldCandidate` 从 10Hz 路径移除，只保留 replan 内部调用）。

## 5. EMERGENCY 制动走 replan（方案 Y）

### 5.1 机制

```
runSafetySupervisor(25Hz)：
  预测 EMERGENCY（min_d ≤ 1m）→ 置 emergency_pending_（记录 primary peer + ttc）→ 唤醒 replanLoop
  （不直接 replan / 制动 / 改 current_）

replanLoop → replanOnce()：
  ① 常规规划（绕开）成功 → 提交，清除 emergency_pending_
  ② 失败：
     if emergency_pending_ && primary ttc < emergency_replan_time_s(1.0s)：
       → 生成制动轨迹（EmergencyBrakeGenerator）→ 提交（EMERGENCY_BRAKE → BRAKE_HOLD）
     else：
       → PLAN_FAILED（收口 HOLD）
```

### 5.2 要点

- **提交唯一化**：制动也走 replan 线程（replan 5-20ms，事件唤醒后延迟可接受）；
- **EMERGENCY 时跳过候选**：ttc 紧时直接判断制动，不做 SLOW/LEFT/RIGHT 浪费时间；
- `replan_failure_fatal_` 标志保留（EMERGENCY 尝试路径失败不收口，由 Y 方案判定制动/PLAN_FAILED）；
- `BRAKE_HOLD` 锁存制动终点，executor 收口 `COMMAND_HELD`（不自动恢复）。

## 6. 候选保留（replan 失败分支）

- **right-of-way**：`parseNumericId` 低 ID STAND_ON（保持 nominal，交常规 replan）、高 ID YIELD（生成候选）；
  episode 内锁存角色与方向；
- **候选档位**（用户确认）：
  - SLOW：scale {0.5, 0.3}（时间缩放，时空错开；悬停前机跳过）；
  - LEFT/RIGHT：lateral {1.0, 1.5, 2.0} × scale {0.7, 0.5}（横向 plateau 偏移 + 可选减速），
    方向 = 远离 peer 时空重叠时段法向分量，plateau 覆盖重叠全程 + C2 过渡；
- **验证**：min_clear ≥ 1.3×clearance（时空不重叠），时间单调/finite，速度 ≤1.5 由时间轴生成保证；
- **cost** = ΔT/T_nominal + dev/1.5（SLOW dev=0 自然偏向）；
- **调用点**：仅 `replanOnce()` 常规规划失败分支（10Hz 独立提交删除）。

## 7. 状态机与 HOLD 简化

- **删除 POSE_STALE 路径**：HOLD 不再读取 pose 更新 setpoint——HOLD setpoint = **进入 HOLD 时锁存的
  位置/yaw**（plan/replan 进入时读一次当前 pose），后续不随 pose 漂移；
- **保留** EMERGENCY_BRAKE / BRAKE_HOLD（方案 Y 由 replan 线程提交）；
- 状态集：HOLD / TAKEOFF / EXECUTING / COMPLETED / EGO_PLAN_FAILED / EGO_EXEC_TIMEOUT /
  EMERGENCY_BRAKE / BRAKE_HOLD（8 个，删 POSE_STALE）；
- executor `TERMINAL_BAD` 映射同步（保留 BRAKE_HOLD）。

## 8. 文件级改动预期

- `ego_planner_driver/src/ego_planner_driver_node.cpp`：
  - `replanLoop`：事件唤醒（条件变量）+ 唯一提交；
  - `collisionCheckTick`：删候选提交，只检测置位；
  - `runSafetySupervisor`：EMERGENCY 只置 `emergency_pending_` + 唤醒；
  - `replanOnce`：优先 EMERGENCY 制动分支 → 常规规划 → 候选；`checkArrival` 推进条件
    （距目标 <0.5m 切下一 keypoint，到达 <0.1m COMPLETED）；
  - 删 POSE_STALE 路径（HOLD 锁存位置）；
- `ego_planner_driver/include/.../ego_planner_driver_node.h`：删 POSE_STALE 相关；加
  `emergency_pending_`/`emergency_peer_ttc_`；改 `planning_requested_` 语义（事件）；
- `ego_planner_driver/src/ego_planner_core.cpp`：planOnce 终点/end_vel 逻辑（keypoints(n+1)、
  当前位置→(n+1) 方向）；
- `swarm_uav_executor/src/.../ego_swarm.py`：`waypoint_densify_spacing=3.0`；
- `yield_candidates`：档位参数（SLOW {0.5,0.3}、LEFT/RIGHT {1.0,1.5,2.0}×{0.7,0.5}）；
- `emergency_brake`：保留（由 replan 调用）；
- 测试：`test_ego_state_machine.cpp`（单一入口/推进/EMERGENCY Y 方案）、`test_yield_candidates.cpp`
  （档位）、`test_ego_swarm_driver.py`（3m 密化）。

## 9. 测试计划

### 9.1 单元测试

- 推进：距目标 <0.5m 切 (n+2)、<0.1m COMPLETED、0.5m 内丢弃；
- 单一入口：collisionCheckTick/EMERGENCY/推进置位 → replanLoop 事件唤醒；
- EMERGENCY Y：EMERGENCY → 置位唤醒 → replan 失败 + ttc 紧 → 制动提交；ttc 充足 → 保持等待；
- 候选：SLOW/LEFT/RIGHT 档位、right-of-way、episode、min_clear ≥1.3；
- 状态机：删 POSE_STALE 后 HOLD 锁存位置不漂移；
- 回归：PositionTarget/relay/固定 clearance/GPS bias。

### 9.2 SITL

1. 双机正交：双方 COMPLETED、min_3d ≥1m、速度 ≤1.5、无多路径竞争振荡；
2. 悬停前机通过：LEFT/RIGHT 绕行；
3. EMERGENCY 注入：replan 失败 + ttc 紧 → BRAKE_HOLD 收口；
4. 每轮录 rosbag / exec_state / intent / ULog。

## 10. 实施顺序与提交边界

1. `replanLoop` 事件唤醒 + `collisionCheckTick` 删候选提交（单一入口）+ 单测；
2. `runSafetySupervisor` 只置 `emergency_pending_` + `replanOnce` 制动分支（方案 Y）+ 单测；
3. plan-exec 循环：3m 密化 + (n+1) 目标 + 推进/到达条件 + 单测；
4. 候选档位调整 + 状态机删 POSE_STALE + 单测；
5. 全量回归 + SITL 正交/悬停/EMERGENCY；
6. 文档：`plans/README.md` 状态更新、测试记录归档；
7. 按仓库提交（`ego_planner_driver`、`swarm_uav_executor`；Conventional Commit，不 push）。

## 11. 回滚方案

- 基线：本计划实施前 `ego_planner_driver`/`swarm_uav_executor` 基线 commit；
- 回滚需同时恢复：collisionCheckTick 独立候选、监督回调直接制动、horizon 窗口滚动、POSE_STALE 路径，
  禁止只回滚部分；
- 若 EMERGENCY 走 replan 后制动延迟（replan 阻塞）导致中心距 <1m，回退监督直接制动（方案 X）并评估
  replan 耗时；executor 距离门禁始终保留兜底；
- 不以启用 rebound、增大 clearance 或改 PX4 增益临时掩盖失败。

## 12. 完成定义

- replan 触发 = 1s 定时 + 10Hz 冲突 + 推进 + EMERGENCY 事件，**全部只置位**，replanLoop 唯一提交；
- EMERGENCY 制动由 replan 线程在"replan 失败 + ttc 紧"时提交（方案 Y），监督回调不改轨迹；
- plan-exec 循环按 3m keypoints + (n+1) 目标 + 0.5m/0.1m 推进/到达条件；
- HOLD 锁存位置（删 POSE_STALE）；候选 SLOW2/LEFT3/RIGHT3 + right-of-way 保留；
- 双机正交/悬停 SITL 双方 COMPLETED、min_3d ≥1m、速度 ≤1.5；
- 全量自动测试 0 failure；PositionTarget 契约、relay 唯一出口、GPS bias、固定 clearance 不回归。

