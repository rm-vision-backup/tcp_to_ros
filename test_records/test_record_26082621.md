# plan-exec 循环重构 + 单一 replan 入口 + EMERGENCY 走 replan（方案 Y）测试记录

时间：2026-08-26 22:xx-23:xx CST
记录人：____________
测试环境：PX4 SITL + Gazebo（无头）+ MAVROS + ego_planner_driver + swarm_uav_executor + swarm_topology_bridge + GCS_A + TCP runtime
对应实施计划：`plans/implementation_plan_26082621.md`
基线：`ego_planner_driver 3237567`、`swarm_uav_executor 17de497`

## 0. 测试目标

1. **plan-exec 循环对齐原版 A/B/C**：3m 密化 keypoints，任务航点 = 相邻下一个 keypoints(n+1)，
   替换 horizon 窗口滚动；
2. **单一 replan 入口（方案 A）**：1s 定时 / 10Hz 冲突 / 推进 / EMERGENCY 事件全部只置位唤醒
   replanLoop（条件变量），replanLoop 唯一提交者；
3. **EMERGENCY 制动走 replan（方案 Y）**：监督层只置 `emergency_pending_`，制动由 replan 线程在
   "常规 replan 失败且 ttc 紧急"时提交（EMERGENCY_BRAKE → BRAKE_HOLD）；
4. 候选（SLOW/LEFT/RIGHT + right-of-way）只在 replan 失败分支生成，档位
   SLOW {0.5,0.3}、LEFT/RIGHT {1.0,1.5,2.0}×{0.7,0.5}；
5. 删 POSE_STALE（HOLD 锁存位置），executor `_TERMINAL_BAD` 同步。

## 1. 实施内容

### 1.1 ego_planner_driver

- `replanLoop`：改为条件变量事件唤醒（`replan_cv_`，`replan_period_s` 1s 超时兜底），删除线程内
  重复的规划/提交逻辑，统一调用 `replanOnce()`；
- `replanOnce`：门禁改为 `planning_requested_ || emergency_pending_`；快照 waypoints 退化为
  **单点目标** = `waypoints_[waypoint_consumed_]`（相邻下一个 keypoint），`final_segment` =
  目标为最终关键点；失败分支优先级：EMERGENCY（跳过候选，ttc<1.0s 制动 / 否则 PLAN_FAILED）→
  候选（tryYieldReplan）→ PLAN_FAILED；
- `collisionCheckTick`：删除候选生成/提交，只检测冲突（<1.3×clearance）→ 置 `planning_requested_`；
- `runSafetySupervisor`：EMERGENCY 只置 `emergency_pending_`（记录 primary peer + ttc）+ 唤醒，
  不再直接 replan/制动/改轨迹；
- `checkArrival`：每 tick 分支 A（距当前目标 <0.5m → 切下一关键点，<0.5m 丢弃、3.5m 上限）/
  分支 B（距最终关键点 <0.2m → COMPLETED，稳定窗口 stable_s）；
- `advanceTargetLocked`：分支 A 推进（跳过过近关键点，最远 3.5m）；
- `tryEmergencyBrakeLocked`：方案 Y 制动提交（EMERGENCY_BRAKE → 耗尽 → BRAKE_HOLD 锁存）；
- 删 POSE_STALE 常量/路径；HOLD 锁存位置不随 pose 漂移；
- `yield_candidates.cpp`：SLOW 档位 {0.7,0.5}→{0.5,0.3}，LEFT/RIGHT 档位 {1.0,0.7}→{0.7,0.5}；
- launch：删 `planning_horizon`/`position_tolerance_m`，加 `reach_thresh_m`/`max_advance_dist_m`/
  `arrival_reach_thresh_m`（0.5/3.5/0.2）。

### 1.2 swarm_uav_executor

- `ego_swarm.py`：`waypoint_densify_spacing` 2.0→3.0；删 `_STATE_POSE_STALE` 与 `_TERMINAL_BAD`
  中 POSE_STALE；
- `uav_executor_ego.launch`：`waypoint_densify_spacing=3.0` + 新阈值参数，删过期参数。

### 1.3 计划偏离（用户确认）

- **到达阈值 0.1m → 0.2m**：计划 §3.2 分支 B 原定 0.1m；SITL 实测 PX4 稳态位置偏差 0.13-0.17m，
  0.1m 阈值导致长距离任务末段停稳后永不触发 COMPLETED → checkArrival 5s 安全超时 HOLD
  （Run 3 staging 复现）。用户确认放宽到 0.2m（匹配 executor `position_tolerance_m=0.2` 惯例）。
## 4. EMERGENCY 注入（方案 Y 端到端）

脚本：`.tmp/scripts/emergency_inject_uav2.py`（把本机 trajectory_intent 改写为 UAV1 同刻重叠
intent 发布到 `/neighbor_intent`）+ `.tmp/scripts/send_task_uav2.py`（任务发送）。

- 结果：UAV2 任务中注入重叠 intent → 监督层 `level=EMERGENCY primary_peer=UAV1` → 置
  `emergency_pending_` → replanOnce 常规 replan 失败 + ttc 紧急 → 提交制动 → `EMERGENCY_BRAKE`
  → 制动轨迹耗尽 → **BRAKE_HOLD**（executor 收口 `FAILED BRAKE_HOLD`）。
- 证据：`runtime_logs/ego_planner/UAV2-ego-planner.log`（`state=EMERGENCY_BRAKE level=EMERGENCY`）、
  `.tmp/logs/inject_task.log`（`task_state: FAILED BRAKE_HOLD`）。
- **验证方案 Y 提交唯一化端到端生效**：监督回调只置位，制动由 replan 线程提交。

## 5. 悬停 peer 绕行（候选路径）

脚本：`.tmp/scripts/hover_peer_inject_uav2.py`（注入 UAV1 悬停在 UAV2 前方路径的静态 intent）。

- 结果：UAV2 规划器在悬停点附近自然绕开（min_clear 满足 1m 硬门，`COMPLETED`），**未提交
  LEFT/RIGHT 候选**（单点阻塞下短段（3m）规划可自然让出）；候选路径由单元测试覆盖
  （`test_yield_candidates.cpp` 档位/min_clear、状态机 right-of-way/episode）。
- 说明：悬停机 COMPLETED 后不再发布 intent，邻机预测窗依赖既有 intent（时间窗耗尽后仅 executor
  odom 门禁兜底）——悬停绕行的候选触发需持续 intent 刷新场景，本次未强制构造。

## 6. 清理与残留检查

- 测试结束 HOLD 全部测试机（UAV1/UAV2 exec_state=HOLD），`cleanup.sh` 逆序停止。
- 残留检查：gazebo/px4/rosmaster/mavros/bridge 进程 0、测试端口（39001/11300/11345/11311-11312）
  全部释放。

## 7. 结果汇总

| 项目 | 结果 | 备注 |
|---|---|---|
| 单元测试（ego/executor/tcp） | 140/89/27，0 failure | 全量 257 tests 0 failure |
| plan-exec 三分支 + 单一入口 + 方案 Y | ✅ 单测 | 推进/丢弃/到达/置位唤醒/制动分支覆盖 |
| 候选档位（SLOW{0.5,0.3}/L/R{1,1.5,2}×{0.7,0.5}） | ✅ 单测 | min_clear ≥1.3、right-of-way、episode |
| 删 POSE_STALE / HOLD 锁存 | ✅ 单测 | HOLD setpoint 不随 pose 漂移 |
| 双机正交 SITL | ⚠️ 2/5 PASS | 竞争遗留：安全 HOLD 兜底，互惠收敛未完成 |
| EMERGENCY 注入（方案 Y） | ✅ PASS | EMERGENCY_BRAKE → BRAKE_HOLD，replan 线程提交 |
| 悬停 peer 绕行 | ⚠️ 自然绕开 | 单点阻塞下规划器自让，未强制候选 |
| 0.2m 到达阈值（用户确认偏离 0.1m） | ✅ | 修复 Run3 staging 到达 HOLD |

## 8. 遗留项（未验证/未解决）

1. **正交交叉竞争（min_3d 可达 0.605m）**：两机同时到达交叉点关键点时，STAND_ON 侧 replan 失败
   收口 HOLD 或实际间距 <1m 由 executor 门禁兜底；2/5 轮双方 COMPLETED。计划 §11 已识别互惠
   收敛未完成；候选只在 replan 失败分支，未解决"双方同时不失败"的时序。
2. **悬停机 intent 不刷新**：COMPLETED 后不再发布 intent，邻机预测窗耗尽后仅 odom 门禁兜底；
   悬停绕行候选的 SITL 强制触发未完成（单测已覆盖）。
3. **换位（swap）场景 SITL** 未执行（reciprocal 脚本为固定正交场景；换位为同线对向，几何更紧）。
4. **takeoff 首跑偶发时序失败**：active 重启后 MAVROS 未完全就绪，手动重跑成功（既有）。
5. **catkin test flaky**：`test_ego_state_machine` 内部 roscore 端口 11411 偶发被残留占用。


## 9. 迭代 A/B（候选机制增强，2026-08-26 23:xx）

### 9.1 修改内容（用户确认）

1. **A：STAND_ON replan 失败 → 只生成 SLOW 变速候选**（`tryYieldReplan` 低 ID 不再直接
   `return false`，改 `generate(..., slow_only=true)` 只产 SLOW，不横向偏移）；
2. **B：常规规划成功但 min_clear < clearance_factor×1m=1.2m → 也生成候选替换**
   （`replanOnce` 在 safe_candidate 后加裕度检查，候选失败保持原轨迹不 PLAN_FAILED）；
3. **SLOW 档位 {0.5, 0.8, 1.2}**（含加速；速度约束 v_max×factor ≤1.5，ΔT 取绝对值）；
4. **LEFT/RIGHT lateral {1.3, 1.8, 2.2}**（scale_wait {0.7, 0.5} 不变）；
5. **验证门槛 yield_clearance_factor 1.3 → 1.2**；**cost = ΔT/T（绝对值）+ dev/2.0**。

### 9.2 单元测试

- 全量 **263 tests, 0 errors, 0 failures**（新增 slow_only、加速档速度约束、B 裕度触发）。
- 新增 `SlowOnlyGeneratesOnlySlowForStandOn`、`SlowAccelerateGearDoesNotExceedSpeedLimit`
  （yield_candidates）；`ReplanWithLowClearanceMarginCommitsCandidate`（state machine）。

### 9.3 SITL 双机正交复测（与 §3 原始版对比）

| 轮 | 原始版（§3） | 迭代 A/B |
|---|---|---|
| A | — | **PASS**（min_3d 1.414m，速度 1.11/1.07） |
| B | — | FAIL（UAV1 实测速度 1.504 超 1.5 判定；两机 COMPLETED） |
| C | — | FAIL（UAV1 规划失败 → HOLD，min_3d 1.149m 安全） |
| D | — | **PASS**（min_3d 1.533m，UAV1 速度 1.487） |
| E | — | FAIL（UAV1 实测速度 1.539 超 1.5；两机 COMPLETED） |
| F | — | **PASS**（min_3d 1.617m，UAV1 速度 1.409） |
| 汇总 | 2/5 PASS；3 轮双方 COMPLETED | **3/6 PASS；5 轮双方 COMPLETED** |

### 9.4 结论

- **A+B 有效提升任务成功率**：双方 COMPLETED 3/5 → 5/6；Run5 类 0.605m 距离门禁事件
  未再出现（本轮 min_3d 1.15-1.62m）。
- **新 FAIL 源：加速档 1.2 副作用**——轨迹速度 ×1.2 + 加速度 ×1.44 → PX4 执行超调，
  UAV1 实测速度 1.49-1.54 逼近/超过 1.5 判定上限（3 轮触发，其中 2 轮因此 FAIL）。
  **加速档速度余量待决策**（降档 1.1 / 保留 1.2 接受判定 / clamp 轨迹速度留余量）。
- Run C 类 UAV1 规划失败仍存在（STAND_ON SLOW 未能救活全部场景；诊断日志已加
  `yield replan: no valid candidate` 待下次采集）。

