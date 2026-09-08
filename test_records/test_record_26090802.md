# 26090802：安全事件优先、目标代次、retry 语义、直接锁存制动与组级安全广播 测试记录

时间：2026-09-08（实施当日）  
对应实施计划：`src/tcp_to_ros/plans/implementation_plan_26090802.md`  
涉及仓库：`ego_planner_driver` / `swarm_uav_executor` / `tcp_to_ros`

## 0. 目标（计划 §1/§8 完成定义要点）

- A：完成转换两路径（`handleArrivalBeforeSnapshotLocked`/`checkArrival`）在 `emergency_pending_`
  置位时让位且不清标志；
- B：`ReplanSnapshot.target_consumed` 代次校验，提交前推进则丢弃重跑（不发布 intent/不清 pending）；
- C：删除 `plan_fail_timeout_s` 时间窗与 soft-landing 措辞；`replan_retry_period_s=0.6`、
  `max_same_input_retries=5`，同输入连续失败计数收口；
- D：制动直接锁存悬停 `BRAKE_HOLD`（去 `EmergencyBrakeGenerator`/`kStateEmergencyBrake` 路径），
  BRAKE_HOLD 不可被 onGoal/onWaypoints/onHold/direct_control 转出；
- E：唯一 emergency 锁存判据 = executor/EgoSwarmDriver 单一执行期 monitor（EGO 巡航与 follower 共用）；
  SafetyPredictor 预测不再直接驱动制动锁存；
- F：backend 安全 error_code 白名单 → 全组（默认 all_enabled 含 reserve）`UavHold(reason=GROUP_SAFETY)`；
  executor reason 分类进安全锁存，拒绝新任务/START，人工复位。

## 1. 修改文件

### ego_planner_driver
- `include/ego_planner_driver/ego_planner_driver_node.h`：删 `emergency_brake.h` include /
  `kStateEmergencyBrake` / `emergency_braking_` / `emergency_brake_generator_` / `plan_fail_timeout_s_`；
  新增 `replan_retry_period_s_`/`max_same_input_retries_`/`same_input_failure_count_`/失败签名；
  `ReplanSnapshot.target_consumed`；`sendBrakeHold`/`onBrakeHold`/`lockBrakeHoldLocked`；`brake_hold_sub_`；
  `replanInputSignatureLocked`；测试访问 `sameInputFailureCountForTest`。
- `src/ego_planner_driver_node.cpp`：A 完成让位（两路径）；B 提交前目标代次丢弃；C retry 计数收口
  + 0.6s 失败态 replanLoop 周期 + 输入签名；D `lockBrakeHoldLocked` 直接锁存 + 四入口守卫 + 订阅
  `/brake_hold`；E 监督 EMERGENCY 只置 `emergency_pending_`+`planning_requested_`；提交失败/资源收口
  计数清零；删除制动轨迹相关提交路径。
- 删除：`include/.../emergency_brake.h`、`src/emergency_brake.cpp`、
  `test/test_emergency_brake.cpp`、`test/test_closed_loop_dynamics.cpp`（D 移除制动轨迹路径）。
- `CMakeLists.txt`：删 emergency_brake 源与两个 gtest。
- `config/planner_defaults.yaml`：去 `plan_fail_timeout_s`，加 `replan_retry_period_s`/`max_same_input_retries`。
- `config/safety_defaults.yaml`：删 emergency 轨迹参数组（`emergency_replan_time_s` 等），保留
  `emergency_decel_mps2`（SafetyPredictor 用）。
- `test/test_ego_state_machine.cpp`：重写/新增 A/B/C/D/E 用例（见 §3）。
- `README.md`、`notes/current_source_runtime_architecture_260906.md`：同步 7 态/retry/直接锁存/BRAKE 语义。

### swarm_uav_executor
- `src/swarm_uav_executor/drivers/ego_swarm.py`：单一执行期 monitor（`_start/stop_breach_monitor`、
  `_breach_monitor_loop`、`_breach_failure`）；EGO `_wait_for_terminal` 与 `_follower_loop` 统一收口
  MIN_DISTANCE_BREACH；`_issue_brake_hold`（`/brake_hold`）；`hold()` 按 `reason=GROUP_SAFETY` 直接锁存；
  reason 常量。
- `src/swarm_uav_executor/executor.py`：`_safety_latch`；`handle_task`/START 在锁存时拒绝；
  `handle_hold` 按 `GROUP_SAFETY` 置位 / `GROUP_SAFETY_RESET` 复位。
- `test/test_ego_swarm_driver.py`、`test/test_executor.py`：单 monitor/group brake-hold/安全锁存用例。
- `README.md`：安全边界同步（E/F 语义）。

### tcp_to_ros
- `scripts/gcs_a_backend_node.py`：`SAFETY_ERROR_WHITELIST`；`group_safety_hold_targets` 参数读取/校验；
  `GroupState.group_safety_broadcasted`；`_on_uav_state` 触发 `_maybe_broadcast_group_safety`；
  `_broadcast_group_safety_hold`（分批/重试 reason=GROUP_SAFETY，all_enabled 含 reserve）。
- `config/backend_defaults.yaml`：`group_safety_hold_targets: all_enabled`。
- `test/test_gcs_a_aggregation.py`：白名单/all_enabled（含 reserve）/participants/单次触发用例。
- `plans/README.md`：登记计划与状态-26090802；新增 `test_records/test_record_26090802.md`（本文件）。

## 2. 构建与单测命令

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
catkin build ego_planner_driver swarm_uav_executor tcp_to_ros --no-deps
catkin test ego_planner_driver --no-deps -p1        # 164 tests, 0 errors/failures
catkin test swarm_uav_executor tcp_to_ros --no-deps -j1 -p1
catkin_test_results --all build/<pkg>                # 见 §3 计数
```

## 3. 结果

| 包 | 测试数 | 结果 |
|---|---|---|
| `ego_planner_driver` | 164 | **0 errors / 0 failures** |
| `swarm_uav_executor` | 117 | **0 errors / 0 failures** |
| `tcp_to_ros` | 60 | **0 errors / 0 failures** |

关键新增用例：

- `EmergencyBrakeDirectBrakeHoldNoTrajectory` / `BrakeHoldRejectsGoalHoldAndDirectControl`（D：直接锁存、
  无轨迹中间态、锁存守卫）；
- `PredictorEmergencyNoDirectBrakeLock`（E：预测 EMERGENCY 只触发 replan/预警，不进 BRAKE_HOLD）；
- `EmergencyPendingPreventsTickArrivalCompletion`（A：checkArrival 完成让位不清标志）；
- retry 语义用例（`ReplanFailRetryResetsOnCommit` / STAND_ON 5 次同输入收口 / `DegenerateWhileFailingClosesAfterTimeout`
  同步为计数收口）；
- Python：`test_single_monitor_covers_follower_loop`、`test_group_safety_hold_publishes_brake_hold`（executor）；
  `test_group_safety_lock_rejects_task_and_start_until_reset`、`test_ordinary_hold_does_not_set_safety_latch`；
- backend：`test_group_safety_broadcast_includes_all_enabled`（含 reserve 15 机）、
  `test_group_safety_participants_target`、`test_group_safety_whitelist_ignores_normal_failures`。

## 4. 冻结契约检查

- `replan_period_s=1.0`、`setpoint_rate_hz=30`、relay candidate timeout 0.2s、TimedTrajectory 0.1s、
  `swarm_clearance×2=1m` hard-clearance、`safety_supervisor_mode=active`、GPS bias、intent epoch/t[]、
  PositionTarget 契约、`setpoint_relay` 唯一 MAVROS 出口 —— 未改动（本计划不触碰）。
- 未改 msg/srv；组级安全指令复用 `UavHold.reason` 字符串约定。

## 5. 未验证项（诚实记录）

- EGO/PI 混编 + 组级安全广播的 PX4 SITL 端到端（任一机注入实时 3D 门禁事件 → 全组含 reserve BRAKE_HOLD
  锁存、拒绝新任务、`GROUP_SAFETY_RESET` 复位）未在本批自动测试内执行。
- 15-UAV active prepare 无头回归未执行（需全 clean 环境、SITL/Gazebo + 跨 master bridge 编排）。
- 本批只完成 build + 单测 + 包级自动测试；端到端按计划 §7/§8 另行受保护验收并归档日志/ULog。

## 6. runtime 主动 HOLD → 全组 BRAKE_HOLD 锁存（同日补充）

- 缺口：runtime TCP `HOLD` 原先走 backend `_handle_emergency_hold` → 普通 `_safe_hold`（reason 文本），
  executor 只进普通 HOLD、不置安全锁存位、不覆盖 reserve；全组 BRAKE_HOLD 只能由单机安全 error_code
  自动广播触发。
- 修复：`_handle_emergency_hold` 改为同步 `_broadcast_group_safety_hold(reason=GROUP_SAFETY)`，目标按
  `group_safety_hold_targets`（默认 all_enabled 含 reserve），participants（exec 层）全部确认才 accepted；
  置 `group_safety_broadcasted` 去重。executor/driver 的 GROUP_SAFETY 锁存与 `GROUP_SAFETY_RESET` 复位复用。
- `_broadcast_group_safety_hold` 增加返回值（成功确认的执行层集合），自动广播路径不受影响。
- 文件：`gcs_a_backend_node.py`、`config/backend_defaults.yaml`、`test/test_gcs_a_aggregation.py`、
  `plans/README.md`（本记录同步）。
- 结果：`test_gcs_a_aggregation` 13/13 passed；`swarm_uav_executor`/`tcp_to_ros` 包级回归不变。
- 限制：无活动组任务时不广播（返回 no active group）；普通 `_safe_hold`（批次正常收口）仍为软 HOLD 语义。
