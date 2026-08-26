# 双机正交交叉 ×5 + prepare 批次（15 机全 active）测试记录

时间：2026-08-27 00:xx-02:xx CST
记录人：____________
测试环境：PX4 SITL + Gazebo（无头）+ MAVROS + ego_planner_driver + swarm_uav_executor + swarm_topology_bridge + GCS_A + TCP runtime
对应实施计划：`plans/implementation_plan_26082621.md`（速度上限入驱动基线）
基线：`ego_planner_driver 93c2cf9`、`swarm_uav_executor c5588b1`、`tcp_to_ros 2a987e0`
前置操作：`.ros_home` 清空；`.tmp/safety_logger_*` 清理；swap/triple 修改与提交已 `git reset --hard` 回滚

## 0. 测试目标

1. **双机正交交叉（UAV1 ↔ UAV2）重复 5 次**，记录成功率（正交 8/9 PASS 基线的稳定性复测）；
2. **prepare 批次任务（A01-A12 MOVE_TO，3×4 点阵）**，UAV1-15 全部启用最新机载程序且**全部 active 监督**
   （全机相互避碰），记录批次任务结果。

## 1. 测试 1：双机正交交叉 ×5

- 编排：`verification/ego_smoke/reciprocal_15sitl.py`（UAV1 stage(2,5) goal(18,5)，UAV2 stage(18,5) goal(2,5)，同线对向正交），
  `--no-require-rebound --execute`，速度上限 `yield_max_velocity_mps=2.5`。
- 环境：`startup_ego_sitl.sh "1 2" active` 干净启动（sim 15 机 + onboard 15 机 shadow + gcs + 标定 + 1/2 active 重启 + takeoff）。

| Run | 结果 | min_3d (m) | UAV1 峰值速度 | UAV2 峰值速度 | 备注 |
|---|---|---|---|---|---|
| R1 | **PASS** | 1.388 | 1.09 | 1.12 | 干净起点 |
| R2 | **PASS** | 1.559 | 1.08 | 1.10 | 连续第二跑 |
| R3 | FAIL | 0.654 | 1.17 | 1.11 | UAV1 HOLD_FAILED，UAV2 MIN_DISTANCE_BREACH（正交竞争门禁兜底） |
| R4 | **PASS** | 1.258 | 1.47 | 1.10 | R3 后 RTL 破僵局；中间 3 次 staging 失败（见下） |
| R5 | **PASS** | 1.608 | 1.07 | 1.12 | 最后连续跑 |

- **成功率：4/5 = 80%**（PASS 判据：min_3d ≥ 1.0 m 且双机均 COMPLETED）。
- R3 失败为已知正交竞争：两机同时接近交叉点时实际间距 <1.0m，执行层距离门禁 MIN_DISTANCE_BREACH 兜底收口，
  UAV1 未确认 HOLD 超时（HOLD_FAILED），整组安全收口，无碰撞。

### 1.1 过程中故障与处置

- **R3 失败后遗留 <1m 僵局** → R4 首跑 staging 时 UAV1 MIN_DISTANCE_BREACH（从失败点出发被邻机门禁挡）：
  处置 = RTL 落地 + 重新起飞，起点干净后重跑。
- **UAV2 HOLD_FAILED LOCAL_TIMEOUT（egoswarm 未确认 HOLD）**：连续 2 次 staging 失败；
  诊断 `exec_state` 卡在 `TAKEOFF`，UAV2 机载层状态机不同步。处置 = `startup_offboard_ego.sh stop 2` + 以
  `EGO_SAFETY_SUPERVISOR_MODE=active` 重启 UAV2 机载层，重新起飞后恢复。
- **UAV1 bridge service timeout**（`/UAV1/uav_hold -> UAV1`）：UAV1 executor 收到 SIGINT 退出，
  `required=true` 连带关闭整个 UAV1 机载层（offboard loss → 自动 LAND）。处置 = 重启 UAV1 机载层 + 重新起飞。

## 2. 测试 2：prepare 批次任务（15 机全 active）

- 环境：`startup_ego_sitl.sh "1 2 ... 15" active` 干净启动（sim + onboard 15 shadow + gcs + 标定 +
  15 机全部 active 重启 + 15 机全部 takeoff），TCP server 由 `run.sh tcp` 启动（39001）。
- 验证 15 机 `safety_supervisor_mode` 全部为 **active**。
- 命令：`python3 references/runtime/task_test.py --backend tcp --group A --task prepare --tcp-host 127.0.0.1 --tcp-port 39001 --timeout-s 180`
- 任务内容：A01-A12 各自 `MOVE_TO`（3×4 点阵，x∈[12.5,27.5]、y∈[45,55]、z=12）。

### 2.1 结果

- **FAILURE**，`failed_robot_ids: ["A05"]`，`ticks: 503`（约 50s 收口）。
- 各机终态：A02 `EGO_PLAN_FAILED`、A05 `EGO_PLAN_FAILED`、A11/A12 `COMPLETED`、其余 8 机 `EXECUTING`
  （FAILURE 整组收口，活动机安全停在原地 HOLD 流程）。
- 本次失败**非** 26082420 的距离门禁（MIN_DISTANCE_BREACH），而是 **EGO_PLAN_FAILED**：
  15 机全部 active 相互避碰约束下，密集编队转场时 A02/A05 规划器无法生成无碰撞可行轨迹
  （`completion_policy=ALL` 整组 FAILURE，其余机安全）。

### 2.2 与 26082420 对比

| 项 | 26082420（15 机 shadow） | 本次（15 机全 active） |
|---|---|---|
| 失败机 | A03、A05 | A02、A05 |
| 失败类型 | MIN_DISTANCE_BREACH（爬升/转场间距 <1.0m 门禁） | EGO_PLAN_FAILED（避碰约束下规划无解） |
| 失败时间 | ~33s | ~50s |

- 结论：全 active 模式下执行层门禁未触发（间距控制更好），但规划器在 15 机互相避碰的密集转场场景下
  出现规划无解（EGO_PLAN_FAILED），批次整组安全收口。后续方向：编队转场分层/错峰、或规划失败重试策略。

## 3. EGO_PLAN_FAILED 修复（软着陆方案，已实施）

### 3.1 改动（ego_planner_driver）

1. **失败软着陆 + 10s 收口**（`replanOnce` 主失败分支）：
   - 常规 replan 失败（含候选无解）**不立即** `EGO_PLAN_FAILED`：保持当前轨迹继续执行
     （末端自然悬停），由 replanLoop **1s 定时 + 冲突/推进事件**继续重试；
   - `plan_fail_since_` 记录首次失败时刻，**连续失败 ≥10s**（`plan_fail_timeout_s_`）
     才收口 `EGO_PLAN_FAILED`（executor 收口 → backend 对全组活动机发 HOLD）。
2. **1s 定时触发修复**（`replanLoop`）：`wait_for` 超时（`timed_out`）也执行一轮
   `replanOnce(timed_trigger=true)`（原代码定时超时被事件门禁跳过，软着陆重试无节奏）。
3. **steady_target_ 例外**：软着陆失败中（`plan_fail_since_` 非零）即使 HOLD 捕获已锁存
   `steady_target_` 也放行定时重试（HOLD 状态首轮失败可恢复/可超时收口）；成功提交后清计恢复拦截。
4. **成功提交清计**：常规提交段 / `commitYieldCandidate` / `tryEmergencyBrakeLocked`
   （制动/fallback）提交后 `plan_fail_since_` 清空——恢复后旧失败累计不直接收口。
5. **cost 惩罚降低**（`yield_candidates.cpp`）：LEFT/RIGHT 候选 `dev/2.0` → **`dev/5.0`**
   （横向让路代价更小，更易选中横向绕行而非撞墙）。
6. **悬停判定放宽**（`yield_candidates.h`）：`hovering_velocity_mps` 0.1 → **0.2**
   （邻机末速 <0.2m/s 判近似悬停 → 跳过 SLOW，更贴合编队低速场景）。

### 3.2 全组 HOLD（backend）

- 任一机 executor 上报 `FAILED` → backend `_on_status` 记入 `failed_uavs` →
  `_wait_for_terminal` break → `_safe_hold`（对全部 accepted + 非 terminal 活动机并发
  `/UAVn/uav_hold`）。26082700 实测 A02/A05 EGO_PLAN_FAILED 后整组安全收口即此链路。
- **26082700 补充修复**（backend `_safe_hold` 分批重试）：
  - 实测中 A12 `/UAV12/uav_hold` **经 bridge service 转发超时**（UAV12 executor 无
    HOLD 请求记录）→ 该机未确认 HOLD；
  - 修改：`_safe_hold` **每批 4 机**下发（`hold_batch_size`，降低 bridge 并发负载）、
    **0.5s 间隔重试 2 次**（`hold_retry_count`/`hold_retry_interval_s`，bridge 瞬时
    超时恢复）；单测 `test_safe_hold_batches_and_retries`（6 机 4+2 两批 + UAV5
    首败重试成功，最终全部确认）。

### 3.3 单元测试

- `ReplanFailSoftLandingResetsOnCommit`：首败软着陆计时 → peer 挪开 → 定时 replan 成功 →
  计时清空（新增）。
- `YieldLowIdStandOnRequestsReplan` / `YieldReplanStandOnFailsToPlan`：改为软着陆语义
  （首败不收口；累计 >10s 收口）。
- 全量 **265 tests, 0 errors, 0 failures**。

## 4. 遗留问题记录

1. `startup_ego_sitl.sh` 不启动 TCP server（39001），`--backend tcp` 前需手动 `run.sh tcp`；
2. 双机交叉连续跑在失败后遗留 <1m 僵局（RTL 可破）；UAV2 egoswarm HOLD 同步偶发失败（重启机载层可恢复）；
3. UAV1 bridge service timeout 连锁导致 executor SIGINT → 整层退出（重启机载层恢复）。
