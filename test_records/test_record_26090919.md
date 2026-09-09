# 测试记录 26090919：runtime/组级直接 BRAKE_HOLD（implementation_plan_26090900）实施与验收

时间：2026-09-09 19:40 CST（实施 + 单测 + 受保护 prepare 验收结束）
记录人：cline-deepseek
对应实施计划：`plans/implementation_plan_26090900.md`（runtime/当前任务 UAV 集群 brake hold）
工作空间：`/home/ub20tg/catkin_swarm6-2`

## 0. 摘要

- **已实施**：统一所有安全出口到"直接 BRAKE_HOLD"——C++ 捕获本机当前位置/yaw →
  `steady_target_` → `BRAKE_HOLD`；删除 `EmergencyBrakeGenerator` 与
  `EMERGENCY_BRAKE` 制动轨迹路径；预测级 EMERGENCY 仍先 replan/让路（保留余地），
  绕开失败才锁存（§0 判据：`min_d ≤ 1.0` / `ttc ≤ 1.5`）。
- executor 实时 `MIN_DISTANCE_BREACH` 保留为最后兜底，且仍属组级 fan-out 白名单。
- runtime 指定 HOLD：`EmergencyHold.accepted=false/异常` 时 TCP 不再假 COMPLETED
  （→ FAILED/HOLD_REJECTED / HOLD_SERVICE_FAILED），目标按 request∩group 下发
  （`RUNTIME_BRAKE_HOLD`）。
- 组级安全收口：白名单安全终态 → 一次性对同任务参与者 fan-out
  `GROUP_BRAKE_HOLD:<code>:<uav>`（排除触发者/terminal/失联；A13–A15 reserve 不在集合）。
- **自动化测试全部通过**：全工作区 `catkin_test_results --all build` = **354 tests,
  0 errors, 0 failures**（ego 162、executor 115、tcp 72，其余包通过）。
- **SITL 受保护 prepare（4 轮，0 SUCCESS，3 次真实 MIN_DISTANCE_BREACH + 1 次排程 pose
  启动抖动）**：每轮物理 3D 破距（0.627–0.963 m @+34~43 s）真实触发 executor 兜底门禁 →
  **组级 fan-out 按设计工作**：后端 `GROUP_BRAKE_HOLD` 日志、其余参与机全部经
  `direct BRAKE_HOLD locked` 锁存（每轮 10 机、excluded=触发者+已 terminal、reserve 不涉
  及）、无提前锁存（事件前全程 nominal）。计划 §7.4-11 的"安全事件时参与者均直接
  BRAKE_HOLD、不含 reserve、不继续旧轨迹"路径被实测验证；但本会话未取得完整 SUCCESS
  轮，任务级验收不能视为通过（见 §4 对照与未验证项）。

## 1. 实施内容与提交

| 仓库 | HEAD | 提交 | 内容 |
|---|---|---|---|
| ego_planner_driver | `c772e96` | feat(planner): latch BRAKE_HOLD directly and drop EMERGENCY_BRAKE path | 直锁入口（/brake_hold + lockBrakeHoldLocked）；EMERGENCY 先 replan/yield 后直锁；删 EmergencyBrakeGenerator/kStateEmergencyBrake/emergency_braking_ 及其参数（safety_defaults.yaml/launch/README/CMake/单测）；BRAKE_HOLD 直接终态 |
| swarm_uav_executor | `a5e4a02` | feat(executor): route safety hold reasons to direct /brake_hold | driver 新增 /brake_hold publisher；hold() 按 reason（RUNTIME_BRAKE_HOLD / GROUP_BRAKE_HOLD:...）选择直锁路由并等 BRAKE_HOLD；普通 HOLD 语义不变 |
| tcp_to_ros | `29d779b`+`9894052` | feat(backend) / fix(tcp) | `_safe_hold(targets=...)`；runtime HOLD request∩group 交集、幂等（已 held 不重发）、保守 accepted；白名单组级 fan-out（`_maybe_start_group_safety_hold` 一次性）；TCP HOLD 诚实失败语义；纯单测 `test_tcp_hold_semantics.py` 注册 CMake |

不改 msg/srv；不改 planner 算法、schedule tau、clearance、intent 或 PX4 setpoint 出口。

## 2. 自动化测试

命令：`catkin build`（全 6 包 0 failure）→ `catkin run_tests -j3 -p1` →
`catkin_test_results --all build`。

| 层级 | 结果 |
|---|---|
| ego_planner_driver（含新 40 项状态机） | 162 tests, 0 errors/failures（新增 `EmergencyPredictionDirectLocksBrakeHold`、`DirectBrakeHoldRequestLocksAtCurrentPose`；更新 `EmergencyReplanFirstMitigatesCollision`；删除 `test_emergency_brake`/closed-loop 制动轨迹用例） |
| swarm_uav_executor | 115 tests, 0 errors/failures（新增 safety reason → /brake_hold 路由两例；BRAKE_HOLD 直锁终态回归） |
| tcp_to_ros（含 `test_tcp_hold_semantics.py` 新注册） | 72 tests, 0 errors/failures（HOLD 诚实语义；runtime 目标过滤/幂等/未知拒绝/部分不可执行保守拒绝；fan-out 一次性/排除规则/非白名单不触发） |
| 全工作区 | **354 tests, 0 errors, 0 failures, 0 skipped**（6 包全部成功） |

单测关键路径（计划 §8 测试门槛）：
- "预测可绕开不锁存"：交叉场景 supervisor EMERGENCY → replan 成功清除紧急标志继续
  EXECUTING（`EmergencyReplanFirstMitigatesCollision` 断言 ∈ {EXECUTING, BRAKE_HOLD}）。
- "预测紧急且 replan 失败/不可避直接锁存"：同轨迹 peer → replan/yield 失败 →
  **直接 BRAKE_HOLD**（无 EMERGENCY_BRAKE 中间态，steady_target_=本机当前 pos）。
- 提交唯一化：锁存后 replanOnce 不覆盖（steady_target_ 拦截）。
- fan-out：只对白名单错误码、只触发一次、reason=`GROUP_BRAKE_HOLD:<code>:<uav>`、
  排除触发者/terminal/失联、非白名单（EGO_PLAN_FAILED）不触发。

## 3. SITL 受保护 15-UAV prepare 验收（2026-09-09，4 轮）

驱动：`src/swarm_simulation/bashs/round_prepare_15sitl.sh --round N --task prepare
--timeout-s 150 --keep-ros-home-on-fail 1`（每轮 clean→sim 15→onboard 15/15→gcs
bridge/backend→calibration→tcp→takeoff 15/15→prepare→cleanup；cleanup 后进程 0、端口 clean）。
配置：`horizontal_speed_mps=0.7`（本会话改回）、`schedule_geometric_distance_m=1.3`、
`safety_margin=1.0`；Safety active；A01–A12 参与，A13–A15 reserve。

| 轮 | 时间 | 结果 | failed | ticks(≈s) | 事件与证据 |
|---|---|---|---|---|---|
| 1 | 19:07–19:09 | FAILURE | A06,A08 | 739(37s) | A06 `MIN_DISTANCE_BREACH`（UAV6-8 实测 3D 0.963m@+34.6s，<1m 0.8s/<1.1m 3.6s）→ fan-out targets=A01..A05,A07,A09..A12（排 A06 触发者/A08 已 terminal）→ 10 机 `direct BRAKE_HOLD locked` @+35.7~36.8s |
| 2 | 19:12–19:14 | FAILURE | A04 | 2(0.1s) | 排程前置 `SCHEDULE_POSE_UNAVAILABLE`（A04 pose 快照过期/缺失）——启动阶段 bridge/pose 环境抖动，任务未起飞；与安全逻辑无关 |
| 3 | 19:15–19:17 | FAILURE | A07,A11 | 981(49s) | A07 `MIN_DISTANCE_BREACH` → fan-out targets=A01..A06,A08..A10,A12（排 A07 触发者/A11 已 terminal）→ 10 机 direct BRAKE_HOLD locked @+49.0s |
| 4 | 19:18–19:21 | FAILURE | A06,A07 | 930(46.5s) | A06 `MIN_DISTANCE_BREACH`（UAV6-7 实测 3D **0.627m**@+42.5s，<1m 5.15s）→ fan-out targets=A01..A05,A08..A12（排 A06/A07 两失败机）→ 10 机 direct BRAKE_HOLD locked |

证据：
- 后端（`.ros_home_roundN/log/<run>/gcs_a_backend_node-1.log`）：`GCS_A group safety
  hold triggered by A06|A07 (MIN_DISTANCE_BREACH); targets=...`（每轮恰一行，fan-out 一次性）。
- 机载 C++（各 UAV `rosout.log`）：`direct BRAKE_HOLD locked at (x,y,z)`——锁存位置为
  本机自身当前位置，非 runtime 目标/邻机点；exec_state=BRAKE_HOLD。
- **reserve A13–A15 未收到任何 /brake_hold / uav_hold**（无直锁日志），符合 §0/§4.3 目标集合。
- 每轮事件前（0→+34~43s）无任何提前 BRAKE_HOLD/EMERGENCY 直锁 → 可解场景不过早锁存。
- ULog 分析：`.tmp/plots/r1_impl_260909/`、`.tmp/plots/r4_impl_260909/`
  （goal-epoch 1788952151.258 / 1788952837.469，ticks 739 / 930；pair 距离为同帧 ULog
  实测，heading 缺省 -0.006 仅影响目标坐标变换，不影响 pair 距离）。
- 失败轮留档：`.ros_home_round1/2/3/4`（257–373M，含 PX4 ulog/GCS 层日志/07_task JSON）；
  全程编排日志 `.tmp/logs/prepare_impl_260909_r{1..4}.log`；默认 `.ros_home` 已清空。

**机制判定**：三场物理破距均为 executor 实时 3D 兜底门禁（`MIN_DISTANCE_BREACH`，
最后兜底，§0）合理触发，非预测层提前锁存造成——破距前双方均按 nominal 执行；
组级 fan-out 与机载直接 BRAKE_HOLD 在真实场景端到端工作。任务层 SUCCESS 未达成。

## 4. 结果对照与未验证项

- **对照**：历史基线 SUCCESS 概率（26090816 回退后 4/5；26090902 v=0.6/geo1.3 时 4/5）
  明显高于本会话 0/4。本会话配置（v=0.7 改回 + geo1.3）与两个基线均不完全相同；3 场
  事件聚集在同类走廊汇合区（UAV6-9 族），物理轨迹收敛 <1m 的触发源（走廊汇合）与
  26090902 R2（0.812m）及 26090816 R5 同族。**样本小且叠加参数组合差异，本记录不把
  事件频率上升归因于本计划实现**；该趋势需后续多轮对照（建议按 geo=1.0/v=0.6 基线组合
  逐项复跑 5 轮）确认。
- **计划 §7.4-11 完成定义**：单次受保护 prepare 已执行但 **任务级 FAILURE**——
  按"验证内容以计划文档为准/不把未验证写成通过"原则，§9 成功标准 1/2/3/4/5 的路径级
  表现已观测（真事件→直锁、目标集合正确、一次 fan-out、TCP 无假 COMPLETED），但
  "完整 SUCCESS 轮（failed=[]）"未在本会话达成，**不得宣告整链路验收通过**。
- **未验证项**：
  1. prepare 完整 SUCCESS 轮（12/12 COMPLETED）——需在目标参数组合下继续对照复测。
  2. runtime 指定 HOLD 端到端（TCP HOLD → 指定 assignments 直锁）尚未在 SITL 单独验证
     （本次事件均为自动 fan-out 路径；runtime 指定语义由单测覆盖）。
  3. TCP 断线本地收口路径未在本会话单独注入验证（既有断线测试/单测覆盖）。
  4. 双机正交/相向让路对照回归：按用户修订的计划门槛不要求额外执行，未跑。
  5. `/brake_hold` 单次发布 vs roscpp 异步建链的竞态未压力验证（沿用现有 /hold 模式）。

## 5. 清理与残留

- 每轮 cleanup 后：gazebo/px4/rosmaster/mavros/bridge/tcp_server 进程数 0；
  测试端口（39001/11300/11310/11345）clean（每轮断言通过）。
- 全会话结束后再次断言 0 进程、端口 clean；默认 `.ros_home` 清空（保留文件夹），
  失败轮 `.ros_home_round1/2/3/4` 按失败留档保留。

## 6. 关联

- 计划：`plans/implementation_plan_26090900.md`；状态登记：`plans/README.md`「状态-26090900」。
- 历史对照：`test_record_26090722.md`（4/5）、`test_record_26090814.md`（1/5）、
  `test_record_26090816.md`（4/5）、`test_record_26090902.md`（v=0.6/geo1.3 4/5）。
- 代码基线提交：ego `c772e96`、executor `a5e4a02`、tcp `29d779b`/`9894052`。
