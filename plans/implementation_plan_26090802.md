# 吸收 offboard 新状态机设计（051e84e 版）：请求优先级、输入版本、retry 语义与直接锁存制动计划

**计划日期：** 2026-09-08 02:23（v1）/ 02:4x（v2 修订，文件生成时间 24 小时制）
**来源：** `ego_planner_driver/notes/offboard_new_state_machine_260902.md`（`051e84e` 版，`1e409e7` 恢复为参考）
第 3.8/4.4/5.1/5.2 章；用户 2026-09-08 决策（retry 语义、制动直接锁存）。
**状态：** 计划已建立，未实施

## 0. 现状澄清：emergency 由谁检测、基于预测还是实时位置

记录事实供实施引用（源码锚点见 §3）：

- **检测者**：`ego_planner_driver_node.cpp` 的 `runSafetySupervisor()`（25Hz，构造函数 timer 注册，经
  `safetySupervisorTick` 调用），内部调用纯数值 `SafetyPredictor::evaluate()`。
- **判定基础 = 预测轨迹，不是实时位置**：以本机**已提交带时轨迹 `current_`** 与邻机
  `trajectory_intent` 在共同绝对时间窗内**同刻采样**（GPS bias 修正），计算 CPA/TTC/min_d；
  `level == EMERGENCY` 判定依据 `min_d <= protected_distance_m(1.0)` 或 `ttc <= emergency_ttc_s(1.5)`。
  实时 odometry 只作诊断（`self_odom_dev_m`）与 HOLD 锁存，不参与 EMERGENCY 判定。
- **触发链**：EMERGENCY → 只置 `emergency_pending_`（记录 primary peer + ttc）并唤醒 `replanLoop`
  （方案 Y）；制动决定在 replan 线程。
- **实时位置门禁是另一层**：executor runtime gate（fresh odom 3D<1.0m → HOLD）与规划提交锁内
  hard-clearance，均基于测量/最新 intent，不属于本 supervisor 的预测判定。

## 1. 背景与证据

`notes/offboard_new_state_machine_260902.md`（051e84e 版）第 4 章定义 `BRAKE_REQUEST > COMPLETE_REQUEST >
REPLAN_REQUEST` 仲裁、第 5 章定义 REPLAN 合并槽（pending + input_version + retry_not_before）与
replanOnce nominal→STAND_ON/YIELD 结构；第 3.8 明确 EMERGENCY_HOLD 直接锁存、不再先执行制动轨迹。
当前 `ego_planner_driver` 已实现单一 replan 入口与候选让路（≈5.2）；缺口在请求优先级、输入版本、
失败重试语义与制动动作。

近期验收证据（`test_record_26090106.md` 0/5、`test_record_26090722.md` 4/5）：A08/A09 互撞 EMERGENCY、
A09 INSUFFICIENT_DATA、UAV1 观测 UAV2 预测 min_d≈0.78m，且完成/失败收口与安全事件存在同帧竞争窗口。

## 2. 决策与边界

### 2.1 吸收（本次实施）

| 编号 | 主张/决策 | 落地形式 |
|---|---|---|
| A | BRAKE 优先于 COMPLETE（4.4） | 完成转换前若 `emergency_pending_` 置位则让位（不清标志），交 replan 线程 |
| B | 结果基于输入版本，过期丢弃（5.1 input_version） | replan 快照记录 `waypoint_consumed_`/intent 版本；提交前推进则丢弃重跑 |
| C | **不再使用 soft-landing 术语，一律 retry**；retry 周期 **0.6s**；相同输入最多 retry **5 次**，均参数化 | 替换 `plan_fail_timeout_s` 时间窗为输入签名一致下的失败计数收口；去 soft-landing 表述 |
| D | **制动直接锁存悬停 BRAKE_HOLD，不再使用制动轨迹**；BRAKE_HOLD 后**不允许切换到其他状态** | `tryEmergencyBrakeLocked` 直接 `steady_target_`+`kStateBrakeHold`；删 `kStateEmergencyBrake` 受控轨迹路径；入口守卫拒绝转出 |
| E | **唯一 emergency 锁存判据改用实时位置**：SafetyPredictor（预测）与 executor runtime gate（实时 odom）功能重复，且预测随 replan 更新快不适合作锁存依据 | 统一唯一检测点 = executor/EgoSwarmDriver 单 monitor（EGO 巡航与 PI/follower 同层共用），用 fresh 邻机实时 3D 中心距；SafetyPredictor/预测只作提前 replan/预警，不再直接触发制动锁存 |
| F | **组级安全广播（051e84e §2.7/3.8 需求，当前未实现）**：任一机 BRAKE_HOLD/安全失败 → tcp_to_ros 自动向全组广播安全指令 → 全体进入 BRAKE_HOLD 锁存悬停，等手动接管降落 | backend 识别安全类终态 error_code 白名单 → 全组 UavHold(reason=GROUP_SAFETY)；executor 按 reason 进 BRAKE_HOLD 锁存语义（配合变更 D 锁存守卫），拒绝新任务/START，人工复位 |

### 2.2 不吸收（明确不做，理由）

| 主张 | 原因 |
|---|---|
| 状态机改名（INIT/COMPLETED_HOLD/EGO_EXECUTING/PI_EXECUTING/EMERGENCY_HOLD） | 目标设计名非现状；当前 8 态已测；改名只增映射成本 |
| PI 跟随迁入 driver（PI_EXECUTING） | follower PI 在 executor `_follower_loop` 已测；跨层迁移超出范围 |
| 新增 `ReplanSlot` 结构体类型 | `planning_requested_` bool 已实现合并；变更 C/D 用参数与守卫落地，不引入新结构 |

## 3. 现状对照（源码锚点）

| 项 | 现状 | 锚点（ego_planner_driver_node.cpp） |
|---|---|---|
| EMERGENCY 检测 | 25Hz `runSafetySupervisor`+`SafetyPredictor::evaluate`，预测同刻 min_d/ttc | 370/920/1528、1583-1601 |
| EMERGENCY 触发 | 只置 `emergency_pending_`+唤醒（方案 Y） | 1583-1601 |
| 完成转换（两路径） | 完成前未查 `emergency_pending_` 且清空它 | 769/809、1601/1633（808/1632 清标志） |
| 输入版本 | 仅 `safety_generation_`（制动后防覆盖），无 waypoint/intent 代次 | 504/704；snapshot 目标 521-527 |
| 失败重试 | 1s 定时 `timed_trigger` soft-landing + `plan_fail_timeout_s`=10s 收口 | 150/482-486/647-671/1972-2003/2020 |
| 制动 | `tryEmergencyBrakeLocked`→EmergencyBrakeGenerator→`kStateEmergencyBrake`→耗尽→`kStateBrakeHold` | 828-870/1431-1437 |
| BRAKE_HOLD 可被转出 | onGoal/onWaypoints(resetRun)→EXECUTING；onHold→HOLD；direct_control→HOLD | 977-1010/1183/1203-1211/1248 |
| 实时距离门禁 | EGO 巡航与 follower loop 各一处 `_distance_breach`（fresh odom 3D<1.0m）；PI/EGO 均在 executor 层 | ego_swarm.py ~414（EGO）、~494（follower） |
| 组级安全联动 | backend `_safe_hold` 仅对活动参与机发普通 HOLD；UavHold.reason 未分类；无锁存语义/未覆盖 reserve | backend `_safe_hold`；executor `handle_hold/_run_hold`；051e84e §2.7/3.8 需求未实现 |

## 4. 文件级改动预期（跨包：`ego_planner_driver` / `swarm_uav_executor` / `tcp_to_ros`；不改 msg/srv，安全指令复用 UavHold.reason 字符串约定）

### 变更 A：安全事件优先于完成
- `src/ego_planner_driver_node.cpp`：`handleArrivalBeforeSnapshotLocked`（769）开头 guard 增
  `|| emergency_pending_`；完成分支（808 前）`if (emergency_pending_) return false;`（不清标志）。
  `checkArrival`（1632 前）同样让位。

### 变更 B：规划目标/输入版本校验
- `ReplanSnapshot` 增 `size_t target_consumed`；snapshot 记 `waypoint_consumed_`（521-527）；提交前
  （704 附近）若 `snap->target_consumed != waypoint_consumed_` → 丢弃结果、不发布 intent、不清
  pending，下轮以新目标重跑（不清 `plan_fail_since_`/retry 计数）。

### 变更 C：retry 语义（去 soft-landing，参数化）
- 新增参数（canonical `config/planner_defaults.yaml` + `ego_planner_driver.launch` +
  `uav_executor_ego.launch` + 节点读取校验）：`replan_retry_period_s = 0.6`（失败态 retry 周期）、
  `max_same_input_retries = 5`（同输入连续 retry 上限）。
- `replanLoop`：`wait_for` 在失败/retry 态用 `replan_retry_period_s_`（常态 `replan_period_s_` 语义不变）；
  移除 soft-landing 措辞。
- 收口：废弃 `plan_fail_timeout_s` 时间窗；改为"输入签名一致（`waypoint_consumed_`/primary intent
  traj_id 与 epoch/目标未变）连续失败计数 ≥ `max_same_input_retries` → `EGO_PLAN_FAILED`"；输入变化或
  成功提交则清零计数。
- 日志/README/测试去 soft-landing 术语。

### 变更 D：制动直接锁存悬停 + BRAKE_HOLD 锁存守卫
- `tryEmergencyBrakeLocked`（828）：不再调用 `EmergencyBrakeGenerator`；决定制动时在当前样本位置锁存
  `steady_target_`（位置/yaw）并 `setState(kStateBrakeHold)`（mask 2552 悬停发布），删除
  `kStateEmergencyBrake` 提交与 `emergency_braking_` 轨迹段（1431-1437）。
- 保留"EMERGENCY 先一轮常规 replan 绕开尝试"语义（replan 成功即清 `emergency_pending_`，不进入制动）。
- BRAKE_HOLD 锁存守卫（不允许切换到其他状态）：
  - `onGoal`/`onWaypoints`（977/997）：BRAKE_HOLD 下忽略新目标，不 `resetRun`/不置 planning，保持
    BRAKE_HOLD；
  - `onHold`（1183）：BRAKE_HOLD 下仅保持（不 `processHold` 转 HOLD）；
  - `setDirectControlActive(true)`（1203）：BRAKE_HOLD 下忽略（不转 HOLD）；
  - `replanLoop`/`replanOnce` 的 `timed_trigger` 兜底在 BRAKE_HOLD 下不产生常规 replan（现状已由
    锁存/无 pending_goal 保证，补守卫断言）。
  - 退出仅：offboard 丢失或人工/遥控接管（软件层保持悬停锁存）。
- 删除/停用 `EmergencyBrakeGenerator` 调用路径与其 `kStateEmergencyBrake` 单测；`emergency_brake.cpp`
  生成器及其测试按删除处理（或独立提交清理，不能残留不可达路径）。
- 保留参数组（`emergency_decel/jerk/velocity/min/max duration`）：不再被直接制动轨迹消费，标注废弃或
  删除前先核对安全参数表（26082916）引用，避免假配置残留。

### 变更 E：唯一 emergency 锁存判据 = 实时位置（executor 层单 monitor）

背景：SafetyPredictor（预测 min_d/ttc）与 executor runtime gate（fresh odom 3D）均能触发安全动作，
功能重复；且预测随 replan/intent 更新变化快，不宜作进入锁存的唯一依据。PI 控制位于 executor
（`_follower_loop`），与 EGO 巡航同层，故唯一检测点应放在 executor/EgoSwarmDriver。

- `ego_swarm.py`：把 EGO 巡航（~414）与 follower（~494）两处 `_distance_breach` 收敛为**单一执行期
  monitor**（EGO 与 PI 共用同一执行期监控线程/频率，阈值统一用 `min_center_distance_m`，fresh odom
  实时 3D 中心距）。触发即安全失败（`MIN_DISTANCE_BREACH` 或进入变更 D/F 的锁存路径），不再双处判定。
- `ego_planner_driver`：SafetyPredictor 预测 EMERGENCY（`min_d≤1.0`/`ttc≤1.5`）**不再直接驱动制动锁存**；
  改为“需立即绕开”的 replan 输入（比 10Hz collision 更早触发的预测级预警），与变更 D 的“直接锁存”
  只由实时门禁事件 / 组级安全指令触发保持一致。预测层仍保留为主动 replan/让路依据。
- 边界：实时门禁滞后由制动锁存本身承担（检测即锁存当前位置，配合变更 D 无制动轨迹）；移除 SafetyPredictor
  直接制动属大改动，实施需以 A/B 保留开关过渡并回归（见 §5/§8）。

### 变更 F：组级安全广播（任一机 brake hold → 全组 BRAKE_HOLD 锁存，051e84e §2.7/3.8）

- `gcs_a_backend_node.py`：UavTaskState 终态 error_code 白名单识别“安全事件”（`BRAKE_HOLD` /
  `MIN_DISTANCE_BREACH` / `EMERGENCY_BRAKE` 等安全锁存类；普通 `EGO_PLAN_FAILED`/超时不在列）→
  在现有整组收口之外，触发 `_broadcast_group_safety_hold`：向目标集合（参数
  `group_safety_hold_targets = all_enabled|participants`，默认 `all_enabled` 全 15 机含 reserve）并发
  调用 `UavHold`，`reason = "GROUP_SAFETY"`（复用现有分批/重试模式，不新增 msg/srv）。
- `swarm_uav_executor/executor.py`+`drivers/ego_swarm.py`：`UavHold(reason=GROUP_SAFETY)` 与普通
  收口 HOLD 区分——进入 BRAKE_HOLD 锁存语义（driver 锁存悬停，变更 D 锁存守卫），并置本机“安全锁存”
  位：拒绝后续普通 command/START，仅人工复位（节点重启或明确复位入口，实施时核对现有复位途径并补
  最小复位机制，避免影响地面联调）。
- 广播为异步、不阻塞 Action/TCP 失败返回；被广播机即使未参与当前任务也进入锁存（含 reserve）。

## 5. 测试计划

### 5.1 单元测试（`test/test_ego_state_machine.cpp`）
- A：`EmergencyPendingPreventsCompletedAtArrival` / `...TickArrivalCompletion`（两路径完成让位且不清标志）。
- B：`StaleReplanTargetDroppedAfterAdvance`（快照后推进 → 丢弃，无 intent，下轮重跑）。
- C：`RetryPeriodAndSameInputCap`（0.6s 参数读取；同输入连续失败第 5 次收口 `EGO_PLAN_FAILED`；
  推进/新 intent 变化清计数；成功提交清计数）。
- D：`EmergencyBrakeDirectBrakeHoldNoTrajectory`（EMERGENCY 决定制动 → 直接 `kStateBrakeHold`，
  无 `kStateEmergencyBrake` 中间态、无制动 intent PHASE_BRAKING）；
  `BrakeHoldRejectsGoalHoldAndDirectControl`（BRAKE_HOLD 下 onGoal/onWaypoints/onHold/direct_control
  均不转出，状态保持）。
- E：`PredictorEmergencyNoDirectBrakeLock`（SafetyPredictor EMERGENCY 只触发 replan/预警，不再经
  `emergency_pending_`→制动锁存路径进入 BRAKE_HOLD；锁存仅由实时门禁事件/组级安全指令触发）。
- 移除/改写 `test/test_emergency_brake.cpp` 中制动轨迹生成用例为直接锁存语义。

### 5.2 回归与冻结契约
- `catkin build ego_planner_driver`；`catkin test ego_planner_driver`；`catkin_test_results --all build`
  0 failure。
- 30Hz / 0.2s candidate timeout / 0.1s TimedTrajectory / 1m 3D 门禁 / Safety active / PositionTarget 契约不回归。
- 参数校验：`replan_retry_period_s`/`max_same_input_retries` finite/正数/整数校验，fail-fast。

### 5.3 联动核对项（实施必须验证，见 §7）
- executor `hold()` 在 planner BRAKE_HOLD 下的确认与上报不超时/不阻塞（backend ALL 收口链路）。
- `kStateBrakeHold` 仍映射 executor TERMINAL_BAD（FAILED 上报），后端整组收口可用。
- F：`UavHold(reason=GROUP_SAFETY)` 全链路（backend 白名单→广播→executor 分类→BRAKE_HOLD 锁存→
  拒绝新任务/START）在 mock 多 Master 与 PX4 SITL 验证；reserve/未参与机被覆盖；人工复位途径可用。

### 5.4 E/F Python/backend 测试
- E：`SingleMonitorCoversEgoAndFollower`（EGO 巡航与 follower 共用同一执行期 monitor，同一阈值；
  注入 fresh odom 3D<阈值 → 统一 MIN_DISTANCE_BREACH 收口，无双处路径）。
- F：`GroupSafetyHoldWhiteList`（error_code 白名单识别）+ `GroupSafetyBroadcast`（backend 向配置集合
  广播 reason=GROUP_SAFETY，含 reserve）+ `ExecutorGroupsSafetyLock`（reason 分类进入锁存，后续
  START/任务被拒）。
- 参数校验：`group_safety_hold_targets` 枚举白名单；monitor 频率/阈值 finite/正数。

## 6. 实施顺序
1. 变更 C（参数 + retry 计数 + 去 soft-landing）→ 单测
2. 变更 A（守卫）→ 单测
3. 变更 B（版本校验）→ 单测
4. 变更 D（直接锁存制动 + 锁存守卫 + 清理制动轨迹路径/测试）→ 单测
5. 变更 E（executor 单 monitor 唯一化 + SafetyPredictor 去直接锁存，A/B 过渡开关）→ 单测/回归
6. 变更 F（backend 白名单+全组广播；executor reason=GROUP_SAFETY 锁存与复位；跨包）→ 测试
7. 联动核对 executor hold/上报与人工复位；全量回归
8. 写测试记录 `test_records/test_record_26090802.md`（实施当日）

## 7. 回滚与提交
- 基线：实施前 `ego_planner_driver`/`swarm_uav_executor`/`tcp_to_ros` HEAD/status。变更 A/B/C/D 各自
  独立回滚；E（monitor/SafetyPredictor 开关）与 F（backend+executor+reason 约定）为跨包集合，须连同
  对应测试一起回滚；不改 msg/srv，`reason=GROUP_SAFETY` 字符串约定随实现同步。
- 提交：按 `ego_planner_driver` 仓库分批 Conventional Commit，不 push；不覆盖既有修改
  （`tcp_to_ros` 的 t_off/阈值改动与当前计划文档不动）。

## 8. 完成定义
- 完成转换两路径在 `emergency_pending_` 置位时让位且不清标志（单测断言）；
- 规划结果基于过期目标/输入被丢弃并下轮重跑（单测断言）；
- soft-landing 术语清零；retry 周期 0.6s、同输入最多 5 次（参数生效并校验）；
- EMERGENCY 决定制动时直接锁存 `BRAKE_HOLD`，无制动轨迹中间态（单测断言）；
- BRAKE_HOLD 后 onGoal/onWaypoints/onHold/direct_control 均不可转出（单测断言）；
- executor hold/backend 收口在 BRAKE_HOLD 下验证通过；`ego_planner_driver` 全量自动测试 0 failure；
- E：进入 BRAKE_HOLD 锁存只有一个触发源（executor 实时 3D 门禁事件 / 组级安全指令）；SafetyPredictor
  预测不再直接驱动制动锁存（单测/回归断言）；
- F：任一机安全失败 error_code 命中白名单 → backend 向配置集合广播 GROUP_SAFETY → 全体（含 reserve）
  BRAKE_HOLD 锁存且拒绝新任务/START，人工复位后恢复（测试断言）；
- §2.2 不实施清单无一项被执行。
