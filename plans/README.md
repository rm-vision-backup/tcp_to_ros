# plans 目录（入口）

> 本目录是 `tcp_to_ros` 项目的开发入口：维护 **Phase 编号约定**、**全局设计规约**、
> **实施计划**与**当前状态**。新会话从此文件开始阅读。
> 测试证据按期归档：`test_records/test_record_2608xxxx.md`。

## Phase 编号约定（唯一权威）

本项目所有文档中的 Phase N 一律以本文件为准：

- Phase 1：接口与映射冻结
- Phase 2：GCS_A 分发与汇总
- Phase 3：`swarm_topology_bridge` 扩展
- Phase 4：机载执行适配层
- Phase 5：仿真联调
- Phase 6：实机联调

其他文档只使用"阶段"或直接引用本文件 Phase 编号，不自行建立并行的 Phase 编号体系。

## 全局设计规约（冻结）

### 跨机通信方案

采用 **GCS 本地 Action + 跨机 Service + 跨机 Topic**：

- 整组生命周期：本地 `ExecuteGroupTask.action`；
- 单机任务接收与 HOLD：跨机 Service；
- 执行反馈与终态：跨机 Topic；
- 不扩展完整跨机 Action；`swarm_topology_bridge` 只做通用序列化代理，不理解任务业务。

### 任务模型与唯一键

- Group A 固定 15 架（A01..A15），A13..A15 为恢复/接替集（RESERVE）；默认 6 个整组批次：
  `prepare-a`、`coverage-segment-1`、`fault-exit`、`recovery-a`、`coverage-segment-2`、`hold-a-end`。
- 任务唯一键固定为：`(mission_id, command_id, uav_id)`。
- 完成策略 `ALL`：任一参与机拒绝/超时/链路 LOST 即整组失败并安全 HOLD 其他活动参与机。
- 单机任务全结构化字段（无 `payload_json`）；`accepted=true` 只表示校验、幂等登记并交给本机执行入口。

### 接收确认时序（TCP 5 秒约束）

GCS_A 必须：

- 对所有参与 UAV 并发调用，不得串行；
- 单次等待上限 `1.0 s`；最多调用 3 次（首次 + 2 次重试）；重试间隔 `0.2 s`；
- 最迟在 `4.0 s` 内形成整组 `ACCEPTED` 或 `FAILED`；
- 仅当全部参与 UAV 返回 `accepted=true` 后，向本地 Action 发布 `ACCEPTED` feedback。

### 单机状态三态

- 主状态：`ACCEPTED`、`COMPLETED`、`FAILED`；`COMPLETED`/`FAILED` 为终态。
- GCS_A 按任务唯一键与 `status_seq` 去重（重复与乱序丢弃）。
- 权威成功终态是 `COMPLETED`（不是 `SUCCEEDED`）。

### 身份映射

- 任务层 `A01..A15` → 执行层 `UAV1..UAV15` / hostname，存入版本化 `config/uav_identity_map.yaml`。
- 首版按已知规则 `A01 -> UAV1 -> 192.168.5.11` 依次递增；正式实机使用前需人工复核。

### 分层高度（动作层语义，不属于分发层）

- `MOVE_TO`：任务开始前 planner 已完成 5m TAKEOFF 并进入 HOLD；任务本身垂直到 15m、EGO 水平、垂直到目标 z；
- `FOLLOW_ROUTE`：leader 在 12m 执行 EGO，followers 在 12m 使用 PI+固定编队偏置；
- `FAULT_EXIT`：先垂直到 8m，再使用 EGO 水平离队。

### 安全基线（不可弃用）

- arm/OFFBOARD 软件所有权属于外部本机脚本 `offboard_takeoff_15.py`，不属于 executor 或 GCS START。
- 起飞顺序固定为：完成标定 → 脚本 HOLD（拒绝时回退 AUTO.LOITER）→ arm → OFFBOARD → planner TAKEOFF
  软起飞至 5m → HOLD → executor PREPARE 只检查 armed/OFFBOARD/origin → GCS START 启动已登记任务。
  START 不触发按需起飞，5m TAKEOFF 不属于 MOVE_TO 子阶段。
- GCS_A 在任务前用 A01-A06 全球位置中位数与圆周平均航向冻结标定结果，A01-A15 原点回读全部确认前拒绝后续流程；
  A01-A12 仍是 formation-1/prepare/coverage 成员集，不得替换为 A01-A06。
- `mission_anchor` 固定为 `(0,0,0)`，与当前配置及源码坐标变换一致。
- 统一球形距离门禁：planner hard-clearance 对预测轨迹同刻 3D 中心距 `<1.0m` 判失败；executor runtime gate
  对 fresh odom 的 3D 中心距 `<1.0m` 触发 HOLD；SafetyPredictor 采用更保守的 `<=1.0m` 边界。
- Safety 正式默认 `active`（2026-08-27）；2026-08-25 的 shadow/active 禁用结论保留为历史，但已被该状态取代。
- 软件 HOLD 不替代 PX4 datalink/offboard/RC failsafe；现场门禁完成前不得上桨。
- 无 RC 的 PX4 SITL 需设 `COM_RCL_EXCEPT=4` + `NAV_RCL_ACT=0`，并按 HOLD/AUTO.LOITER → arm → OFFBOARD 执行。

## 实施计划（现存）

| 文件 | 内容 | 状态 |
|---|---|---|
| `implementation_plan_26082300.md` | 方案 B：机载分层航点 + C++ `planning_horizon` 滚动局部规划 | 已实施（prepare 通过；避碰未验收） |
| `implementation_plan_26082322.md` | 周期性零速修复：PositionTarget 分源 + relay 唯一出口、仲裁与失效 HOLD | 已实施（2026-08-24） |
| `implementation_plan_26082415.md` | min-snap 统一单段重构：轨迹平滑 + 双机不安全时本机 HOLD 暂停 | **已实施（2026-08-24，代码+单测；单机验证进行中）** |
| `implementation_plan_26082500.md` | 预测式制动门禁 + tentative intent 屏障 + ID right-of-way + 四候选让路 | **已撤回**（L3 精简：yield/角色/协商已删除，监督只留 EMERGENCY 制动；见 `implementation_plan_26082602.md` 决策依据） |
| `implementation_plan_26082602.md` | 避碰门槛解耦（固定 clearance）+ 定时/周期碰撞检查 replan 机制 | **已实施（2026-08-26，代码+单测 232 tests 0 failure；双机 SITL 验收待执行）** |
| `implementation_plan_26082621.md` | plan-exec 循环重构（3m keypoints + 相邻目标）+ 单一 replan 入口 + EMERGENCY 走 replan（方案 Y） | **已实施（2026-08-26，代码+单测 257 tests 0 failure；SITL 双机验收进行中）** |
| `implementation_plan_26082722.md` | 轨迹末端 OOM 根因修复：`[0,T]` 有界采样、资源上限、single-QR、结果分流与内存回归 | **已实施（2026-08-28，代码+单测+PSI；全量 295 tests 0 failure；15-UAV prepare 验收进行中）** |

> 更早实施计划（26080804..26081406）与早期测试记录已归档删除，设计沉淀在本 README
> 规约与「当前状态」段。被审计冻结基准不回填修改；本 README「当前状态」段为唯一实时权威。

## 当前状态（实时）

- **已落地**：A01-A06 全员原点/航向计算门禁、A01-A15 原点回读确认、`mission_anchor=(0,0,0)`
  任务右-前-上变换、固定编队 1/2 精确集合、内部 PREPARE/START/ABORT、外部脚本 arm/OFFBOARD、
  planner TAKEOFF 5m 后 HOLD、15/12/8m 任务分层动作、followers PI、planner hard-clearance 与 executor
  runtime gate（统一 3D 中心距 1m 球形规范）。A01-A12 保持 formation-1/prepare/coverage 成员集。
- **坐标冻结**：A01-A06 连续采样 2s 后逐机求全球位置中位数和圆周平均航向，任一缺失、已 arm 或
  航向偏差超过 30° 均拒绝；冻结值在 GCS_A 进程内不变。15/15 原点回读达到水平/高度 0.1m 才就绪。
- **起飞/任务时序**：标定 → 外部脚本 HOLD（或 AUTO.LOITER）→ arm → OFFBOARD → planner TAKEOFF 软起飞
  5m → HOLD → executor PREPARE 只检查 armed/OFFBOARD/origin → GCS START。executor 不拥有 arm，
  START 不触发起飞，5m 不属于 MOVE_TO 子阶段。
- **链路健康（2026-08-12 晚修复）**：`link_health` + `safe_hold` 承担节点丢失 → 停止 → 全悬停 → 手动接管；
  `is_healthy_or_undiscovered` 不阻任务，heartbeat 阈值 suspect 10s / lost 15s，`ego_swarm.state_timeout_s=200`。
- **lease 已删除（2026-08-14）**：lease 时延敏感（renew 0.9s）且实机自组网必失败，按
  `implementation_plan_26081406.md` 删除，由 `link_health` + `safe_hold` 替代；`UavSafetyLease.srv` 保留为 dead definition。
- **方案 B（2026-08-23）**：`EgoSwarmDriver` 分层关键点 + `planning_horizon`(5m) 滚动窗口、
  滚动段继承 vel/acc、到达发布 COMPLETED、位置容差 0.6m；prepare 无头验收 SUCCESS（12/12 COMPLETED，
  速度峰值 1.05-1.12m/s）。
- **26082322（2026-08-24）**：EGO 发 P/V/A/yaw PositionTarget(mask 2048)、follower mask 2496、
  `setpoint_relay` 独占 `/mavros/setpoint_raw/local`（选源/generation/freshness/finite 校验、固定 HOLD）；
  `replan_period_s=1.0`。详见 `test_records/test_record_26082400.md`。
- **min-snap 统一单段重构（2026-08-24 已实施）**：上层 keypoints 最大 2m 密化（仅窗口推进/末点参考，
  不进 B-spline）；C++ `planOnce` 用 `one_segment_traj_gen` 单段（起点=旧轨迹 P/V/A、end_vel=切线巡航、
  最终段静止、梯形 T）→ 0.4m 弧长采样 point_set → `parameterizeToBspline` → L-BFGS（jerk+feasibility）；
  默认停用 `enable_rebound`/`enable_time_reparam`；执行层 EGO 巡航 `_distance_safe` runtime gate 使用
  fresh odom 的 3D 中心距 `<1.0m` 球形判定；
  `replan_period_s=1.0`、PositionTarget 契约不回归。多段 `minSnapTraj` 为备用方案（仅记录不实施）。
- **验证进展（min-snap 基线）**：当时全量 `catkin_test_results --all build` 166 tests 0 failure。多次 15 机无头 prepare
  因机间接近（如 UAV3-UAV5 0.85m）触发距离门禁 HOLD 而整组 FAILURE，符合"双机不安全即 HOLD"预期；
  单机 MOVE_TO 独立飞行验证进行中。
- **轨迹 intent 时间契约审计（2026-08-24）**：新增 `frame_id`；`stamp` 固定为候选实际激活 epoch，
  `t` 为从 0 开始、严格递增且覆盖精确 duration 的相对时间。EGO 节点在优化后按实际激活时刻重基准
  邻机预测，最多 3 次有限重试，并在提交锁内对最新 intent 做最终 hard-clearance；非法/过期/未来、
  乱序和非有限 intent 均拒绝。rebound 输出强制满足动力学再按新时间轴复验 clearance。
- **主动互惠边界（2026-08-24）**：理想匀速正交交叉已能主动绕行并满足 1m、速度 ≤1.5m/s、
  加速度 ≤`max_acc*1.05`；但真实 peer min-snap nominal intent 的单边让路仍可能 `planOnce=false`，
  多轮互惠收敛未通过。`enable_rebound` 因此继续默认 false，生产兜底仍是执行层距离门禁 HOLD，
  禁止仅恢复开关作为主动避碰完成证据。最终全量结果为 186 tests、0 failure（含生产 EGO launch
  展开后的 `enable_rebound=false` runtime rostest，以及 TCP listener 早绑定/端口策略回归）。
- **主动避碰增强阶段 A（2026-08-25 已完成，历史状态）**：新增纯 C++ `SafetyPredictor`、数字 ID role、
  episode 迟滞和有界异步 `SafetyLogger`；当时生产 launch 以 25Hz shadow 计算同轴 CPA/TTC、数据年龄和
  两级动态门槛，不改状态/轨迹/intent/setpoint。`ego_planner_driver` 96 tests 0 failure。后续依次实现
  受约束紧急制动、最多 1s tentative intent 协商窗口、
  数字 ID right-of-way 和左/右/减速/加速四候选。缺失邻机默认不阻启动，旧 1m 门禁仅在全部延迟/
  惯性 A/B 验收通过后才可独立删除。
- **主动避碰增强阶段 B（2026-08-25 自动测试完成）**：新增从执行轨迹同刻 P/V/A 连续衔接的
  受约束停车轨迹，active emergency 可抢占 nominal；safety generation 阻止晚到规划覆盖制动；停车后
  锁存真实终点 `BRAKE_HOLD`。当时生产仍默认 shadow，SITL/延迟闭环前不声明飞行验收；该默认状态已于
  2026-08-27 被正式 active 状态取代。
- **主动避碰增强阶段 C/D（2026-08-25 自动测试完成）**：intent 协议 2.0 增加
  tentative/active/braking phase（ROS MD5 变更，须同批重建全消费者）；≤1s 协商窗口、缺失邻机
  不阻塞；数字 ID right-of-way + LEFT/RIGHT/SLOW/FAST 四候选与确定性评分。全工作区 229 tests
  0 failure。当时生产 launch 默认 shadow；该默认状态已于 2026-08-27 被正式 active 状态取代。
- **⚠️ 2026-08-25 审查后冻结的历史基线（已被 2026-08-27 active 状态取代）**：
  以下内容保留用于解释当时决策，不再代表当前正式默认状态：
  - ✅ **当时可用**：通信链路（bridge/Service/TCP 协议未动）、min-snap 轨迹执行/分层/起飞、
    shadow 预测（只写日志不改行为）。
  - 🚫 **当时冻结**：`safety_supervisor_mode=active`（制动/让路）存在已确认逻辑缺陷
    （见 `implementation_plan_26082500.md` §7.5 缺陷清单 1–7），当时 **SITL 禁止用 active 跑**、
    生产保持 shadow。此禁用结论已由 2026-08-27 正式默认 active 状态取代；缺陷 1
    （`EgoSwarmDriver.hold()` 忽略 deadline 阻塞 200s）已修复，缺陷 2–7 为当时待修清单。
  - ⚠️ **假配置**：`neighbor_stale_policy`/`neighbor_intent_stale_s`/`neighbor_missing_policy`
    仅存在于 launch，生产 C++ 未接入决策分支，勿据此判断系统行为。
  - 阶段 C tentative 协商屏障只在测试路径 `replanOnce()` 生效，**生产 `replanLoop()` 未接线**
    （直接提交 ACTIVE）；当前生产无 1s 协商延迟。接线与否属阶段 E 决策项。
  - **参数改名（2026-08-25）**：`ego_hold_timeout_s` 为 ego HOLD/收口超时（默认 2s，
    HOLD 确认/任务失败回退/shutdown join）；`uav_task_timeout_s` 为任务执行超时（默认 200s）；
    driver 内部 `state_timeout_s` 改名 `task_timeout_s`。历史记录（如 `26082420`）中的
    `ego_state_timeout_s`/`state_timeout_s` 指旧语义的任务执行超时，阅读时注意区分。
    同日删除 ego launch 中已无代码读取的过期参数 `terminal_to_idle_s`/`idle_hover_refresh_hz`
    （IDLE 概念已随初始态改为 HOLD 而废弃）。
- **L3 精简（2026-08-25/26）**：删除 yield 候选（SLOW/LEFT/RIGHT/FAST）、SafetyPredictor 角色判定
  （YIELD/STAND_ON）、intent 协商（TENTATIVE/NEGOTIATING）；监督层只保留 EMERGENCY 制动兜底；
  动态距离公式合并为 `dynamicMargin` 单一实现；源码 4646→4083 行、node.cpp 1660→1426 行。
  相应 launch 已清残留 yield/negotiation 参数。
- **GPS 时钟偏置（2026-08-25）**：新增 `gps_bias_node`（订阅 `/mavros/time_reference`，10s 滑动均值
  bias，1Hz 发布）+ 拓扑 bridge 转发；`ego_planner_driver` 接收端用
  `T_own = T_other - bias_other + bias_own` 修正邻机 intent 激活时刻（`bias_valid_s=120s`，过期回退
  相对时钟）。SITL 实测 `source=fcu` 有数据（bias≈90µs，共享时钟修正量≈0；实机 GPS/RTK 生效）。
  时间戳修正为"固定 clearance 解耦"提供预测能力前提（见 implementation_plan_26082602）。
- **SITL 复测复盘（2026-08-26，26082602 后七轮）**：双机正交多次复测暴露 ① 多路径 replan 提交竞争
  （replanLoop/collisionCheckTick/runSafetySupervisor 三处改轨迹）、② 候选让路破坏监督预测窗
  （SLOW 时间延长 → INSUFFICIENT_DATA）、③ 候选方向/长度/速度约束缺陷。决策（`implementation_plan_26082621.md`）：
  plan-exec 循环对齐原版（3m keypoints + 相邻目标推进）、单一 replan 入口（所有触发只置位）、
  EMERGENCY 制动走 replan 线程（方案 Y）、候选保留为 replan 失败分支、删 POSE_STALE。
- **plan-exec/单一入口/方案 Y（2026-08-26 实施，26082621）**：执行循环三分支（推进 0.5m / 到达
  0.2m（复盘放宽，PX4 SITL 稳态偏差 0.13-0.17m，0.1m 太紧）/ 定时 1.0s）全部只置位唤醒
  replanLoop（条件变量，唯一提交者）；EMERGENCY 监督回调只置 `emergency_pending_`，制动由
  replan 线程在"常规 replan 失败且 ttc <1.0s"时提交（EMERGENCY_BRAKE → BRAKE_HOLD），否则收口
  PLAN_FAILED；删 POSE_STALE（HOLD 锁存位置）；`waypoint_densify_spacing=3.0`。
- **候选迭代（26082621 追加，A/B）**：STAND_ON 只生成 SLOW 变速候选（`slow_only`，非对称时序
  等待/加速）；常规规划成功但 min_clear <1.2m 也生成候选替换（裕度触发）；SLOW {0.5,0.8,1.2}
  含加速、LEFT/RIGHT {1.3,1.8,2.2}×{0.7,0.5}；验证门槛 1.3→1.2；cost=ΔT/T（绝对值）+dev/2.0。
  全量 263 tests 0 failure。SITL 双机正交复测：迭代 A/B + 速度判定放宽 1.5→2.5
  （verification `--max-speed`，加速档 1.2 + PX4 超调实测 1.49-1.54）后 **8/9 PASS**
  （仅 1 次 UAV1 规划失败 HOLD）；双方 COMPLETED 8/9（原始 3/5），min_3d 全程 ≥1.0m
  （1.15-1.62m），0.605m 距离门禁事件消除。
- **Safety 正式状态（2026-08-27）**：正式默认 `safety_supervisor_mode=active`。planner hard-clearance
  以预测轨迹同一时刻的 3D 中心距 `<1.0m` 为硬门，executor runtime gate 以 fresh odom 的 3D 中心距
  `<1.0m` 为门禁，均为球形规范；SafetyPredictor 使用 `<=1.0m`，在恰好 1m 的边界上更保守。
  2026-08-25 的生产 shadow 与 active 禁用说明仅保留为历史，已被本状态取代。
- **prepare 15 机全 active + 软着陆修复（2026-08-27，26082700）**：`safety_supervisor_mode=active`
  下 15 机 prepare 批次 →
  A02/A05 `EGO_PLAN_FAILED`（非距离门禁，互惠避碰约束下规划无解）→ 整组安全收口。修复：
  ① replan 失败软着陆（保持当前轨迹执行，1s 定时/冲突/推进事件继续重试，连续失败 ≥
  `plan_fail_timeout_s`(10s) 才 EGO_PLAN_FAILED）；② backend `_safe_hold` 分批 4 机 + 0.5s 重试
  2 次；③ 候选 cost `dev/2.0→dev/5.0`；④ `hovering_velocity_mps` 0.1→0.2。全量 266 tests 0 failure
  （26082700 记录为 265，系未含后续回归；当前实测 266）。
- **轨迹末端 OOM 修复（2026-08-28，已实施 26082722 + 260828 复审修复）**：根因固定 `[0,1]` 外推采样
  + 0.4m 无界 point set + 三次 dense QR。修复：`samplePolynomialByArcLength` 只按真实物理时长
  `[0,T]` 有界细采样（`N=ceil(T/0.02)+1` 上限 `max_arc_samples=4096`），弧长重采样点数在分配前
  checked arithmetic 并限制 `max_parameterization_points=256`；`parameterizeToBspline` 改为 bounded
  single-QR（一次分解三列 RHS、rank/residual/finite 校验、失败清空输出）；core 返回四类 `PlanResult`
  （SUCCESS/DEGENERATE_SEGMENT/RETRYABLE_FAILURE/RESOURCE_LIMIT）；node 对 DEGENERATE 等待真实
  odometry、RETRYABLE 软着陆、RESOURCE 立即 `EGO_PLAN_FAILED→HOLD`；yield 候选传播资源失败；replan
  线程最外层 `bad_alloc` containment；intent/轨迹/采样全部硬上限。timed trajectory nominal step 保持
  0.1s、candidate/relay 30Hz、candidate timeout 0.2s 未动（冻结契约无回归）。**260828 复审修复**：
  ① 退化段状态机（snapshot 前 fresh-odom 推进/最终到达；无 current_ 真实到达 → COMPLETED；DEGENERATE
  已失败计时 → 软着陆 10s 收口；退化等待禁 HOLD 捕获锁存）；② 生产开关全关下采样后欧氏峰值
  >2.5×max_vel/max_acc → RETRYABLE_FAILURE（不发布 intent）；③ 资源分类方式 B（独立紧 cap 测试只接受
  RESOURCE_LIMIT + 空输出、普通矩阵 resource==0、node 层零 retry/零 yield/无部分 intent）；④ 两个
  setrlimit 返回检查。测试（260828 重跑，完整 suite filter）：受限短末段 7 tests、ego_planner_driver
  **172 tests 0 failure**、全工作区 **303 tests 0 errors 0 failures**、500/2000 次压力（完整
  `ShortFinalSegment.*` 前缀）均过 RSS 门禁。**计划级 15-UAV active prepare 验收状态（260828 修正，
  §4.1）**：实现已提交；memory-specific observations PASS（第 3 轮重测 15 机：planner RSS
  11.76-12.41 MB、OFFBOARD 15/15 保持、relay 直接实测 30.000 Hz / max 间隔 0.035 s、planner
  candidate 30.000 Hz、host available 19.5 GiB、无 OOM/−6/−9）；prepare 批次第 3 轮返回
  **FAILURE(A03/HOLD)**（11/12 COMPLETED，A03 终点 0.29m 偏差未满足 0.2m 到达判据 → checkArrival
  看门狗转 HOLD → executor COMMAND_HELD；前两轮 FAILURE(A02/A05 EGO_PLAN_FAILED)）⇒ 按冻结计划 §15
  整轮判 **计划级 acceptance FAIL**，需修复当前任务缺陷后重跑整轮 15-UAV acceptance（A03 根因是否
  属 OOM 范围不改变判 FAIL 事实）。
- **到达阈值放宽 0.2m→0.5m（2026-08-28 用户决策）**：`arrival_reach_thresh_m` 由 0.2m 放宽为
  **0.5m**（`checkArrival` 分支 B：距最终关键点 <0.5m 且稳定 `arrival_stable_s`=1.0s → COMPLETED）。
  依据：第 3 轮 prepare 实测 UAV3 终点偏差 0.29m 停在 0.2m 判据外 → 看门狗 HOLD → COMMAND_HELD；
  0.5m 覆盖该偏差。同步修改 `ego_planner_driver`（node.h/.cpp 默认与注释、`ego_planner_driver.launch`、
  README、`test_ego_state_machine.cpp` 注释 + `DegenerateWhileFailingClosesAfterTimeout` pose
  4.7→4.3 保持"距目标 0.7m>0.5m 不到达"语义）与 `swarm_uav_executor`（`uav_executor_ego.launch` 透传、
  `test_ego_launch_config.py` 断言）。自动测试全部通过：相关包 -j1 -p1（ego 172 + executor 94）、
  受限短末段 7 tests、全工作区 **303 tests 0 errors 0 failures**。冻结契约（30Hz/0.2s timeout、
  PositionTarget、relay、Safety、1m 距离门禁）未回归。计划级 15-UAV acceptance 待重跑整轮确认。
- **换位/三机/编队未通过（2026-08-27）**：双机正交 8/9 PASS，但双机交换（`reciprocal_15sitl.py --swap`）
  与三机交叉（`triple_cross_15sitl.py`）未通过（三机最小 3D 0.563m 破距、UAV2 HOLD_FAILED、
  UAV3 MIN_DISTANCE_BREACH、UAV3 横向摆动 5.1m）；prepare 批次 EGO_PLAN_FAILED。机载避碰对
  "走廊式/多机并发"冲突仍不足——改进方向见 `ego_planner_driver/README.md` 与
  `.tmp/notes/UAV_swarm_waypoint_execution_collision_avoidance_research.md`。
- **未完成**：15 机 PX4 SITL 六批次端到端重跑；双机交换/三机交叉/parepare 通过；Phase 6 实机不上桨及上桨联调。
- **已知问题（测试流程）**：`gcs_a_backend.launch` 需 `catkin build tcp_to_ros` 后
  `roslaunch tcp_to_ros` 才能找到；跨 Master 建链初期多机 heartbeat 抖动易触发 fail-fast；
  UAV 逐机串行启动耗时较长，一机 roslaunch 偶发崩溃需逐机独立启动并补充轮。

## 文档索引

| 路径 | 内容 |
|---|---|
| `plans/README.md`（本文件） | Phase 编号、设计规约、计划索引、当前状态 |
| `plans/implementation_plan_26082500.md` | 预测式制动安全门禁、首次 intent 协商与确定性主动让路（已撤回，L3 精简） |
| `plans/implementation_plan_26082602.md` | 避碰门槛解耦（固定 clearance）+ 定时/周期碰撞检查 replan（已实施） |
| `plans/implementation_plan_26082621.md` | plan-exec 循环重构 + 单一 replan 入口 + EMERGENCY 走 replan（方案 Y，已实施） |
| `plans/implementation_plan_26082722.md` | 轨迹末端 OOM 最终修复：真实 `[0,T]` 有界采样、资源边界、single-QR 与回归验收（已实施 2026-08-28） |
| `plans/implementation_plan_26082415.md` | min-snap 统一单段重构（历史） |
| `plans/implementation_plan_26082300.md` / `26082322.md` | 方案 B / PositionTarget 契约基线 |
| `test_records/test_record_26082300.md` / `26082400.md` / `26082421.md` | 方案 B prepare / PVA 链路 / intent 时间契约审计记录 |
| `test_records/test_record_26082501.md` | 预测避碰阶段 A 实施与自动测试记录 |
| `test_records/test_record_26082602.md` / `26082621.md` / `26082700.md` | 26082602 / 26082621 / prepare 15 机全 active 与软着陆修复记录 |
| `test_records/test_record_26082722.md` | 轨迹末端 OOM 修复：有界采样、single-QR、PlanResult 分流与内存门禁（本记录） |
| `CONTEXT.md` | 领域词汇表 |
| `../README.md` | 项目主 README（操作手册） |
