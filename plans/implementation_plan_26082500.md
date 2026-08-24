# 预测式制动门禁、任务级 intent 屏障与确定性让路实施计划

日期：2026-08-25 00:57

状态：**待实施**

## 1. 目标与成功判据

本计划在现有 min-snap 单段滚动规划、PositionTarget P/V/A 控制链路和 intent 时间契约之上，
分阶段补齐主动避碰闭环。目标不是仅在当前中心距小于 1m 后 HOLD，而是在 20–30Hz 持续预测、
通信延迟和飞行器制动惯性存在时，仍使正常实验场景的双机/三机实际中心距保持 **不小于 1.0m**。

最终成功判据：

- C++ `ego_planner_driver` 具有独立于 1s 常规重规划周期的预测式安全监督，运行频率 20–30Hz；
- 安全监督使用同一时间轴上的本机执行轨迹、邻机 intent、当前 P/V/A、相对速度、CPA/TTC 和数据年龄；
- 两级动作有效：预警级请求确定性让路/重规划，紧急级切换到从当前 P/V/A 生成的受约束制动轨迹；
- 首次 nominal 轨迹先发布为 tentative intent，最多等待 1.0s 收集已到达的邻机 intent，再决定激活；
- 不把“预期活动邻机全部出现”作为启动必要条件，1.0s 到期后允许按当前可用信息继续；
- 冲突时固定 right-of-way：数字 ID 小者保持 nominal，数字 ID 大者选择让路候选；
- 让路候选固定为水平左绕、水平右绕、减速、加速四类，速度范围 0–1.5m/s，全部通过全时域
  clearance、速度、加速度和连续性检查后才能激活；
- 双节点闭环仿真、双机 SITL 同层正交、双机 SITL 换位、三机 SITL 交叉均满足中心距 ≥1.0m，
  且没有双方反复互相重规划的振荡；
- 12机 prepare 最后使用完整 TCP→GCS→UAV 通信链路回归；新增避碰专项测试不引入该完整链路；
- 每机每次运行覆盖写入独立诊断文件，仿真从输入身份确定 UAVn，实机可从 hostname 回退确定身份。

## 2. 已确认基线与问题

### 2.1 保留并复用的基线

- `ego_planner_driver` 已以 30Hz 发布同一轨迹时刻的 PositionTarget P/V/A/yaw；
- `setpoint_relay` 是 `/mavros/setpoint_raw/local` 唯一出口，EGO/HOLD mask 契约不变；
- `replan_period_s=1.0` 是常规滚动重规划周期，不能代替高频安全监督；
- intent 已有 `frame_id`、激活 epoch、相对时间轴、版本去重、有限性和过期检查；
- 候选轨迹提交前已有 activation-time 重基准和 hard-clearance 复验；
- `enable_rebound=false`、`enable_time_reparam=false` 保持默认值；本计划不以开启 rebound 作为避碰方案；
- 执行层已有水平 <1.0m 且垂直 <2.0m 时本机 HOLD 的事后兜底。

### 2.2 当前缺口

1. 首次规划时 inbox 可为空，节点可立即激活并发布一条与邻机冲突的 nominal 轨迹；
2. 新 intent 只有在已执行轨迹被判冲突时才设置 `planning_requested_`，没有独立 20–30Hz 未来预测；
3. 当前 hard-clearance 是轨迹接受/拒绝条件，执行层 1m 门禁是事后 HOLD，均未计入 closing speed、
   200–800ms 双向通信延迟和制动距离；
4. HOLD 锁存当前位置，是固定位置目标，不是从当前 P/V/A 连续衔接、满足动力学限制的制动轨迹；
5. 生产代码没有按 UAV ID 分配让路责任，冲突双方仍可能同时请求重规划；
6. 当前优化器不能稳定从真实 peer min-snap intent 中单边找出可行绕行，不能依赖一次 L-BFGS；
7. 缺少能复现实机网络年龄、乱序、丢失、执行延迟和滑行距离的双节点闭环测试；
8. 现有 ROS 日志不足以还原每轮输入、预测阈值、right-of-way 和候选拒绝原因。

## 3. 范围与明确不做事项

### 3.1 本计划范围

- C++ 预测式安全监督和两级动态门槛；
- 受约束 emergency braking trajectory；
- 最多 1.0s 的任务级首次 intent 协商窗口；
- 数字 UAV ID 的确定性 right-of-way；
- 左绕、右绕、减速、加速四类有限候选及确定性选择；
- 可关闭、可调阈值的 stale/missing 处理和完整诊断；
- 避碰专项双节点仿真与双/三机 SITL，最后完整链路 12机 prepare 回归。

### 3.2 不做事项

- 不修改统一坐标和时间同步流程，不增加原点重启/异常重启恢复逻辑；
- 不处理初始水平间距小于 3m 的实验外启动条件；测试预检发现该条件时直接拒绝启动场景；
- 不修改任务规划层航路来预先消除冲突；
- 不恢复通信双向看门狗，不因预期邻机 intent 缺失直接判任务失败或永久 HOLD；
- 不增加 leader priority；UAV1 已固定为 leader，right-of-way 仅按数字 ID；
- 不开启全机 rebound，不以提高 `lambda_swarm` 或固定放大 clearance 代替预测闭环；
- 不在专项避碰测试中引入 TCP→GCS→UAV 全链路；该链路仅用于最终 12机 prepare 回归。

## 4. 冻结安全语义

### 4.1 right-of-way 与紧急制动的优先级

对预测到冲突的一对 UAV：

- 从 `uav_id`（优先）或 `exec_target` 提取末尾数字，数值小者为 **STAND_ON**，保持 nominal；
- 数值大者为 **YIELD**，从四类候选中选取安全轨迹；
- ID 无法解析、重复或身份不一致时，不猜测优先级，双方均可进入预警 HOLD/紧急制动并记录错误；
- right-of-way 只决定常规避让责任。**任何 UAV（包括低 ID）一旦进入紧急级，都必须执行制动**，
  不能为“保持 nominal”而穿越紧急安全边界；
- 一次冲突 episode 内锁存 peer、角色和已选左右方向，直到预测安全持续一段迟滞时间后解除，防止振荡；
- 多邻机同时冲突时，以最早预计越界/最小预测距离者为主冲突；候选仍必须对全部 fresh 邻机检查。

### 4.2 任务级首次 intent 屏障

“等待1s”定义为**有上限的协商窗口**，不是全员 barrier：

1. 新 goal/waypoints 到达后生成 nominal 候选，但不立即写入 `current_`，状态进入 `NEGOTIATING`；
2. 立即发布 tentative intent，使异步启动的邻机能够看到本机计划；
3. 在最多 `intent_negotiation_wait_s=1.0` 内，每次收到新的有效 intent 都重新评估 nominal/让路候选；
4. 当前收到的邻机 intent 均不冲突时，可提前激活，不强制等待满 1s；
5. 1s 到期后，即使预期活动邻机缺失，也按当前可用信息激活，不因完整性不足阻塞任务启动；
6. 若已收到的 intent 与本机冲突且尚无安全解，则保持 braking/固定 HOLD，不提交冲突执行轨迹；
7. tentative intent 不得被 setpoint 执行；消息需可区分 tentative/active。优先给
   `UavTrajectoryIntent` 增加枚举/字段并升级协议；若为兼容旧桥接暂不改消息，则必须使用独立 tentative topic，
   禁止仅靠日志文本区分。

协议升级必须保持 topology bridge 通用转发语义，不把任务业务塞入 bridge。旧版本 intent 的兼容策略在
单测中冻结：只能按 active 旧语义处理，不能误认成已协商 tentative。

### 4.3 两级动态安全门槛

每个 safety tick 对本机和每条 fresh 邻机轨迹在共同有效时间窗内同步采样，并计算：

- 当前相对位置 `r = p_peer - p_self` 和相对速度 `v_rel = v_peer - v_self`；
- closing speed `v_close = max(0, -r·v_rel/|r|)`；
- CPA 时间 `t_cpa = clamp(-r·v_rel/|v_rel|², 0, horizon)`、CPA 距离和首次预计越过 1m 的 TTC；
- 本机/邻机 intent 接收年龄、轨迹激活年龄和共同预测窗长度；
- 以 `max_acc`、配置的保守制动减速度、控制延迟和数据年龄估算制动余量。

动态门槛不冻结为一个未经标定的常数，先采用可审计公式并通过测试标定：

```text
reaction_s = control_delay_s + local_compute_budget_s + peer_data_age_s
closing_margin_m = v_close * reaction_s
braking_margin_m = v_close² / (2 * emergency_decel_mps2)
warning_distance_m = 1.0 + closing_margin_m + braking_margin_m + warning_buffer_m
emergency_distance_m = 1.0 + closing_margin_m + braking_margin_m + emergency_buffer_m
```

约束：

- `warning_buffer_m > emergency_buffer_m >= 0`，预警级应早于紧急级；
- 预警级：预测将越过 warning 边界或 TTC 小于阈值，STAND_ON 保持 nominal，YIELD 生成候选；
- 紧急级：预测将越过 emergency 边界、TTC 过短、预警重规划失败或安全信息不足以覆盖制动窗时，
  立即切 emergency braking trajectory；
- 判定加入进入/退出迟滞和连续 tick 计数，但紧急进入不得因滤波延迟错过制动点；
- 先保留当前 1m 执行层门禁作为最后兜底。只有两级门禁在全部延迟/惯性测试中有效，且关闭旧门禁的
  A/B 测试仍满足 ≥1m，才允许后续删除旧门禁；删除必须单独提交和单独验收。

## 5. 轨迹设计

### 5.1 Emergency braking trajectory

紧急制动不能调用当前“锁存当前位置 HOLD”代替。新增纯数值制动轨迹生成器：

- 起点从安全 tick 的本机执行轨迹同一时刻采样 P/V/A；轨迹不可用时才使用 fresh odometry P/V，
  acceleration 置零并记录降级来源；
- 沿当前速度方向以 jerk/acceleration 受限方式减速到零，位置由积分得到，不瞬移到当前 pose；
- 初版优先实现可验证的一维速度方向 S-curve/分段多项式，三轴同步缩放；最大速度不超过 1.5m/s，
  最大加速度不超过现有 `max_acc`，新增 `emergency_decel_mps2` 与 `emergency_jerk_mps3` 参数；
- 生成后执行 finite、P/V/A 连续、动力学和对所有 fresh intent 的全时域 clearance 检查；
- emergency 已经触发时，即使不存在完全满足 clearance 的制动轨迹，也执行“最小停车距离”的可行制动，
  同时发布高优先级诊断；不能退回继续执行 nominal；
- 制动完成后锁存制动终点为 HOLD，不自动恢复任务；由上层任务失败/HOLD 流程收口；
- emergency trajectory 作为 active intent 发布，使邻机预测到真实制动过程，而不是固定点假设。

### 5.2 四类确定性让路候选

仅 YIELD 机生成以下候选，顺序和评分必须确定：

1. **LEFT**：在水平面沿 nominal 切线左法向增加横向中间点；
2. **RIGHT**：沿右法向增加横向中间点；
3. **SLOW**：保持几何路径，通过降低滚动段末速度/延长候选时间避开占用时窗，最低允许 0m/s；
4. **FAST**：保持几何路径，在足够安全距离内提前通过冲突区，最高允许 1.5m/s。

候选规则：

- 左/右偏移量、构造前视距离、速度档位均参数化，但第一版使用有限离散档位，不做连续搜索；
- 从当前 P/V/A 连接候选，不允许候选切换产生 P/V/A 阶跃；
- 速度范围 `[0, 1.5]m/s`，加速度不超过 `max_acc*1.05`，并检查 jerk/finite；
- 对共同时间窗全部采样点做 1m clearance；预测窗短于本机停车时间时，候选视为证据不足；
- 评分按“安全可行优先 → 最大最小 clearance → 最小到达时间增量 → 最小横向偏移 → 固定枚举顺序”排序；
- 一旦 LEFT/RIGHT 被选中，在 episode 内保持该方向；仅当其变为不可行时才按固定顺序切换；
- FAST 只有在从当前到冲突点全程具有足够加速/制动余量时可选，不能为抢行突破动态 emergency 边界；
- 四类均失败时进入 emergency braking，不回退到一次 L-BFGS 自行找路；
- `enable_rebound` 继续为 false，四候选由独立 candidate generator 实现，不复用 rebound 开关。

## 6. stale/missing 语义与诊断日志

### 6.1 可关闭的 stale 策略

考虑自组网 200–800ms 双向 ping 时长，不恢复短周期通信看门狗。新增独立参数：

- `neighbor_stale_policy`: `diagnose_only`（默认）/`brake_if_closing`/`brake_always`/`disabled`；
- `neighbor_intent_stale_s`：初始建议 2.0s，必须通过网络回放标定，不沿用 relay 的 0.2s timeout；
- `neighbor_missing_policy`: 默认 `continue_after_barrier`，不因预期邻机从未出现阻止启动；
- `intent_negotiation_wait_s`: 默认 1.0s，可关闭为 0；
- `peer_data_age_cap_s`: 动态门槛计算时的数据年龄上限；超过后按 policy 处理，避免无界放大门槛。

默认 `diagnose_only` 下 stale/missing 只记录、计入预测不确定性，不直接导致任务失败。只有完成延迟注入
A/B 后，才允许把 `brake_if_closing` 设为实验默认；`brake_always` 不作为自组网默认配置。

### 6.2 每机覆盖日志

在当前工作空间内新增运行日志目录，例如：

```text
/home/ub20tg/catkin_swarm6-2/runtime_logs/ego_planner/
  UAV1-ego-planner.log
  UAV2-ego-planner.log
```

实现约束：

- 默认目录必须可由 launch 参数 `diagnostic_log_dir` 覆盖；目录不存在时创建，创建失败则保留 ROS 日志并告警；
- 每次节点启动以 truncate 模式覆盖本机旧文件，不追加跨运行记录；
- 仿真优先从 `exec_target=UAVn` 输入生成文件名；实机未显式提供时可从 hostname 解析 `UAVn`；
- 文件名只允许 `[A-Za-z0-9_-]`，解析失败使用 `UNKNOWN-ego-planner.log` 并在启动时告警；
- 使用单写线程/有界队列或低频批量 flush，禁止 safety timer 同步磁盘 IO 阻塞控制回调；
- 日志至少包括 monotonic/ROS 时间、节点身份、状态、轨迹版本、peer 版本和年龄、P/V/A、相对状态、
  CPA/TTC、两级动态门槛、角色、episode、候选列表及每个拒绝原因、最终动作和状态转换；
- 不新造专有二进制格式；首版使用逐行文本或 JSON Lines，便于 grep 和离线脚本解析；
- 进程正常退出时 drain 队列，异常退出允许丢失末尾少量诊断，但不能影响制动动作。

## 7. 分阶段实施

### 阶段 A：预测数学、身份和日志基础（不改变飞行行为）

修改 `src/ego_planner_driver`：

- 新增纯 C++ `SafetyPredictor`：同步采样、相对状态、CPA/TTC、动态门槛和判级；
- 新增严格数字 ID 解析与 pair role 判定 helper；
- 新增有界异步 `SafetyLogger`，按第 6.2 节输出每机覆盖日志；
- node 增加 20–30Hz safety timer，但先以 `safety_supervisor_mode=shadow` 运行，只记录不改状态；
- 统一 `replanOnce()` 与 `replanLoop()` 中重复的候选生成/提交代码，抽取单一事务函数后再接安全状态，
  避免两条路径实现不同的门禁；
- launch 增加 supervisor、预测窗、延迟、buffer、日志和 stale policy 参数，默认 shadow；
- `test_trajectory.cpp` 覆盖 CPA/TTC、同时间轴采样、数据年龄和动态门槛；
- `test_ego_state_machine.cpp` 覆盖 ID 解析、STAND_ON/YIELD、episode 迟滞和 shadow 不改变输出。

阶段门槛：现有 78 个 `ego_planner_driver` 测试全部回归，新测试通过；单机运行的 setpoint、intent 和状态
与基线一致；日志不造成 safety tick 超期。

### 阶段 B：受约束紧急制动闭环

- 新增 `EmergencyBrakeGenerator`，从同一时刻 P/V/A 生成受约束停车轨迹；
- node 增加 `WARNING`、`EMERGENCY_BRAKE`、`BRAKE_HOLD` 内部状态；对外 `exec_state` 如需新增字符串，
  同步更新 executor 映射，最终任务仍以失败/HOLD 收口；
- safety timer 在 active 模式下可原子替换 `current_`、`plan_start_` 并发布 braking active intent；
- 轨迹替换与常规 replan commit 使用同一 mutex/版本规则，紧急动作优先，旧规划结果不得晚到覆盖制动轨迹；
- 制动结束锁存实际制动终点，不调用“触发瞬间当前位置”的旧 HOLD 作为主动作；
- 保留旧 1m 门禁，先验证动态门禁会更早动作，旧门禁只作未触发兜底。

阶段门槛：数值单测覆盖初速度 0、0.2、1.0、1.5m/s，不同方向和非零加速度；闭环模型加入
200–800ms intent 延迟、控制延迟和位置跟踪惯性后，紧急制动轨迹连续且不超动力学限制。

### 阶段 C：tentative intent 与最多 1s 协商窗口

- 扩展 `swarm_uav_interfaces/UavTrajectoryIntent.msg` 表达 tentative/active/braking phase；
- 同步修改消息生成、桥接序列化测试、拓扑 smoke 和旧版本兼容测试；
- 新 goal 后 nominal 先进入 `NEGOTIATING`，发布 tentative，不发布轨迹 PositionTarget；relay 继续输出既有 HOLD；
- 新 intent 到达触发协商重算；安全可提前激活，1s 到期且无已知冲突时激活；
- 缺失邻机不阻塞；已知冲突无解则制动/HOLD；
- 激活时仍执行现有 activation-time 重基准和 commit-lock hard-clearance，不能因协商屏障删除最终复验。

阶段门槛：空 inbox 同时启动、100/400/800/1200ms 延迟、乱序和一侧丢失均无冲突 nominal 先执行；
缺失邻机在 1s 后能启动，已收到冲突 intent 时不会提交不安全 active 轨迹。

### 阶段 D：确定性 right-of-way 与四候选

- 新增独立 `YieldCandidateGenerator`，构造 LEFT/RIGHT/SLOW/FAST；
- STAND_ON 在 warning 级保持 nominal 并持续发布 active intent；YIELD 才生成让路候选；
- 候选使用统一 min-snap/B-spline 参数化和 P/V/A 起点约束，但不经过 rebound；
- 所有候选通过统一全时域 clearance 和动力学 validator；
- 锁存 episode 角色和左右方向，记录候选评分及拒绝原因；
- 多 peer 冲突按最危险 peer 选角色，但最终检查覆盖全部 fresh peers；
- 四候选失败或 emergency 级触发时切制动轨迹。

阶段门槛：真实 peer min-snap intent 下单边让路可行；同一冲突 episode 内低 ID 不反复重规划，
高 ID 不左右抖动；速度 ≤1.5m/s、加速度 ≤`max_acc*1.05`。

### 阶段 E：策略标定、旧 1m 门禁 A/B 与文档收口

- 用 200–800ms 网络时延分布、控制延迟和 SITL 跟踪误差标定 reaction/deceleration/buffer；
- stale policy 保持默认 `diagnose_only`，完成回放后再决定是否把 `brake_if_closing` 作为实验配置；
- 开启旧 1m 门禁跑完整验收，证明其只作最终兜底；
- 仅在所有验收通过后增加“关闭旧门禁”的显式 A/B 参数并重跑全部专项场景；
- 若关闭后仍满足 ≥1m，可在独立变更中删除旧门禁；否则保留，不以计划目标强行删除；
- 更新 `ego_planner_driver`、`swarm_uav_executor` README、launch 注释、操作手册和测试记录。

## 8. 文件级改动预期

实施时以现有命名约定为准，预期涉及：

- `src/ego_planner_driver/include/ego_planner_driver/ego_planner_driver_node.h`
- `src/ego_planner_driver/src/ego_planner_driver_node.cpp`
- `src/ego_planner_driver/include/ego_planner_driver/safety_predictor.h`（新增）
- `src/ego_planner_driver/src/safety_predictor.cpp`（新增）
- `src/ego_planner_driver/include/ego_planner_driver/emergency_brake.h`（新增）
- `src/ego_planner_driver/src/emergency_brake.cpp`（新增）
- `src/ego_planner_driver/include/ego_planner_driver/yield_candidates.h`（新增）
- `src/ego_planner_driver/src/yield_candidates.cpp`（新增）
- `src/ego_planner_driver/include/ego_planner_driver/safety_logger.h`（新增）
- `src/ego_planner_driver/src/safety_logger.cpp`（新增）
- `src/ego_planner_driver/launch/ego_planner_driver.launch`
- `src/ego_planner_driver/CMakeLists.txt`、`package.xml`（只按实际新增 ROS/测试依赖调整）
- `src/ego_planner_driver/test/test_trajectory.cpp`
- `src/ego_planner_driver/test/test_ego_state_machine.cpp`
- `src/ego_planner_driver/test/test_safety_predictor.cpp`（新增）
- `src/ego_planner_driver/test/test_emergency_brake.cpp`（新增）
- `src/ego_planner_driver/test/test_yield_candidates.cpp`（新增）
- `src/swarm_uav_interfaces/msg/UavTrajectoryIntent.msg`
- `src/swarm_uav_executor/src/swarm_uav_executor/drivers/ego_swarm.py`（状态/终态映射与旧门禁 A/B）
- `src/swarm_uav_executor/test/test_ego_swarm_driver.py`
- `verification/ego_smoke/` 下新增闭环场景 runner，并更新现有 reciprocal 脚本的过时 rebound 前置条件；
- `src/tcp_to_ros/config/topology_group_a_sim.yaml`（仅在 intent 协议/topic 改变时同步）；
- `src/tcp_to_ros/docs/test_record_2608xxxx.md`（实施当日新建，不回填旧测试记录）。

不预先承诺新增第三方库；数值计算继续使用 Eigen/C++14，测试沿用 gtest/ROS/Python 现有栈。

## 9. 测试计划

### 9.1 纯数值与状态机测试

- 相向、追越、正交、背离、静止、零相对速度的 CPA/TTC；
- intent 不同激活 epoch、部分重叠时间窗、过期/未来/乱序/丢失；
- closing speed、数据年龄、制动减速度变化时两级门槛单调且 finite；
- right-of-way 数字比较（A01/A2/UAV10）、非法/相同 ID 降级；
- emergency brake 的 P/V/A 起点连续、停车终点、速度/加速度/jerk 限制；
- 四候选方向、速度档位、评分稳定性、全失败转制动；
- episode 锁存/释放、左右方向保持、紧急动作抢占晚到规划结果；
- tentative 不执行、active 才执行、1s 超时不等全员、已知冲突不提交；
- logger 覆盖写、身份文件名、队列满降级和退出 drain。

### 9.2 双节点闭环仿真（避碰专项，无 TCP/GCS）

新增可重复的受控动力学模型，不再像现有 smoke 一样把 setpoint 直接回灌为 pose。模型至少包含：

- 位置/速度/加速度状态和加速度/速度饱和；
- 可配置 50–300ms 控制延迟、跟踪一阶滞后和制动滑行；
- intent 0–1200ms 延迟、乱序、重复、单包/连续丢失；
- 两节点异步启动、空 inbox 同时启动、多轮 active/tentative/braking intent 更新；
- 记录实际轨迹而非仅检查规划采样。

场景：

1. 同时空 inbox 的同层正交；
2. 低 ID 先启动/高 ID 后启动，反向再测；
3. 同层换位；
4. 一侧 intent 延迟 800ms、乱序和短时丢失；
5. warning 重规划失败后 emergency brake；
6. nominal/LEFT 不可行而 RIGHT/SLOW/FAST 可行；
7. 四候选均失败，验证制动而非继续 nominal；
8. stale policy 各模式 A/B，默认模式不能高概率制造任务超时。

断言：实际中心距 ≥1.0m、速度 ≤1.5m/s、加速度 ≤`max_acc*1.05`、无双方重规划 ping-pong、
无同一 episode 左右振荡、制动后固定 HOLD、日志包含完整决策链。

### 9.3 PX4 SITL 专项测试（不走完整 TCP 链路）

按风险从低到高执行，每次先预检初始水平间距 ≥3m、OFFBOARD setpoint 唯一出口和日志目录可写：

1. 单机：制动轨迹注入和跟踪，标定实际制动距离；
2. 双机同层正交：低 ID nominal、高 ID 绕行/变速；
3. 双机同层换位：检验持续冲突和方向锁存；
4. 三机交叉：检验多 peer 最危险冲突选择和全邻机 clearance；
5. 对上述场景分别注入 200/400/800ms intent 延迟和控制延迟。

每轮录制 `/rosout`、odometry、tentative/active/braking intent、`exec_state`、hold、EGO candidate、
relay raw setpoint 和每机 ULog，并保存每机 `UAVn-ego-planner.log`。测试结束无论成功失败均 HOLD 全部测试机。

### 9.4 12机 prepare 完整链路回归

专项测试全部通过后，才启动 TCP→GCS→UAV 完整通信链路执行 A01–A12 prepare：

- 起始水平间距 ≥3m；
- 12/12 到达 `COMPLETED`，或任何真实安全失败按 ALL 策略安全收口；
- 全程中心距 ≥1.0m、速度 ≤1.5m/s；
- intent 延迟分布、warning/emergency 次数、候选选择和旧门禁触发次数写入测试记录；
- 若旧 1m 门禁仍触发，必须判定预测门禁为何未提前消除风险，不能仅把该轮视为安全通过。

构建/自动测试基线：

```bash
cd /home/ub20tg/catkin_swarm6-2
catkin build swarm_uav_interfaces ego_planner_driver swarm_uav_executor tcp_to_ros safe_valley_exp
catkin test ego_planner_driver swarm_uav_executor tcp_to_ros
catkin_test_results --all build
```

## 10. 推荐实施顺序与提交边界

1. **先可观测**：预测器、ID helper、异步日志和 shadow safety timer；不改变飞行行为；
2. **再保底**：emergency braking trajectory 与紧急抢占；保留旧 1m 门禁；
3. **再封首轮窗口**：tentative intent + 最多 1s 协商，缺失邻机不阻塞；
4. **再主动让路**：right-of-way + 四候选 + episode 方向锁存；
5. **先双节点闭环**：含真实动力学滞后和网络故障注入；
6. **再 SITL 递进**：单机制动 → 双机正交 → 双机换位 → 三机交叉；
7. **最后完整链路**：12机 prepare；
8. **最后才评估删除旧门禁**：关闭 A/B 全过后独立变更，不与候选实现混在同一提交。

建议提交拆分：

- `feat(safety): add predictive conflict monitor`
- `feat(safety): add constrained emergency braking`
- `feat(intent): add tentative activation barrier`
- `feat(avoidance): add deterministic yield candidates`
- `test(safety): add delayed multi-node dynamics tests`
- `docs(safety): record predictive avoidance validation`

每个提交都必须通过对应单测；协议字段与所有消费者/测试在同一兼容提交中完成。提交不 push。

## 11. 回滚方案

- 实施前记录 `swarm_uav_interfaces`、`ego_planner_driver`、`swarm_uav_executor`、`tcp_to_ros`、
  `safe_valley_exp` 各仓库基线 commit；
- shadow 阶段可通过 `safety_supervisor_mode=off` 回滚到无行为变化基线；
- active 阶段回滚必须同时恢复 intent phase、状态机、launch 和 executor 映射，禁止只回滚消息生产者；
- 四候选异常时可关闭 `enable_yield_candidates`，但 emergency braking 和旧 1m 门禁继续保留；
- 日志失败只能降级为 ROS 日志，不能使节点退出或禁止 emergency braking；
- 任一 SITL 出现 <1m、非有限 setpoint、多 publisher 或制动轨迹不连续，立即停止该级测试并回滚到上一个
  已验收阶段，不进入更多 UAV 或实机测试；
- 不以启用 rebound、增大 `lambda_swarm`、增大固定 clearance 或提高 PX4 增益临时掩盖失败。

## 12. 完成定义

- shadow 预测、两级 active 门禁、受约束制动、tentative 屏障、ID right-of-way 和四候选均有自动测试；
- 默认不会因从未出现的预期邻机阻止启动，1s 协商窗口语义有延迟/丢失测试；
- stale/missing 策略可关闭、可调，默认不恢复高误触发通信看门狗；
- 每机覆盖日志可从仿真输入或实机 hostname 正确命名，完整记录安全决策链；
- 双节点动力学闭环、SITL 双机正交、双机换位、三机交叉全部实际中心距 ≥1.0m；
- 无双方重规划 ping-pong、无让路方向振荡，速度 ≤1.5m/s，动力学约束不回归；
- 12机 prepare 使用完整通信链路回归并归档 rosbag、节点日志和 ULog；
- `enable_rebound=false`、PositionTarget 契约、relay 唯一出口、分层高度和 1s 常规重规划基线不回归；
- 旧 1m 门禁只有在关闭 A/B 全部通过后才可独立删除；未达到证据门槛则继续保留；
- 自动测试 0 failure，实施结果写入新的 `docs/test_record_2608xxxx.md`。
