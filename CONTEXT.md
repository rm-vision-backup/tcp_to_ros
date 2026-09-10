# Context Glossary

> 本文件是项目领域词汇表，只解释概念、不承载文件路径、实现算法与验证参数。
> - Phase 编号约定、全局设计规约、文档索引与当前状态（唯一权威）：`plans/README.md`
> - 当前实施计划：`plans/implementation_plan_26082415.md`（基线 `26082300`/`26082322`）
> - 近期测试记录：`test_records/test_record_26082400.md` / `test_records/test_record_26082300.md`
> - 包关系：`tcp_to_ros` 为 GCS_A 地面站业务包；`swarm_uav_interfaces` 为 GCS 与机载共同依赖的纯 ROS 接口包；`swarm_uav_executor` 为每架 UAV 本机部署的执行适配包；三者是相互独立的 Git 仓库。`swarm_topology_bridge` 是不解释 command 的通用通信包。工作空间根目录不拥有这些仓库，也不将两个新包登记为 submodule。

## Group A

本轮开发只指 A 组地面站与 A 组无人机任务链路，不包含 Group B。

## 整组批次任务

指 runtime 发给地面站的一次 Group A 批次命令。当前默认抽象为 6 个批次：`prepare-a`、`coverage-segment-1`、`fault-exit`、`recovery-a`、`coverage-segment-2`、`hold-a-end`。

整组批次任务不是逐机独立长会话。

## 批次执行时限

指 runtime 随整组批次任务下发的 `timeout_s` 所表达的全批次总执行时限。它覆盖首个目标点和后续目标点，
不存在由 GCS_A 额外硬编码的“首点 30 秒”时限。

## assignments

指整组批次任务内部的逐机任务项集合。地面站负责把 `assignments` 拆分为单机任务下发，并汇总单机状态。

## 任务参与集

指当前整组批次任务由 `assignments` 锁定的 UAV 集合。只有该集合中的链路故障会终结当前整组任务；身份配置中允许使用但未参与当前任务的 UAV 不属于任务参与集。

## 配置全集

指版本化身份配置中允许被解析和监测的 UAV 集合。配置启用不表示飞机必须上电，也不表示它自动成为当前任务参与机。

## 当前在线集

指已经被 GCS_A 收到有效链路心跳、可形成健康观察的 UAV 集合。未上电且从未被发现的配置成员不应被伪报为链路丢失。

## 飞行会话集

指已建立飞行安全租约且尚未安全结束的 UAV 集合。它可以跨越同一 mission 的多个任务批次，因此不等同于当前任务参与集。

## 分发层

指地面站与机载之间负责"任务下发 + 状态回传 + 整组汇总"的 ROS 通信层。不负责轨迹规划、编队形成、运动控制实现。

## 作业类型

指 `MOVE_TO`、`FOLLOW_ROUTE`、`FAULT_EXIT`、`HOVER` 这类任务类型。在分发层只表示任务类别，不表示具体算法实现。

## 统一任务接口

指地面站向单架无人机下发任务时采用统一的业务模型，通过 `command` 区分作业类型，而不是为每类作业定义独立的跨机接口。任务请求需要获得明确的接收确认，执行过程和终态通过单机状态回传。

## 接收确认

指单机任务已经通过校验、完成幂等登记，并成功交给本机执行入口。仅收到网络数据不构成接收确认。

## 整组启动屏障

指 MOVE_TO 参与机先完成任务登记与飞行前置状态检查（executor `prepare()` 只承担 armed、OFFBOARD、
原点已确认三项飞行前置门禁），全部报告 PREPARED 后才由 GCS_A 统一释放 START；任一成员拒绝或超时则
发送 ABORT。该内部屏障不改变 runtime 的 TCP 命令和状态协议，GCS_A 只在所有 START 已可靠登记后
向上游报告 ACCEPTED。START 只启动已登记任务，不负责 arm、切换 OFFBOARD 或触发起飞；5m TAKEOFF
在 PREPARE 之前已由 planner 完成。

## 混合跨机通信

指 GCS_A 与执行身份之间采用两类互补通信：任务请求和安全请求使用同步请求响应，执行过程与终态使用异步状态流。整组任务生命周期仍由 GCS_A 本地接口管理。

## 幂等任务

指具有相同任务唯一键的重复请求只能对应同一次实际执行。执行端需要返回当前状态或缓存结果，不得重复启动任务。

## 失败即停

指采用 `ALL` 完成策略时，任一参与机明确拒绝任务或无法完成接收确认，整组任务立即失败，并安全停止已经接受该任务的参与机。

## 单机状态三态

指单架无人机回传给地面站的主状态：`ACCEPTED`、`COMPLETED`、`FAILED`。`COMPLETED` 与 `FAILED` 为终态；允许附带细节字段（如 `detail_stage`、`message`、`error_code`）。

## 任务层身份

指任务包中使用的无人机身份命名，例如 `A01 ... A15`。

## 执行身份

指机载实际运行时的节点身份，例如 `UAV1 ... UAV15` 或对应 hostname。任务层身份与执行身份通过配置映射，不直接混用。

## 单机任务接口

指地面站向单架 UAV 下发统一任务请求的跨机 Service（`UavTask.srv`）。请求字段为全结构化 ROS 字段，不使用 `payload_json`；响应 `accepted=true` 表示任务已通过校验、完成幂等登记并交给本机执行入口，不表示任务已完成。

## 单机 HOLD 接口

指地面站向单架 UAV 请求安全悬停的跨机 Service（`UavHold.srv`）。稳定悬停完成后通过单机状态 Topic 报告终态。

## 链路健康

指 GCS_A 根据最近收到的本机飞控连接状态，形成的逐机全链路可达性观察。它用于任务前门禁、故障聚合和报警，但只证明最近一次上行心跳链路可达，不单独保证后续反向控制请求可达。

## 飞行安全租约

指 GCS_A 周期续期、由每架 UAV 本地 watchdog 独立监督的飞行会话凭证。租约失效后，本机不依赖跨机链路进入安全 HOLD；安全结束租约后才允许建立新的飞行会话。

## 会话 epoch

指一次飞行安全租约会话的唯一世代标识。旧世代的迟到续期不能解除已锁存的安全故障，也不能恢复已经失败的旧任务。

## 安全 END

指在无活动任务且本机满足安全落地状态时明确结束飞行安全租约。安全 END 只释放未来任务的安全锁存，不删除或复活旧任务终态。

## 单机状态消息

指单架 UAV 通过跨机 Topic 回传给地面站的执行状态（`UavTaskState.msg`）。主状态为单机状态三态；GCS_A 按任务唯一键与 `status_seq` 去重。

## 身份映射

指把任务层身份映射到执行层身份 / hostname / IP 的版本化配置（`uav_identity_map.yaml`），字段含 `uav_id / exec_target / hostname / ip / enabled`。实机使用前需人工复核。

## GCS_A

指 Group A 地面站在 `swarm_topology_bridge` 拓扑中的逻辑节点名。负责接收整组批次任务、按机下发统一任务消息、接收单机状态、汇总整组结果。

## GCS_A 分发后端

指 GCS_A 本地的整组 `ExecuteGroupTask` Action server（`gcs_a_backend_node.py`）。按身份映射拆分单机任务，并发确认并汇总单机状态，执行 `ALL` 失败即停与 HOLD；具体参数见 `plans/README.md`（Phase 2）。

## 通用 Service 代理

指 `swarm_topology_bridge` 中基于 ZMQ 的跨机请求/响应代理。配置声明 Service 名称、类型、目标节点；请求携带 bridge 层唯一 request ID，支持并发、超时与迟到响应丢弃。bridge 不理解任务业务，只代理 ROS 序列化请求/响应。

## 共享机载接口包

指 `swarm_uav_interfaces`。它只拥有 GCS 与机载共同使用的任务、状态、链路健康和飞行安全租约接口定义，不包含 GCS 聚合、机载动作或 bridge 业务逻辑。

## 单机执行器

指 `swarm_uav_executor`。每个实例在启动时绑定唯一且不可变的任务身份与执行身份，只能连接和控制本机动作 driver。其他 UAV 信息未来只能作为感知或避碰输入，不能形成跨机控制权。

## 动作 driver

指单机执行器与具体运动实现之间的可替换端口。首版实现确定性 mock 与 direct MAVROS position-setpoint；未来 EGO Planner、航线和编队能力通过新 driver 或上层协调器接入，不改变冻结的跨机接口。

## direct MAVROS driver

指首版直接向本机 MAVROS 持续发布 position setpoint，并以本机 pose/state 健康与连续稳定窗口判断 `MOVE_TO`/HOLD 的动作 driver。它默认不自动 arm、不自动切换 OFFBOARD，也不拥有其他 UAV 的控制权。

## 不上桨联调

指真实机体连接但不允许推进器产生飞行动力的现场安全验证，用于冻结身份/IP/namespace、坐标系/yaw、阈值、断联与 PX4 failsafe。单元测试、fake callback 和 PX4 SITL 均不能替代该门禁。

## PX4 SITL 主动飞行

指在 PX4 Software-In-The-Loop 中由仿真飞行器实际响应 setpoint 的闭环验证。它可以证明 executor、MAVROS、OFFBOARD 与仿真动力学链路成立，但不等价于真实硬件不上桨或实机上桨验收。

## 跨 Master 通信安全闭环

指 GCS_A 链路健康监测、任务参与集故障聚合、机载飞行安全租约、本地/远程 HOLD 分工和安全恢复共同形成的 Phase 5 安全能力。mock 多 Master 进程级断链已验证 ROS 业务闭环；使用真实 MAVROS + PX4 SITL（11300 仿真、A01/UAV1，2026-08-10）已验证状态断流、Service 断链、租约过期、本机 HOLD、链路恢复与安全 END 门禁。仍不替代实机不上桨门禁。


## 带时轨迹

指由规划器生成的、在每个时间点都给出位置（必要时含姿态）的平滑轨迹表示。runtime 提供的航点是稀疏关键点，不满足 PX4 平滑执行；执行层必须经 planner + trajectory generator 生成带时间的密集平滑轨迹后才下发。

## 机间避碰

指多机在执行各自轨迹并可能靠近时，保证无人机之间不碰撞的能力。本场景无环境静态障碍，障碍物仅为其他无人机；由 ego-swarm 风格的分布式轨迹规划在优化目标中把邻机轨迹视为动态障碍（互惠避碰）实现。

## 互惠避碰 (reciprocal collision avoidance)

指 ego-swarm 原版运行时避碰语义：参与机对彼此的轨迹施加对等惩罚（论文式 5），双方都主动避开，而非单方面让行。与“单向优先级避碰”不同；本计划 P1 按原版实现，不做单向改造。

## 规划范围 (planning range)

指本机在规划时将哪些邻机轨迹纳入避碰计算的阈值范围；超出该范围（或其轨迹离本机足够远）的邻机被忽略，以控制计算复杂度。对应论文 §V-B-1 “any trajectory outside the planning range will be ignored”。

## leader 参考

指编队跟随中 follower 所跟踪的 leader 实时位姿来源。follower 目标 = leader 参考 + 本机固定编队偏移；leader 参考 topic 超过 1 s 未刷新时视为丢失，本机应执行 HOLD 并报告 LEADER_LOST。

## 编队槽位

指按固定编队矩阵为每架 follower 预先确定的相对 leader 的固定偏移位置。A01 固定为 leader；
A 组 coverage-1 为 3 行×4 列（12 架）、coverage-2 为 3 行×3 列（9 架），间距 5 m。
槽位使用任务场地坐标系，由 GCS_A 的版本化配置实例化，不依赖 runtime 逐机下发编队形状。

## 固定编队

指 GCS_A 对 coverage 批次认可的成员集合与编队槽位组合。当前编队1对应 A01 至 A12，
编队2对应 A01 至 A06 与 A13 至 A15；收到的 FOLLOW_ROUTE 参与集必须与其中一个固定编队精确一致，
否则不得执行。

## 分层高度

指按命令划分的飞行高度层：MOVE_TO 从已完成 5m TAKEOFF 并处于 HOLD 的状态开始，先垂直到 15m，
再水平移动，最后下降到目标点；FOLLOW_ROUTE leader 保持 12m 编队层并使用 EGO 导航，followers 通过
PI 跟踪 leader 位置与编队槽位；FAULT_EXIT 先下降到 8m 离队层，再使用 EGO 水平移动到目标点。
5m TAKEOFF 是任务前的 planner 状态，不属于 MOVE_TO 子阶段。MOVE_TO 的三段动作按顺序分别执行并
通过实测到达门禁，所有任务子阶段共享 runtime 下发的同一个批次执行时限，不因切换阶段而重置计时。

## 机间最小距离

指任意两架无人机机体中心之间必须维持的统一球形安全距离，判定量均为三维欧氏中心距，不再使用
“水平 <1.0m 且垂直 <2.0m”的圆柱规则。planner hard-clearance 对本机与邻机预测轨迹按同一绝对时刻
采样，3D 中心距 `<1.0m` 即不通过；executor runtime gate 只使用 fresh odom，实测 3D 中心距
`<1.0m` 即触发安全 HOLD。SafetyPredictor 使用更保守的边界，预测 3D 中心距 `<=1.0m` 即判定越界。

## Safety 监督器

指 `ego_planner_driver` 的预测安全监督能力。正式运行默认状态自 2026-08-27 起为 `active`；
2026-08-25 曾采用 shadow 并禁用 active 的说明属于历史状态，仅用于解释当时测试与决策，已被
2026-08-27 的 active 状态取代。

## 误差积分

指编队跟随（FOLLOW_ROUTE follower）在位置层对偏差的积分项。follower 目标 = leader 参考 + 编队槽位偏移；本机当前位姿与该目标的偏差经 P 项与 I 项合成补偿速度，I 项累积稳态偏差以缓解 leader 参考延迟与链路抖动造成的滞后。补偿速度按水平/垂直分别限幅，超时未刷新 leader 参考则判为 LEADER_LOST。

## 轨迹意图

指本机通过机间 Topic 广播的自身带时轨迹描述（含 uav_id、轨迹 id、采样时间与全局 ENU 位姿、所需间距），供邻机规划时做互惠避碰。编队跟随模式（P2）中 follower 不发布、不接收轨迹意图，从而与 ego-swarm 网络互不可见。

## 全局原点

指飞行会话起飞前，由 GCS_A 根据达到质量门槛的无人机全球位置中位数确定并向全机统一下发的
GPS 地理原点（经纬高）。它在满足可配置的最小合格机数后冻结，后续无人机只使用该原点而不触发重算，
使仿真与实机的 local ENU 坐标系共享同一地理基准。全局原点只确定平移基准，不定义任务场地的朝向；
其冻结生命周期等同于 GCS_A 进程生命周期，只有操作员重启 GCS_A 才重新标定。

## 原点标定集

指冻结全局原点时，全球位置已达到质量门槛并被纳入中位数计算的无人机集合。它不等同于单个批次的任务参与集，
当前标定计算集由 `calibration_uavs` 参数指定（默认 UAV1 至 UAV6），其中每架必须全部通过全球位置、
姿态新鲜度与航向一致性门禁。标定过程不得通过自动剔除离群成员来构造一致子集；标定计算集内任一成员
未通过，都会阻止原点与集群参考航向冻结以及后续起飞流程。集合外的已登记 UAV（`all_uavs`，默认
UAV1 至 UAV16）不参与本次原点和航向计算，且未上电时不阻塞 ready；冻结结果下发 `all_uavs`，
在线成员回读确认。A01 至 A12 仍是 formation-1/prepare/coverage 成员集，不得因标定集合收缩而替换。

## 任务场地坐标系

指 runtime 三维任务坐标进入飞行系统后采用的右-前-上坐标系：x 向场地右侧、y 向场地前方、z 向上。
其水平朝向由 arm 前冻结的集群参考航向确定，不绑定地理东、北方向；runtime 携带的 yaw 不作为飞行朝向依据。

## 集群参考航向

指飞行会话 arm 前，根据达到姿态质量门槛的无人机机头朝向求得并冻结的共同水平朝向。
它定义任务场地坐标系的“前方”；原点标定集中每架无人机与共同朝向的最大圆周角差不得超过
可配置门槛（当前默认 30°），冻结后在 GCS_A 本次进程生命周期内不随单机姿态、任务批次或
后续上线成员改变，也不进行逐任务复验。

## 标定就绪

指操作员已通过 QGC 确认飞控状态正常和 RTK 收敛，随后 GCS_A 完成 `calibration_uavs`（默认
UAV1 至 UAV6）的全局原点与集群参考航向冻结，并完成该集合的原点回读确认。RTK 收敛不由 GCS_A 等待；标定就绪之前
不允许进入外部起飞与任务执行流程。

## 原点应用确认

指机载端从飞控回读到与 GCS_A 冻结值一致的实际全局原点事件。该确认是一次性的，不要求飞控
持续发布原点，也不使用连续稳定时间窗口。`calibration_uavs`（默认 UAV1 至 UAV6）用于计算原点和
参考航向，冻结结果下发 `all_uavs`（默认 UAV1 至 UAV16）；`calibration_uavs` 全部完成一次性回读
确认后，GCS_A 即进入标定就绪（缺席机不阻塞），后续任务不再复验。

确认逻辑仅使用 MAVROS 提供的 ROS 话题：先订阅每架无人机的 `gp_origin`，再向对应的
`set_gp_origin` 发布冻结的 `geographic_msgs/GeoPointStamped`，并且只接受本次发布之后收到的
回读事件。业务代码不直接处理 MAVLink 编码，也不自行进行经纬度量化、椭球高与海拔高转换。
将发布值与回读值换算为空间误差后，水平距离不超过 0.1 m 且高度差绝对值不超过 0.1 m 才算
确认成功；超差或等待超时则重试。

## 任务锚点

指任务场地坐标系中与标定计算集全球位置中位点对应的固定坐标。它为任务坐标到共享 local ENU
坐标的平移基准，当前固定为 `(0, 0, 0)`，与配置及源码一致，不表示实机中任何指定无人机必然位于该点。

## 偏置航点

指 runtime 下发的任务坐标相对于任务锚点所表达的场地位移目标。无人机起飞前位置与编队形状不构成
任务坐标基准；执行层只要求在任务规定时间内到达转换后的目标。

## 按需起飞

指完成标定后，由外部起飞脚本 `offboard_takeoff_15_sitl.py` 对本机执行 HOLD（拒绝时回退 AUTO.LOITER）
→ arm → OFFBOARD；随后 `ego_planner_driver` 的 `TAKEOFF` 状态软起飞至
`takeoff_height_m=5.0` 并锁存 HOLD。完成上述流程后，executor PREPARE 只检查 armed、OFFBOARD、
原点确认，GCS START 只启动已经登记的任务。START 不触发按需起飞，5m 也不是
MOVE_TO 子阶段。

## arm 权限

指对本机飞控执行 arm 与切换 OFFBOARD 的动作。当前只由外部起飞脚本（`offboard_takeoff_15_sitl.py`）
执行：先 HOLD（失败时 AUTO.LOITER），再 arm，最后 OFFBOARD。executor 不调用 arm/set_mode 服务，
PREPARE 只检查状态；GCS START 不拥有 arm/OFFBOARD 或起飞权限。多机任务中脚本也仅操作对应本机飞控。

## 起飞检查点

指 `ego_planner_driver` 的 `TAKEOFF` 状态：外部脚本完成 arm→OFFBOARD 后，以本机起飞位置为
水平基准、从当前 z 以 `takeoff_ascend_speed` 软上升到 `takeoff_height_m`（5m），完成后锁存 HOLD。
它发生在 executor PREPARE 和 GCS START 之前，不属于 MOVE_TO 的任何子阶段。
不再有“检查点容差连续稳定 2s”门禁（早期按需起飞设计，已随外部起飞流程移除）。

## 约束垂直轨迹

指在阶段开始时冻结本机水平位置与航向，只沿 z 轴按照速度和加速度约束生成的升降轨迹。
它用于 TAKEOFF 软起飞、MOVE_TO 进入 15 m 层及下降到目标高度、FOLLOW_ROUTE leader 进入 12 m 层、
FAULT_EXIT 进入 8 m 层，并以实测三维位置稳定到达作为完成条件。只有各任务层内的水平航段使用 EGO 导航。
系统不在升降前预测垂直通道冲突；升降期间实时机间距离突破安全门槛时，当前整组批次任务立即失败并 HOLD。
