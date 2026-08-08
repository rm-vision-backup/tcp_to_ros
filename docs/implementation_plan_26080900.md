# Implementation Plan

[Overview]
在现有 Group A 分发与单机执行基线之上建立“GCS_A 链路健康监测 + 机载安全租约看门狗”的分层通信安全闭环，以可复现的进程级断网注入收口 Phase 5，并把验证结果作为进入 Phase 6 实机不上桨联调的前置门禁。

本计划承接 `/home/ub20tg/catkin_swarm6-2/src/tcp_to_ros/handoff/handoff_swarm6-2_26080821.md`，不回填修改冻结基准 `/home/ub20tg/catkin_swarm6-2/src/tcp_to_ros/docs/implementation_plan_26080804.md`。当前 Phase 4 `MOVE_TO + HOLD` 首版、自动化回归和三机 PX4 SITL 主动飞行已经完成；本轮不扩展 `FOLLOW_ROUTE`、`FAULT_EXIT`、`HOVER`、EGO Planner、编队或机间避碰，而是先解决实际飞行前最关键的通信丢失发现与本地安全停止问题。

健康监测复用 `swarm_topology_bridge` 现有 Topic 来源前缀规则。每机本地 `/mavros/state` 原样经过 bridge 后，在 GCS_A ROS Master 中形成 `/{exec_target}/mavros/state`。因此，GCS_A 收到一条 `connected=true` 的新消息，能够同时证明本轮观测时刻的 PX4 飞控、机载 MAVROS/ROS、机载 bridge、自组网链路、GCS_A bridge 与 GCS_A ROS 订阅链可达。该结论只表示“最近一次心跳对应的上行全链路健康”，不能证明反向控制链此后仍然可达，也不能单独保证断链后 GCS_A 发出的 HOLD 能到达故障机。

为消除这一安全缺口，采用分层双看门狗：GCS_A 负责发现每机 `DISCOVERED/HEALTHY/SUSPECT/LOST/RECOVERED`、执行任务参与集门禁、在参与机 LOST 时终结整组任务并 HOLD 其他可达参与机；每架 UAV executor 负责维护由 GCS_A 周期续期的飞行安全租约，租约超时后不依赖跨机链路而直接调用本机 HOLD。GCS_A 不在已判定 LOST 后继续依赖 `UavHold` 控制故障机，避免跨机 HOLD 与本地租约 HOLD 竞争；故障机仅由本地租约看门狗接管。

成员管理区分四个集合：版本化身份配置中的“配置全集”、已收到有效心跳的“当前在线集”、已建立租约且尚未安全结束的“飞行会话集”、当前整组 `assignments` 锁定的“任务参与集”。GCS_A 启动时不等待固定 A01-A15 全部上线，也不把未上电飞机视为故障。每条任务仅要求其 `assignments` 对应飞机连续健康；任务开始前飞机上电顺序不固定，但所有参与机必须先达到健康稳定门禁、成功建立安全租约，之后才能下发 `UavTask`。一架飞机正常情况下上线后至任务结束落地前不应退出；其退出将按本计划视为意外通信故障。

`/mavros/state` 约为 1 Hz。GCS_A 在 callback 中使用接收端单调时钟记录最后一条消息的到达时刻，并检查 `msg.connected == true`；不使用消息时间戳推导超时。默认连续 3 秒没有有效心跳进入 `SUSPECT` 并报警，连续 5 秒进入 `LOST`。首条任务前要求参与机连续健康 3 秒（至少覆盖连续 3 条有效心跳）；所有阈值均由 ROS 参数配置，实机不上桨阶段再现场复核。收到 `connected=false` 时立即视为无效健康心跳并进入故障判定，不等待旧的 `connected=true` 永久续命。

安全租约以 `mission_id + session_epoch` 标识飞行会话。首条同 `mission_id` 任务前，由 GCS_A 为参与机执行 START；后续每机以 1 Hz 并发 RENEW，默认 TTL 为 5 秒。任务终态不结束租约，以覆盖批次之间仍在空中悬停的时间。只有“无活动任务且机载持续报告 `armed=false` 达到稳定窗口”或操作员显式请求结束时才执行 END；切换新 `mission_id` 前旧会话必须结束。链路恢复只形成 `RECOVERED` 并允许未来新的 `command_id`，不得恢复已因 LOST/HOLD 失败的旧任务。

活动任务参与机 LOST 时，现有 `ALL` 语义保持不变：当前 Action/TCP 命令立即 `FAILED`，`failed_uavs` 包含断联机，`error_code` 使用 `UAV_LINK_LOST`，并向其他已接受且未终止的参与机请求 HOLD。飞行会话内但未参与当前批次的飞机 LOST，只触发该机本地租约 HOLD 和 GCS_A ROS 报警，不使当前整组任务失败。不属于活动 `command_id` 的实验意外故障停留在 GCS_A ROS 侧，不伪造 TCP `command_id`，不向 runtime 发送异步 `FAILED`，也不修改现有 TCP JSON Lines 协议。

软件 HOLD 不是 PX4 failsafe 的替代品。direct MAVROS driver 捕获 HOLD 位姿需要新鲜 pose/state；如果通信故障同时伴随 MAVROS 或定位失效，本地 HOLD 可能返回失败。executor 必须锁存安全故障、停止接受新任务并记录 `HOLD_FAILED`，最终安全仍依赖 Phase 6 前现场配置和不上桨验证的 PX4 datalink/offboard/RC failsafe。默认禁止自动 arm 和自动切换 OFFBOARD 的基线保持不变。

Phase 5 的网络故障模型按实际需求收缩为进程级可复现注入：暂停/恢复或受控重启指定 UAV bridge，覆盖短时断网、超过 TTL 的全断、重连和迟到消息；Service 迟响应继续使用 mock executor 注入。不引入 `tc netem`、sudo 网卡规则或通用 bridge 内置测试分支。验收归档健康 Topic、Action/TCP 结果、executor/bridge/GCS 日志、命令参数和必要 rosbag。

[Types]

### 共享 ROS 接口

1. 新增 `swarm_uav_interfaces/UavLinkHealth.msg`
   - `string protocol_version`：固定为 `"1.0"`。
   - `string uav_id`：任务层身份，例如 `A01`。
   - `string exec_target`：执行身份，例如 `UAV1`。
   - `string state`：只允许 `DISCOVERED`、`HEALTHY`、`SUSPECT`、`LOST`、`RECOVERED`。
   - `bool connected`：最近收到的 `mavros_msgs/State.connected`；从未收到消息时为 `false`。
   - `float64 age_s`：以 GCS_A 接收端单调时钟计算的最近有效心跳年龄；从未收到时使用 `-1.0`，不得使用 NaN/Inf。
   - `string mission_id`：该机当前飞行会话所属 mission；未建立会话时为空。
   - `string session_epoch`：该机当前租约 epoch；未建立会话时为空。
   - `string reason`：稳定、可审计的状态变化原因，不用于控制分支。
   - `float64 reported_at`：发布时 ROS time 秒值，仅用于记录和 rosbag 对齐，不参与 watchdog 超时。
   - GCS_A 发布 Topic 固定为 `/group_a/uav_link_health`；状态变化时立即发布，并以可配置低频周期发布当前快照，便于晚加入订阅者观察。

2. 新增 `swarm_uav_interfaces/UavSafetyLease.srv`
   - 请求 `string protocol_version`：固定为 `"1.0"`。
   - 请求 `string operation`：只允许 `START`、`RENEW`、`END`。
   - 请求 `string mission_id`：非空；必须与当前会话一致。
   - 请求 `string session_epoch`：由 GCS_A 生成的 UUID 字符串，非空。
   - 请求 `uint64 lease_seq`：同一 epoch 内严格递增；START 使用初始序号，重复的完全相同请求按幂等处理。
   - 请求 `float32 ttl_s`：START/RENEW 必须为有限正数并处于配置允许范围；默认 `5.0`；END 可保留最后 TTL 值但不据此续期。
   - 请求 `string uav_id`、`string exec_target`：必须与 executor 启动时冻结身份完全一致。
   - 请求 `string reason`：仅用于日志，不按文案决定 operation。
   - 响应 `bool accepted`：表示 operation 已完成校验并原子应用；RENEW 请求一旦合法接收即续期，即使响应随后在反向链路丢失也不回滚。
   - 响应 `string state`：`INACTIVE`、`ACTIVE`、`EXPIRED_HOLD` 或 `END_PENDING`。
   - 响应 `string error_code`、`string message`：稳定错误码和审计说明。

### GCS_A 内部数据结构

- `LinkHealthState`：字符串常量/枚举集合 `UNDISCOVERED`、`DISCOVERED`、`HEALTHY`、`SUSPECT`、`LOST`、`RECOVERED`；`UNDISCOVERED` 只用于进程内初态，不发布为已发现事件。
- `UavLinkObservation`：包含 `uav_id`、`exec_target`、`last_message_monotonic_s`、`last_valid_monotonic_s`、`last_connected`、`valid_sample_count`、`healthy_since_monotonic_s`、`state`、`reason`。所有字段由同一锁保护。
- `SafetyLeaseSession`：包含 `mission_id`、`session_epoch`、逐机 `lease_seq`、`members`、`started_members`、`renew_failures`、`created_monotonic_s`、`ending`。同一 GCS_A 实例同时最多一个未结束飞行会话。
- `ParticipantHealthGateResult`：逐机记录健康稳定门禁和 START 结果，用于在 4 秒任务确认预算外形成明确的前置拒绝原因。飞行会话门禁必须在向 runtime 发送整组 ACCEPTED 之前完成，但其最大允许时间需由独立参数限制并在 TCP 5 秒 ACCEPTED 约束内统筹；默认要求任务到达时参与机已经健康，不能通过等待上电来消耗协议预算。
- `GroupState` 增加 `link_lost_uavs`，并保持它是 `failed_uavs` 的原因子集；同一 UAV 只记录一次 LOST，恢复消息不得清除当前任务终态。

### 机载内部数据结构

- `SafetyLeaseRecord`：不可变快照，字段为 `mission_id`、`session_epoch`、`last_seq`、`ttl_s`、`last_renew_monotonic_s`、`state`、`expired_at_monotonic_s`。
- `SafetyLeaseConfig`：包含 Service 名、watchdog 检查频率、允许 TTL 最小/最大值、默认 TTL、disarmed 稳定窗口和是否要求租约才能接受正常任务。默认 watchdog 检查频率至少 10 Hz，默认 TTL 5 秒。
- 租约错误码冻结首版集合：`INVALID_LEASE_PROTOCOL`、`INVALID_LEASE_OPERATION`、`LEASE_IDENTITY_MISMATCH`、`INVALID_LEASE_TTL`、`LEASE_EPOCH_CONFLICT`、`LEASE_SEQ_STALE`、`LEASE_NOT_ACTIVE`、`LEASE_EXPIRED`、`LEASE_END_UNSAFE`、`LEASE_HOLD_FAILED`。
- 链路/任务错误码新增：`UAV_LINK_SUSPECT` 只用于 ROS 健康详情，不终结任务；`UAV_LINK_LOST` 用于活动任务失败；机载租约过期导致活动任务终结时使用 `SAFETY_LEASE_EXPIRED`，HOLD 本身失败则保留 `LEASE_HOLD_FAILED`/`HOLD_FAILED` 事实。

### 状态机与幂等规则

- GCS 链路状态：首次收到消息进入 `DISCOVERED`；连续有效心跳满足稳定门禁进入 `HEALTHY`；有效心跳年龄达到 3 秒进入 `SUSPECT`；达到 5 秒进入 `LOST`；LOST 后重新连续健康时先发布 `RECOVERED`，随后进入 `HEALTHY`。单条恢复心跳不得自动恢复旧任务。
- 机载租约状态：`INACTIVE -> ACTIVE -> EXPIRED_HOLD`；安全 END 为 `ACTIVE -> END_PENDING -> INACTIVE`。`EXPIRED_HOLD` 为当前 epoch 锁存状态，旧 epoch 的 RENEW 不能恢复；只有明确结束旧 epoch 后，以新 epoch START 才允许新任务。
- 同一 `(session_epoch, lease_seq, operation)` 完全重复时返回缓存结果；同 epoch 更小序号拒绝为 `LEASE_SEQ_STALE`；同 epoch 相同序号但内容不同拒绝为冲突；ACTIVE 状态收到不同 epoch 的 START 必须拒绝，防止 GCS 重启静默覆盖飞行中的保护会话。

[Files]

### 新建文件

#### `/home/ub20tg/catkin_swarm6-2/src/swarm_uav_interfaces/`

- `msg/UavLinkHealth.msg`：定义 GCS_A ROS 侧机器可读的逐机链路健康事件/快照。
- `srv/UavSafetyLease.srv`：定义每机 START/RENEW/END 安全租约契约。

#### `/home/ub20tg/catkin_swarm6-2/src/tcp_to_ros/`

- `scripts/tcp_to_ros/link_health.py`：纯逻辑健康状态机、单调时间超时判定、稳定窗口与消息快照转换，不包含 Action/TCP 业务。
- `scripts/tcp_to_ros/safety_lease.py`：GCS 飞行会话、逐机租约 START/RENEW/END、序号和并发续期协调器。
- `test/test_link_health.py`：GCS 链路状态机纯单元测试。
- `test/test_safety_lease_coordinator.py`：GCS 租约会话、并发调用、失败补偿和 epoch/seq 单元测试。
- `test/test_link_loss_aggregation.py`：活动参与机 LOST 与非参与机会话成员 LOST 的整组聚合测试。
- `test/link_safety_multimaster.test`：若 rostest 能稳定编排多进程则作为入口；若 ROS Master 多实例不适合单一 rostest，保留为单 Master 组件集成并由外部脚本负责多 Master。
- `test/run_link_fault_injection.py`：非交互测试编排器，启动或连接 GCS_A/UAV 独立 Master，记录 PID，使用 `SIGSTOP/SIGCONT` 或受控重启指定 bridge，采集结果并确保 finally 清理；禁止修改产品 bridge 注入测试分支。
- `config/link_safety_defaults.yaml`：GCS 端 3 秒 SUSPECT、5 秒 LOST、3 秒健康稳定窗口、1 Hz RENEW、5 秒 TTL、快照发布频率和会话结束阈值。
- `launch/gcs_a_link_safety_test.launch`：单 Master mock 组件测试入口。

#### `/home/ub20tg/catkin_swarm6-2/src/swarm_uav_executor/`

- `src/swarm_uav_executor/safety_lease.py`：机载租约状态机、Service handler、单调时钟 watchdog 与锁存过期逻辑。
- `test/test_safety_lease.py`：START/RENEW/END、幂等、旧 epoch、倒退 seq、过期和恢复锁存单元测试。
- `test/test_safety_lease_executor.py`：租约过期与 `UavTaskExecutor` 活动任务/HOLD 的集成测试。

### 修改文件

#### `/home/ub20tg/catkin_swarm6-2/src/swarm_uav_interfaces/`

- `CMakeLists.txt`：将 `UavLinkHealth.msg` 加入 `add_message_files`，将 `UavSafetyLease.srv` 加入 `add_service_files`；不增加不必要的消息依赖。
- `README.md`：记录健康状态、租约状态机、epoch/seq 幂等、防迟到续命和版本升级要求。

#### `/home/ub20tg/catkin_swarm6-2/src/tcp_to_ros/`

- `scripts/gcs_a_backend_node.py`：装配每机 MAVROS state 订阅、健康 Topic publisher 和租约协调器；首条任务执行健康/START 门禁；等待终态期间吸收参与机 LOST；任务终态后保持飞行会话续租；结束条件满足时执行 END；不把无活动任务的报警发送给 TCP。
- `scripts/tcp_to_ros/__init__.py`：仅在现有包导出约定需要时导出新逻辑类，避免节点从脚本路径复制实现。
- `config/topology_group_a_sim.yaml`：在 Topic 列表加入 `mavros/state`、类型 `mavros_msgs/State`、`max_freq: 0`；在每机 Service 列表加入 `/{exec_target}/uav_safety_lease`、类型 `swarm_uav_interfaces/UavSafetyLease`。GCS_A 拓扑仍只接收实际配置的 UAV 来源。
- `config/link_safety_defaults.yaml`：由 launch 加载并允许现场覆盖；不得把 3/5 秒阈值散落硬编码在 callback。
- `config/uav_identity_map.yaml`：不把 `enabled=true` 解释为“必须上电”；补充注释说明 enabled 仅表示可被任务解析和监测，任务参与集仍由 assignments 锁定。
- `launch/gcs_a_backend.launch`：加载链路安全参数，增加健康 Topic、租约频率/TTL、健康门禁和安全 END 参数。
- `launch/gcs_a_backend_test.launch`、`launch/gcs_a_backend_fail.launch`：为 mock executor 启用可控租约 Service 与 MAVROS state 测试源，保持原正常、幂等和失败即停用例。
- `launch/topology_group_a_sim.launch`：为 UAV mock 测试角色启动可控 `/mavros/state` 发布源，或通过参数选择真实 MAVROS；不得在 GCS_A 角色发布伪心跳。
- `CMakeLists.txt`：注册新增 nosetest/rostest 和故障注入脚本安装项；加入运行时使用 `mavros_msgs` 后所需 catkin 依赖。
- `package.xml`：增加 `mavros_msgs` build/exec 依赖和新增测试所需依赖；不新增 pip 包。
- `README.md`、`CONTEXT.md`：README 记录部署参数和运行步骤；CONTEXT 仅新增“链路健康”“飞行安全租约”“任务参与集”“飞行会话集”等领域词汇，不写文件路径和阈值算法。
- `docs/tcp_to_ros_后续开发计划.md`：Phase 5 更新为通信安全闭环与简化故障注入完成后方可收口；Phase 6 增加租约/HOLD/PX4 failsafe 不上桨门禁。
- `docs/tcp_to_ros_工程过程.md`：实施完成后追加本轮架构和落地过程。
- `docs/tcp_to_ros_测试记录.md`：按日期追加两 Master topic 冒烟、单元/ROS 回归、短断/长断/重连和失败证据。
- `docs/tcp_to_ros_提问记录.md`：把本轮已确认决策写入历史，移除或标记相应待确认项，不复制完整计划。

#### `/home/ub20tg/catkin_swarm6-2/src/swarm_uav_executor/`

- `src/swarm_uav_executor/models.py`：新增 `SafetyLeaseConfig`、`SafetyLeaseRecord` 和租约状态/错误常量；扩展 `ExecutorConfig` 以包含租约 Service 和 watchdog 参数。
- `src/swarm_uav_executor/executor.py`：提供线程安全的“租约过期并本机 HOLD”入口；过期时取消活动任务、锁存安全故障、调用 driver HOLD，并拒绝旧任务恢复；避免直接从 watchdog 复制 `handle_hold` 请求对象。
- `scripts/uav_executor_node.py`：装配 `UavSafetyLease` Service、`SafetyLeaseWatchdog`、单调时钟和 shutdown 顺序；只有配置明确允许的测试模式才能关闭租约门禁，真实 MAVROS launch 默认启用。
- `config/executor_defaults.yaml`：加入 `safety_lease` 默认配置，默认 1 Hz 上游续期对应 5 秒 TTL，watchdog 检查频率至少 10 Hz；保留 direct driver 当前 state/pose 健康阈值，两类超时不得混为一个参数。
- `launch/uav_executor_mock.launch`：增加租约 Service 名、测试时是否要求租约和可控过期参数。
- `launch/uav_executor_mavros.launch`：默认启用安全租约，显式绑定 `/{exec_target}/uav_safety_lease`；继续强制 `auto_arm=false`、`auto_offboard=false`。
- `CMakeLists.txt`：注册新增租约测试。
- `README.md`：补充租约启动/结束、锁存恢复、HOLD 失败与 PX4 failsafe 边界。

#### `/home/ub20tg/catkin_swarm6-2/src/swarm_topology_bridge/`

- 通用 `scripts/bridge_node.py` 原则上不修改；先通过冒烟验证它已将源 `/mavros/state` 发布为 GCS 侧 `/{source}/mavros/state`，并能动态代理 `UavSafetyLease`。
- 只有冒烟发现 Topic 名规范化或 Service 并发存在通用缺陷时才修改 `scripts/bridge_node.py` 及其通用测试；不得加入 MAVROS、租约或 Group A 业务判断。
- `README.md`：仅在需要时增加动态 Topic/Service 配置示例，不增加 `swarm_uav_interfaces` 编译依赖。

### 删除或移动文件

- 本轮不删除、移动现有 ROS Action/msg/srv 或 mock/direct driver。
- 不修改或替换冻结计划 `implementation_plan_26080804.md`。
- 不把新接口重新定义到 `tcp_to_ros`；共享健康消息与租约 Service 只归属 `swarm_uav_interfaces`。
- 不将独立仓库登记为工作空间根仓库的 submodule，不配置 remote，不 push。

[Functions]

### 新函数

#### `/home/ub20tg/catkin_swarm6-2/src/tcp_to_ros/scripts/tcp_to_ros/link_health.py`

- `record_state(uav_id: str, exec_target: str, connected: bool, received_monotonic_s: float) -> UavLinkObservation`：原子记录 callback 到达和 connected 值；只有 connected=true 更新最后有效心跳。
- `evaluate(now_monotonic_s: float) -> List[LinkHealthTransition]`：按稳定窗口、SUSPECT/LOST 阈值计算所有已发现飞机的状态变化；未发现飞机不生成 LOST。
- `is_stably_healthy(uav_id: str, now_monotonic_s: float) -> bool`：供任务前门禁查询，要求当前 HEALTHY 且稳定时间满足配置。
- `snapshot(uav_id: str, ros_time_s: float, session: Optional[SafetyLeaseSession]) -> UavLinkHealth`：构造结构化健康消息，age 使用单调时钟差，reported_at 使用 ROS time。
- `bind_session(uav_id: str, mission_id: str, session_epoch: str) -> None`、`clear_session(...) -> None`：只更新健康快照中的会话关联，不改变链路健康状态。

#### `/home/ub20tg/catkin_swarm6-2/src/tcp_to_ros/scripts/tcp_to_ros/safety_lease.py`

- `start_session(mission_id: str, members: Sequence[Identity], deadline_monotonic_s: float) -> LeaseOperationResult`：生成 UUID epoch，并发向参与机 START；全部成功才允许任务下发，部分成功时对已 START 机执行有界 END 补偿并返回失败事实。
- `ensure_members(mission_id: str, members: Sequence[Identity], deadline_monotonic_s: float) -> LeaseOperationResult`：同 mission 后续任务复用活动 epoch，对新加入任务的飞机先完成健康门禁和 START，不重启已有成员租约。
- `renew_due(now_monotonic_s: float) -> None`：每秒对会话成员并发 RENEW；逐机递增 seq；响应失败只记录反向确认异常，机载是否续期以其合法接收为准。
- `end_member(uav_id: str, reason: str, deadline_monotonic_s: float) -> LeaseOperationResult`：仅在安全结束条件满足时发送 END；失败保留 END_PENDING 和报警，不伪装会话已结束。
- `end_session(reason: str, force: bool = False) -> LeaseOperationResult`：操作员或安全自动结束入口；`force` 不得绕过 armed/活动任务检查，除非后续另行形成现场安全 ADR。
- `_build_request(identity, operation, session, lease_seq, reason) -> UavSafetyLeaseRequest`：统一构造冻结字段。
- `_call_member_with_timeout(...) -> LeaseMemberOutcome`：沿用有界工作线程模式，隔离迟响应；旧 seq 的迟响应不得覆盖新状态。

#### `/home/ub20tg/catkin_swarm6-2/src/swarm_uav_executor/src/swarm_uav_executor/safety_lease.py`

- `handle_lease(request: UavSafetyLeaseRequest) -> UavSafetyLeaseResponse`：校验协议、身份、operation、TTL、epoch 和 seq，原子应用 START/RENEW/END，并缓存幂等响应。
- `_handle_start(...)`：仅在 INACTIVE 时创建 epoch；EXPIRED_HOLD 或其他 ACTIVE epoch 不得被静默覆盖。
- `_handle_renew(...)`：合法请求先更新接收单调时间与 seq，再返回响应；响应发送失败不撤销更新。
- `_handle_end(...)`：要求无活动任务且 driver/MAVROS 显示持续 disarmed 达配置窗口；不安全时返回 `LEASE_END_UNSAFE` 并保持 ACTIVE。
- `check_expiry(now_monotonic_s: float) -> Optional[LeaseExpiryEvent]`：周期检查 `now-last_renew >= ttl_s`，只产生一次锁存过期事件。
- `acknowledge_new_epoch(...)`：只允许在旧 epoch 已明确 INACTIVE 后由 START 路径创建，不提供单独绕过安全锁的方法。
- `shutdown() -> None`：停止 watchdog timer；若仍处于 ACTIVE，调用 executor 安全停止入口后再结束节点，不自动伪造 END。

#### `/home/ub20tg/catkin_swarm6-2/src/tcp_to_ros/test/run_link_fault_injection.py`

- `start_role(role: str, master_uri: str, args: Sequence[str]) -> ManagedProcess`：非交互启动独立 Master/bridge/executor/GCS 角色并保存日志路径。
- `wait_for_health(uav_id: str, expected_state: str, timeout_s: float) -> None`：从 ROS 健康 Topic 等待状态，超时即失败。
- `pause_bridge(role: ManagedProcess, duration_s: float) -> None`：向指定 bridge 发送 SIGSTOP，等待指定时长后 SIGCONT；finally 必须恢复或终止进程。
- `restart_bridge(role: ManagedProcess) -> ManagedProcess`：受控终止并重启相同配置，验证 slow-joiner/重连。
- `archive_evidence(case_name: str, metadata: Mapping[str, object]) -> None`：保存命令、阈值、PID、时间线、Action/TCP 结果与日志索引，不把大日志提交进源码仓库。
- `cleanup_all() -> None`：按逆序终止子进程并清理 ROS Master，保证失败测试不遗留暂停进程或端口。

### 修改函数

#### `/home/ub20tg/catkin_swarm6-2/src/tcp_to_ros/scripts/gcs_a_backend_node.py`

- `GcsABackendNode.__init__()`：为所有 enabled 身份订阅 `/{exec_target}/mavros/state`，创建 `/group_a/uav_link_health` publisher、健康 timer 和租约协调器；enabled 只决定可发现/可解析，不代表必须上线。
- `_execute(goal)`：解析 assignments 后先建立任务参与集；要求参与机已稳定 HEALTHY，并在首条/新增成员任务时完成 START，再进入现有 `_run_goal`。健康或 START 门禁失败时不得调用任何 `UavTask`，返回明确失败并安全补偿已 START 成员。
- `_run_goal(goal, state)`：保持现有并发任务确认参数；租约门禁的耗时必须计入 TCP 5 秒 ACCEPTED 总约束，不能把 4 秒任务确认预算再串行叠加 3 秒等待。推荐实际运行前预热健康窗口，goal 到达后只做即时健康检查和并发 START。
- `_wait_for_terminal(state, deadline)`：检查 `state.link_lost_uavs`；参与机 LOST 时设置 `UAV_LINK_LOST`、终结当前整组并进入失败即停。
- `_safe_hold(state, reason, skip)`：链路 LOST 的故障机加入 skip，由租约本地 HOLD；其他已接受且未终止参与机保持现有并发 `UavHold` 和未确认事实记录。
- `_on_mavros_state(msg, uav_id, exec_target)`：只负责记录 callback 接收时刻和 connected，不在 callback 中阻塞调用 Service/HOLD。
- `_evaluate_link_health(event)`：在 timer/工作线程中发布变化、节流日志，并把参与机 LOST 注入活动 GroupState；无活动任务时只更新 ROS 健康与会话报警。
- `_handle_emergency_hold(request)`：继续只作用于活动任务；不替代会话 END，也不因无活动任务而伪造 TCP 状态。
- `shutdown()`：停止租约续期线程/timer；活动飞行会话不得静默 END，应报警并尽力触发已有本地安全路径，随后有界关闭资源。

#### `/home/ub20tg/catkin_swarm6-2/src/swarm_uav_executor/src/swarm_uav_executor/executor.py`

- `handle_task(request)`：真实部署默认要求 ACTIVE 且未过期的匹配 mission 租约；测试兼容模式可由显式参数关闭。EXPIRED_HOLD 锁存时拒绝新任务。
- `handle_hold(request)`：保持跨机人工/补偿 HOLD 幂等；与租约过期入口共享一个内部原子安全停止函数，防止双线程重复切换 driver target。
- `_fail_active_and_hold(error_code, message)`：改为返回结构化结果，并允许无活动任务时仍执行本机 HOLD；当前实现只在 active 存在时调用 `handle_hold`，不足以保护任务间隙悬停。
- 新增 `trigger_local_safety_hold(error_code: str, message: str, latch: bool = True) -> SafetyHoldResult`：租约 watchdog 专用入口；抢占活动任务、阻止新任务、只调用一次 driver HOLD，并锁存结果。
- `shutdown()`：先停止租约接收/watchdog，再处理活动任务和 driver；明确避免 reporter/driver 已关闭后 watchdog 再次触发。

### 移除函数

- 不移除现有 GCS 聚合、TCP 断线、EmergencyHold、TaskStore 或 driver 函数。
- 不以新 watchdog 取代 direct MAVROS driver 的 `health()`；driver health 判断本机运动执行条件，安全租约判断 GCS 反向链路活性，二者职责不同。

[Classes]

### 新类

- `UavLinkMonitor`（`/home/ub20tg/catkin_swarm6-2/src/tcp_to_ros/scripts/tcp_to_ros/link_health.py`）：管理配置全集中各机的心跳观察、稳定窗口和状态迁移；只使用注入的单调时钟判定超时，不直接控制 UAV。
- `LinkHealthTransition`（同文件）：不可变事件，包含 previous/current、uav identity、age 和 reason，供节点发布/聚合。
- `SafetyLeaseCoordinator`（`/home/ub20tg/catkin_swarm6-2/src/tcp_to_ros/scripts/tcp_to_ros/safety_lease.py`）：GCS 单飞行会话应用服务，拥有 epoch/seq、START/RENEW/END 并发调用和迟响应隔离；不解释运动 command。
- `SafetyLeaseWatchdog`（`/home/ub20tg/catkin_swarm6-2/src/swarm_uav_executor/src/swarm_uav_executor/safety_lease.py`）：每机本地租约状态机和 timer，拥有本机租约状态但不拥有跨机路由。
- `LeaseValidationError`（同文件）：携带稳定错误码和 message，无 ROS 副作用。
- `SafetyHoldResult`（executor models 或 executor 模块）：记录 HOLD 是否接管、driver 结果、锁存状态和原活动任务终结情况。
- `ManagedProcess`（故障注入测试脚本）：保存命令、PID/进程组、日志和清理动作，不进入产品安装运行路径。

### 修改类

- `GcsABackendNode`：增加链路观测和租约协调职责，但仍不取得位置控制权，不订阅其他 UAV 数据用于跨机控制。健康状态只影响任务门禁、安全聚合和报警。
- `GroupState`：增加参与机链路故障事实；现有任务键、status_seq 去重、accepted_uavs 和失败即停语义不变。
- `ExecutorConfig`：增加租约配置和 Service 名；构造方式应改用关键字或分组配置，避免继续扩大当前长位置参数列表。
- `UavTaskExecutor`：增加本机安全故障锁和统一安全 HOLD 入口；仍然每实例只控制启动时冻结的本机 identity/driver。

### 移除类

- 本轮不移除任何现有类。
- 不新增中心化“全机控制器”；GCS 的健康监测和租约协调只发送已冻结的单机安全接口，不允许直接连接 MAVROS setpoint。

[Dependencies]

- `swarm_uav_interfaces`：继续只依赖 catkin message generation/runtime；新增 msg/srv 均由原生标量组成，不引入 `mavros_msgs`，保持共享接口纯净。
- `tcp_to_ros`：新增已在工作空间和 executor 中使用的 `mavros_msgs` build/exec 依赖，以订阅 bridge 后的 `mavros_msgs/State`；继续依赖 `rospy`、`swarm_uav_interfaces`、`swarm_topology_bridge`。
- `swarm_uav_executor`：沿用 `rospy`、`mavros_msgs`、`swarm_uav_interfaces`、标准库 `threading/time/uuid`；不新增 pip 包。
- `swarm_topology_bridge`：不增加 `swarm_uav_interfaces` 或 `mavros_msgs` 编译依赖；运行环境安装相同接口包和 MAVROS 消息包即可动态解析。
- 故障注入使用 Python 标准库 `subprocess`、`signal`、`pathlib`，不依赖 `tc netem`、root/sudo、Docker 或第三方 process manager。
- 时间源分离：watchdog 与租约 TTL 必须使用 `time.monotonic()`；ROS 消息 `reported_at` 和仿真动作 deadline 继续使用现有 ROS time 约定。测试通过注入 fake monotonic clock 保证确定性。
- 保持 ROS Noetic、catkin_tools、ZeroMQ、MAVROS 与 PX4 当前版本，不执行全局安装或版本升级。

[Testing]

### 接口、构建与静态验证

1. `catkin build swarm_uav_interfaces swarm_uav_executor tcp_to_ros` 成功。
2. `rosmsg show swarm_uav_interfaces/UavLinkHealth` 与 `rossrv show swarm_uav_interfaces/UavSafetyLease` 字段、顺序和本计划一致。
3. 搜索确认租约类型只在共享接口包定义，bridge 不含 Group A/租约业务判断。
4. `git diff --check` 在涉及的独立仓库均通过；不修改 build/devel/logs 和冻结计划。

### GCS 链路健康单元测试

- 未上电 A04-A15 不生成 LOST，不阻塞只包含 A01-A03 的任务。
- 任意上电顺序下，首次 connected=true 产生 DISCOVERED，连续有效心跳达到稳定窗口后 HEALTHY。
- callback 每次均更新消息接收事实，但 connected=false 不更新最后有效健康时间；持续 connected=false 最终进入 SUSPECT/LOST。
- 1 Hz 下小于 3 秒的短暂缺包不报警；3 秒 SUSPECT；5 秒 LOST；边界使用 fake monotonic clock 精确断言。
- LOST 后一条心跳不得恢复旧任务；健康恢复发布 RECOVERED，再进入 HEALTHY。
- ROS time 回拨、`/use_sim_time` 暂停或消息时间戳异常不影响 watchdog 超时。
- 健康快照 age_s、identity、mission/epoch 和 reason 正确；从未收到时不发布虚假 HEALTHY。

### 机载租约与 HOLD 单元测试

- 正确 START 建立 ACTIVE；1 Hz RENEW 使 5 秒 TTL 不过期。
- 同 epoch/seq/内容重复返回相同结果；同 seq 不同内容冲突；倒退 seq、错误 identity、错误协议、非法 TTL、未知 operation 被稳定拒绝。
- ACTIVE 时不同 epoch START 被拒绝；模拟 GCS 重启产生的新 UUID 不能覆盖飞行中的旧租约。
- 超过 TTL 只触发一次 `trigger_local_safety_hold`；活动 MOVE_TO 被终结为 `FAILED/SAFETY_LEASE_EXPIRED`，旧目标不恢复。
- 无活动任务但仍在飞行会话中，租约过期仍调用 driver HOLD，覆盖任务间隙。
- HOLD 成功后 EXPIRED_HOLD 锁存；链路恢复的旧 RENEW 不解除；旧任务/新任务均按规则拒绝，直到旧会话安全结束并建立新 epoch。
- pose/state stale 导致 HOLD 失败时记录 `LEASE_HOLD_FAILED`，保持故障锁，不虚报安全悬停成功。
- END 在 active task、armed 或 disarmed 稳定窗口不足时拒绝；安全 disarmed 后 END 转 INACTIVE。
- watchdog 与人工 UavHold、motion failure fallback、shutdown 并发时 driver HOLD 最多一次接管，不死锁、不重复终态。

### GCS 会话与聚合单元测试

- 任务参与集只来自 assignments；身份配置 enabled 不等于必须在线。
- 参与机未稳定 HEALTHY 时任务在调用 UavTask 前失败；不得等待未上电飞机直到协议超时。
- 首任务所有成员 START 成功后才调用 UavTask；START 部分失败时不发任何任务，并对已 START 成员执行 END 补偿。
- 同 mission 后续任务复用 epoch；新加入成员单独完成健康与 START；任务终态不 END。
- 每机 RENEW 并发，15 机不会串行放大到 15 秒；旧 worker/迟响应不能覆盖更大 seq。
- 当前参与机 LOST：整组 Action FAILED、`failed_uavs` 准确、`UAV_LINK_LOST` 透传 TCP，并 HOLD 其他活动参与机；故障机不调用跨机 UavHold。
- 会话成员但非当前参与机 LOST：ROS 健康 Topic LOST + 日志，当前 Action 不失败；该机依靠本地租约 HOLD。
- 无活动 command 时 LOST 不产生 TCP STATUS，不复用最近 command_id，不修改已缓存 COMPLETED/FAILED。
- RECOVERED 不复活已失败 Action，不清除 command 缓存；新 command_id 在满足新会话/租约条件后才可执行。

### bridge 与多 Master 冒烟

1. 两个 ROS Master：UAV1 启动 MAVROS state 测试源和 bridge，GCS_A 启动 bridge。
2. UAV1 本地发布 `/mavros/state`，GCS_A 必须看到 `/UAV1/mavros/state`，类型和 connected 值原样保留。
3. GCS_A 调用 `/UAV1/uav_safety_lease`，确认请求经 Service 代理到达 UAV1，START/RENEW/END 响应关联正确。
4. 确认新增 Topic 使用独立端口索引且不限频，未破坏 `uav_task_state`；新增 Service 未破坏现有 task/hold 代理。
5. 若冒烟失败，先判断是配置 namespace 还是 bridge 通用缺陷；仅对通用缺陷修改 bridge 并补通用测试。

### 可复现 Phase 5 故障注入

- **基线**：GCS_A + 至少两个独立 UAV Master + bridge + mock executor；参与机健康、START 成功、1 Hz RENEW、正常任务 COMPLETED。
- **短断**：SIGSTOP UAV1 bridge 小于 3 秒后 SIGCONT；不进入 LOST、不触发 HOLD，任务不重复执行。
- **预警断**：暂停超过 3 秒但小于 5 秒；GCS 发布 SUSPECT，恢复后 RECOVERED/HEALTHY；租约未过期时不 HOLD。
- **长断**：暂停超过 5 秒；GCS 发布 LOST；UAV1 本地租约触发 HOLD；若 UAV1 是当前参与机，Action/TCP FAILED 且其他参与机 HOLD；GCS 不向 UAV1 发跨机 HOLD。
- **非参与机长断**：会话中的 UAV3 不参与当前 A01/A02 任务时断链；A03 本地 HOLD并报警，A01/A02 任务按自身状态继续。
- **重连**：SIGCONT 或重启 bridge；GCS 发布 RECOVERED，旧任务保持 FAILED，旧 epoch/seq 不错误续命，新 command_id 需重新通过门禁。
- **Service 迟响应**：mock 延迟 START/RENEW/任务确认响应；验证绝对 deadline、迟响应隔离、幂等和任务不复活。
- **GCS 进程退出**：停止 GCS_A 或 GCS bridge，所有已建立飞行会话的机载租约在 TTL 后各自 HOLD。
- **机载 bridge 退出**：只影响该机；其他机 RENEW 和健康监测继续，不因线程/Service 阻塞被串行拖垮。
- 每个场景归档阈值、时间线、健康 Topic、UavTaskState、Action/TCP 结果、Service 调用计数、HOLD driver 计数、进程日志；必要时 rosbag 只录控制/状态/健康 Topic，避免无关高频遥测。

### 回归测试

- `catkin run_tests swarm_uav_executor`：原 22 项及新增租约测试全部通过。
- `catkin run_tests tcp_to_ros`：原 7 项及新增健康/租约/聚合测试全部通过。
- 复跑正常整组、重复幂等、单机失败即停、Action cancel、EmergencyHold、TCP 断线 HOLD；新租约不得改变已冻结的任务唯一键、4 秒确认、ALL 聚合和 TCP 三态。
- 三机 PX4 SITL 可在不自动 arm/offboard 前提下复跑；新增租约必须覆盖任务间 HOLD，但本轮不以再次主动飞行为 Phase 5 的唯一通过条件。

### Phase 6 不上桨进入门禁

- 每架实物机验证本地 `/mavros/state` 约 1 Hz、connected 语义及 GCS 侧 `/{exec_target}/mavros/state` 全链路可见。
- 现场复核 A01-A15/hostname/IP/namespace，允许任意测试子集，不把未上电飞机误报 LOST。
- 不上桨验证断开机载 bridge、自组网或 GCS_A 后，5 秒租约到期可触发本机安全路径；记录 HOLD 成功或失败及 PX4 failsafe 实际行为。
- 冻结 armed=false 稳定窗口、操作员显式 END 流程、GCS 重启恢复流程、急停职责和日志/rosbag 保存位置。
- PX4 datalink/offboard/RC/battery failsafe 未现场确认前，不得因为 ROS 租约测试通过而授权上桨。

[Implementation Order]

1. **保存基线和验证 bridge 命名假设**：记录四个独立仓库状态、现有构建/测试结果；用两个 ROS Master 手工桥接一个 `mavros_msgs/State`，确认 GCS 侧名称确为 `/UAV1/mavros/state`。若假设不成立，先修正规划中的配置名，不先改业务代码。
2. **扩展共享接口**：在 `swarm_uav_interfaces` 新增 `UavLinkHealth.msg` 与 `UavSafetyLease.srv`，更新 CMake/README，单独构建并用 rosmsg/rossrv 校验；所有 Master 必须部署同一接口 commit。
3. **实现纯逻辑 GCS 健康状态机**：完成 `UavLinkMonitor`、单调时钟、稳定窗口和 DISCOVERED/HEALTHY/SUSPECT/LOST/RECOVERED 单元测试；此时不触发 Action/HOLD。
4. **实现机载租约状态机**：完成 `SafetyLeaseWatchdog` 的 START/RENEW/END、epoch/seq 幂等、TTL 和 END 安全判据单元测试；先用 fake executor callback 验证过期事件，不连接 bridge。
5. **建立 executor 统一安全 HOLD 入口**：重构 `UavTaskExecutor`，使活动任务和任务间隙均可安全 HOLD，处理并发、锁存和 HOLD_FAILED；复跑原 22 项 executor 测试，确保 direct MAVROS 安全基线不退化。
6. **装配机载租约 Service/watchdog**：更新 node、config 和 mock/MAVROS launch；单 Master 验证 1 Hz RENEW 下不超时、停止 RENEW 后 5 秒 HOLD、恢复不复活旧任务、安全 END 后新 epoch 可 START。
7. **实现 GCS 租约协调器**：完成单会话 UUID、并发 START/RENEW/END、迟响应隔离和部分 START 补偿；用 fake Service 测试 15 机并发不会串行放大。
8. **接入 GCS_A 任务门禁与故障聚合**：订阅桥接 MAVROS state、发布健康 Topic；在 UavTask 前执行即时稳定健康检查和 START；参与机 LOST 注入现有 FAILED/ALL 路径，非参与机仅报警；保证不向 TCP 发送无 command 故障。
9. **更新 Group A 拓扑和 launch**：加入 `/mavros/state` Topic 与每机租约 Service 配置；完成两 Master Topic + Service 双向冒烟，再扩展到 A01-A03 独立 Masters。
10. **补齐自动化回归**：运行三包联合构建、executor/tcp_to_ros 全量测试、单 Master rostest和原正常/幂等/失败/cancel/断线用例；修复问题时保持 GCS、bridge、executor 的既有职责边界。
11. **实现进程级 Phase 5 编排与归档**：使用 SIGSTOP/SIGCONT 和受控 restart 覆盖短断、SUSPECT、LOST、本机 HOLD、整组 FAILED、非参与机隔离、GCS 退出和 RECOVERED 不复活；Service 迟响使用 mock 注入，不引入 tc/netem。
12. **收口 Phase 5 文档证据**：把命令、阈值、日志索引和测试结论追加到测试/工程过程文档，更新后续开发计划和问题历史；只有端到端证据完整后才声明 Phase 5 通信安全闭环完成。
13. **执行 Phase 6 实机不上桨门禁**：逐机验证真实 namespace、connected 心跳、任意子集、租约超时、本机 HOLD/PX4 failsafe 和安全 END；默认不 arm、不切 OFFBOARD，不以 SITL 或软件 HOLD 成功替代现场签字。
14. **后置设计后续动作能力**：Phase 5 收口且 Phase 6 不上桨门禁通过后，再为 EGO Planner、`FOLLOW_ROUTE`、`FAULT_EXIT`、`HOVER`、leader-followers 和机间避碰产出 ADR/时序/故障语义与仿真方案；不得为了后续动作修改本轮冻结的通信安全职责。
15. **按独立仓库提交**：分别在 `swarm_uav_interfaces`、`swarm_uav_executor`、`tcp_to_ros` 以及确有通用修复时的 `swarm_topology_bridge` 提交；提交信息覆盖用户冻结的安全决策与实现/文档改动，遵循英文 Conventional Commit header，不配置 remote、不 push。