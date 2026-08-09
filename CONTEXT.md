# Context Glossary

> 本文件是项目领域词汇表，只解释概念、不承载文件路径、实现算法与验证参数。
> - 方案与接口/聚合/超时约束：`docs/tcp_to_ros_后续开发计划.md`（Phase 编号唯一权威）
> - 进度与已落地内容：`docs/tcp_to_ros_工程过程.md`
> - 验证结果：`docs/tcp_to_ros_测试记录.md`
> - 问题与结论历史：`docs/tcp_to_ros_提问记录.md`
> - 冻结实施计划：`plans/*`
> - 会话交接参数：`handoffs/*`
> - 包关系：`tcp_to_ros` 为 GCS_A 地面站业务包；`swarm_uav_interfaces` 为 GCS 与机载共同依赖的纯 ROS 接口包；`swarm_uav_executor` 为每架 UAV 本机部署的执行适配包；三者是相互独立的 Git 仓库。`swarm_topology_bridge` 是不解释 command 的通用通信包。工作空间根目录不拥有这些仓库，也不将两个新包登记为 submodule。

## Group A

本轮开发只指 A 组地面站与 A 组无人机任务链路，不包含 Group B。

## 整组批次任务

指 runtime 发给地面站的一次 Group A 批次命令。当前默认抽象为 6 个批次：`prepare-a`、`coverage-segment-1`、`fault-exit`、`recovery-a`、`coverage-segment-2`、`hold-a-end`。

整组批次任务不是逐机独立长会话。

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

指 GCS_A 本地的整组 `ExecuteGroupTask` Action server（`gcs_a_backend_node.py`）。按身份映射拆分单机任务，并发确认并汇总单机状态，执行 `ALL` 失败即停与 HOLD；具体参数见后续开发计划 Phase 2。

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
