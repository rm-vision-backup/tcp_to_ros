# tcp_to_ros 后续开发计划

## Phase 编号约定（唯一权威）

本项目所有文档中的 Phase N 一律以本文件为准：

- Phase 1：接口与映射冻结
- Phase 2：GCS_A 分发与汇总
- Phase 3：`swarm_topology_bridge` 扩展
- Phase 4：机载执行适配层
- Phase 5：仿真联调
- Phase 6：实机联调

其他文档（如 `tcp_to_ros_工程过程.md`、`handoffs/*`）只使用"阶段"或直接引用本文件 Phase 编号，不自行建立并行的 Phase 编号体系。

## 目标

- 完成 Group A 整组批次任务到各无人机的可靠分发与结果汇总；
- 明确 `tcp_to_ros`、`swarm_topology_bridge` 和机载执行层的职责；
- 复用同一套接口支持多 ROS Master 仿真与实机部署。

## 当前范围

本计划只考虑：

- Group A；
- 整组批次拆分；
- GCS 本地整组 Action；
- 跨 ROS Master 的 Service 请求/响应与 Topic 状态回传；
- 单机状态聚合；
- 仿真与实机兼容。

本计划不考虑：

- Group B 或地面站间通信；
- 编队、轨迹规划和运动控制算法；
- 跨机完整 ROS Action 桥接；
- 用 `multimaster_fkie`、multi-agent-suite 或 `nimbro_network` 替换当前 bridge。

## 当前任务模型

Group A 默认包含 6 个整组批次：

1. `prepare-a`
2. `coverage-segment-1`
3. `fault-exit`
4. `recovery-a`
5. `coverage-segment-2`
6. `hold-a-end`

共同规则：

- runtime 发送整组批次命令；
- 每个批次含逐机 `assignments`；
- 当前完成策略为 `ALL`；
- GCS_A 负责拆分、确认、汇总和安全停止；
- `command` 只表示作业类型，不与固定无人机绑定。

## 职责边界

### `tcp_server_node.py`

只负责：

- TCP 接入和 JSON Lines 协议；
- 字段校验、TCP 连接管理和 `command_id` 去重；
- 将整组任务交给 GCS 本地 `ExecuteGroupTask` Action；
- 将 Action feedback/result 转换为 TCP `ACCEPTED/COMPLETED/FAILED`；
- TCP 断线时触发本地整组安全请求。

不得加入逐机网络路由和单机状态聚合。

### GCS_A 分发与汇总层

负责：

- 提供 GCS 本地整组 `ExecuteGroupTask` Action server；
- 根据身份映射把 `assignments` 拆成单机结构化任务；
- 并发调用各 UAV 的跨机任务 Service；
- 订阅单机状态 Topic；
- 按 `ALL` 策略形成 Action feedback/result；
- 任一参与机拒绝、确认超时或链路 LOST 时执行失败即停；
- 监测逐机链路健康并维护飞行安全租约；故障机由本地租约 HOLD，其他可达参与机由 GCS_A 请求 HOLD。

### `swarm_topology_bridge`

负责：

- 保留现有 ZMQ Topic 桥；
- 新增通用 ROS Service 请求/响应代理；
- 根据静态拓扑连接 GCS_A 和 UAV*；
- 不理解任务业务，不执行任务聚合；
- 不桥接完整 ROS Action 的内部 Topic。

### 机载执行适配层

负责：

- 在本机 ROS Master 提供统一单机任务 Service 和安全 HOLD Service；
- 校验任务、执行身份和幂等键；
- 将 `command` 交给本机动作层；
- 通过 Topic 回传执行状态和终态；
- 本地监督飞行安全租约，租约过期时不依赖跨机链路触发并锁存安全 HOLD；
- 不负责跨机通信或整组汇总。

## 跨机通信方案评估

### 单纯 Topic

- 延迟最低，适合异步反馈；
- 当前 bridge 已支持；
- 但单次发布没有业务接收确认，存在启动顺序、断线和 PUB/SUB slow-joiner 丢失风险；
- 不能单独满足 TCP `ACCEPTED` 的严格语义。

### 跨机 Service

- 请求和响应关联明确，适合任务接收与 HOLD 确认；
- Service 响应只表示任务已校验、幂等登记并交给本机执行入口；
- 长任务不能阻塞在 Service 中，后续状态仍需 Topic。

### 跨机 Action

- 原生支持 goal、feedback、result 和 cancel；
- 跨 Master 需要桥接 `goal/cancel/status/feedback/result` 多类内部 Topic；
- 15 架 UAV 会增加配置、连接、恢复和测试复杂度；
- 当前阶段收益不足，不采用。

### 现有多 Master 方案

- `multimaster_fkie` / multi-agent-suite 可以同步远程 Topic 和 Service，但会引入动态发现、主机名解析和 ROS 节点直连，偏离当前静态拓扑方案；
- `nimbro_network` 提供 Topic/Service transport，适合弱网，但会替换或重叠现有 bridge；
- 当前阶段继续扩展 `swarm_topology_bridge`，不引入替代框架。

### 最终结论

采用 **GCS 本地 Action + 跨机 Service + 跨机 Topic**：

- 整组生命周期：本地 `ExecuteGroupTask.action`；
- 单机任务接收：跨机 Service；
- 单机 HOLD：跨机 Service；
- 执行反馈和终态：跨机 Topic；
- 不扩展完整跨机 Action。

## Phase 1：接口与映射冻结

### 全结构化单机任务

不得使用 `payload_json`。复用现有 `Pose3DYaw` 和 `TaskAssignment` 的结构化能力，并冻结单机请求字段：

- `protocol_version`
- `mission_id`
- `group_id`
- `command_id`
- `uav_id`
- `exec_target`
- `command`
- `timeout_s`
- `leader_id`
- `assignment`

任务唯一键固定为：

```text
(mission_id, command_id, uav_id)
```

单机任务 Service 响应至少包含：

- `accepted`
- `status`
- `error_code`
- `message`

其中 `accepted=true` 表示请求已通过校验、完成幂等登记，并成功交给本机执行入口，不表示任务已经完成。

### 单机状态消息

冻结字段：

- `protocol_version`
- `mission_id`
- `command_id`
- `uav_id`
- `exec_target`
- `command`
- `status`
- `status_seq`
- `detail_stage`
- `error_code`
- `message`
- `reported_at`

主状态固定为：

- `ACCEPTED`
- `COMPLETED`
- `FAILED`

`COMPLETED` 和 `FAILED` 是终态；状态允许重复发布，GCS_A 按任务唯一键和 `status_seq` 去重。

### HOLD 请求

HOLD 使用独立跨机 Service，请求至少包含：

- `protocol_version`
- `mission_id`
- `command_id`
- `uav_id`
- `exec_target`
- `reason`

响应包含 `accepted/error_code/message`。稳定悬停完成后通过单机状态 Topic 报告终态。

### 身份映射固化

任务层使用 `A01 ... A15`，执行层使用 `UAV1 ... UAV15` / hostname。映射必须存入版本化 YAML，至少包含：

- `uav_id`
- `exec_target`
- `hostname`
- `ip`
- `enabled`

首版按已知规则固化：

- `A01 -> UAV1 -> 192.168.5.11`
- 依次递增；
- `A15 -> UAV15 -> 192.168.5.151`

正式实机使用前仍需人工复核映射和 IP。

## Phase 2：GCS_A 分发与汇总

### 接收确认时序

TCP 要求 5 秒内收到整组 `ACCEPTED`。GCS_A 必须：

- 对所有参与 UAV 并发调用任务 Service，不得串行；
- 单次等待上限 `1.0 s`；
- 最多调用 3 次（首次 + 2 次重试）；
- 重试间隔 `0.2 s`；
- 最迟在 `4.0 s` 内形成整组 `ACCEPTED` 或 `FAILED`；
- 仅当全部参与 UAV 均返回 `accepted=true` 后，向本地 Action 发布 `ACCEPTED` feedback。

### 可靠性语义

采用 **至少一次请求 + 幂等执行**：

- GCS_A 可以因超时重发同一任务；
- UAV 根据任务唯一键拒绝重复执行；
- 重复请求返回当前状态或缓存终态；
- Service 明确拒绝或重试耗尽视为该 UAV 接收失败。

### `ALL` 聚合与失败即停

- 全部参与 UAV 完成才返回整组 `COMPLETED`；
- 任一 UAV 返回 `FAILED`，整组立即 `FAILED`；
- 任一 UAV 拒绝任务或确认超时，整组立即 `FAILED`；
- 整组失败时，对已接受且尚未终止的参与 UAV 并发请求 HOLD；
- 已完成和失败列表可同时存在，用于保留现场事实；TCP 最终状态仍为整组 `FAILED`。

### 超时与取消

- `timeout_s` 是整组任务预算，从 GCS_A 接收整组 Action goal 时开始计时；
- 超时或 Action cancel 时，对活动参与 UAV 请求 HOLD；
- HOLD 请求本身需要逐机接收确认；
- 未确认 HOLD 的 UAV 必须记录到错误详情，不得把整组 HOLD 报为成功。

## Phase 3：`swarm_topology_bridge` 扩展

### Topic 桥保留与修正

- 保留 ROS 消息原生序列化与来源命名空间；
- 任务状态 Topic 不得被普通遥测限频策略误丢；
- 修正启动窗口首条消息可能被限频丢弃的问题；
- 对控制/状态 Topic 支持独立限频或不限频配置。

### 通用 Service 代理

在现有 ZMQ 静态拓扑中新增请求/响应通道：

- 配置声明 Service 名称、类型、目标节点和超时；
- 每次请求携带 bridge 层唯一 request ID；
- 请求端关联响应并向本地 ROS Service client 返回；
- 服务端调用本地 ROS Service 后发送响应；
- 支持并发请求、超时和迟到响应丢弃；
- bridge 只代理 ROS 序列化请求/响应，不理解任务业务；
- 业务重试和幂等由 GCS_A 与机载适配层负责。

### Group A 拓扑

新增 GCS_A 与 UAV* 的实机和仿真配置：

- `GCS_A` 接收所有 UAV 的状态 Topic；
- 每个 UAV 接收 GCS_A 的 Service 请求；
- 任务 Service 和 HOLD Service 双向请求/响应可达；
- 实机使用物理 IP；
- 单机多 Master 仿真使用 `127.0.0.1 + port_offset`；
- GCS_A、UAV* 必须安装并 source 同版本消息定义。

## Phase 4-6 后续路线

### Phase 4：机载执行适配层

状态：**首版代码、自动化测试、单 Master ROS 闭环及三机 PX4 SITL 主动飞行验证已完成；实机不上桨与上桨现场验收仍待完成。**

- 已将共享接口一次性迁移至独立纯接口包 `swarm_uav_interfaces`；
- 已实现独立单机包 `swarm_uav_executor`：任务/HOLD Service、幂等缓存、状态 Topic、身份隔离、单活动任务及 HOLD 抢占；
- 已提供 `MockMotionDriver` 与 direct `MavrosPositionDriver`；direct driver 默认不 arm、不切 OFFBOARD；
- 已用单元测试、单 Master rostest 和三机 PX4 SITL 验证 `MOVE_TO + HOLD`；SITL 中验证了持续 position setpoint、OFFBOARD `MOVE_TO` 到达与 HOLD 捕获；
- 自动 arm/自动切 OFFBOARD 仍按设计禁止；真实机体坐标系、yaw、阈值、namespace 与 PX4 failsafe 尚未现场冻结，因此未取得实机上桨许可。

### Phase 5：仿真联调

状态：**mock 跨 Master 通信安全闭环、既有三机 PX4 SITL 主动飞行，以及真实 MAVROS/PX4 租约断链与安全 END 门禁验证均已通过。**

- 已用新 executor 替换旧 `tcp_to_ros/uav_mock_node.py`，单 Master 正常、重复幂等、单机失败即停回归通过；
- GCS_A 已按参与 UAV/exec_target 聚合状态，并按 `status_seq` 丢弃重复与乱序状态；
- 共享绝对 deadline 保证整组确认不按 UAV 数量串行放大；迟到 Service 响应不能把已超时尝试复活；
- Action cancel 与 TCP 连接关闭均进入 HOLD 补偿路径，未确认 HOLD 会保留错误事实；
- 三套独立 UAV ROS Master + bridge + direct MAVROS executor 已完成三机 `MOVE_TO` 与 HOLD 的 PX4 SITL 主动飞行；
- 已在隔离的双/三 Master mock 环境验证 MAVROS state Topic 与安全租约 Service 代理、`HEALTHY → SUSPECT → LOST`、参与机 LOST 整组失败、本地/远程 HOLD 分工及安全恢复；
- 安全 END 后只解锁未来任务，旧 epoch 续期被拒，旧任务保持失败终态，新 epoch 与新任务可重新接受；
- 已在使用真实 MAVROS + PX4 SITL（11300 仿真、A01/UAV1）的单机链路验证状态断流、Service 断链、租约过期、本机 HOLD、链路恢复与安全 END 门禁（2026-08-10，详见测试记录）。剩余对 PX4 自身 datalink/offboard/RC failsafe 的现场验证仍在 Phase 6 实机无桨门禁范围内。

### Phase 6：实机联调

- 在不上桨条件下冻结并验证真实机载的身份/IP/namespace、坐标系/yaw、阈值、链路心跳、租约断联与 PX4 failsafe；
- 验证 PX4 状态新鲜、connected、稳定 disarmed 的安全 END 门禁，以及 GCS/bridge 退出后的本机安全路径；
- 完成现场安全职责、急停和场地许可签字后，才允许评估实机上桨；
- 后续接入 EGO Planner、航线、编队与机间避碰，并逐步验证 `FOLLOW_ROUTE/FAULT_EXIT/HOVER` 闭环。

当前仅实现 `MOVE_TO + HOLD`。`FOLLOW_ROUTE/FAULT_EXIT/HOVER`、EGO Planner、编队及机间避碰均为后续范围。

## 机载动作层语义参考

这些语义不属于分发层实现：

- `MOVE_TO`：16m 高层机动，12m 中层完成；
- `FOLLOW_ROUTE`：12m 中层执行；
- `FAULT_EXIT`：8m 低层执行并最终降落。

## 尚待确认

1. GCS_A 最终实际 IP、hostname 和部署位置；
2. A01-A15 映射表的现场人工复核结果；
3. `detail_stage` 与 `error_code` 的完整枚举；
4. bridge Service 代理的端口分配和并发上限；
5. GCS_A/UAV 重启后是否需要跨进程持久化活动任务与终态；
6. 机载断联时 PX4/MAVROS 的独立 failsafe 参数及现场验证结果；
7. 实机不上桨联调的现场负责人、急停职责、验收阈值与证据归档位置；
8. MAVROS/PX4 租约断链测试采用无头 SITL 还是由操作员启动图形化 SITL，以及对应日志/rosbag 归档位置；
9. EGO Planner、`FOLLOW_ROUTE`、leader-followers 编队和机间避碰的正式设计与仿真验收方案。

## `implementation_plan_26080804.md` 完成度审计（2026-08-08）

该实施计划为冻结基准，不回填修改。按其 `[Implementation Order]` 的 12 个步骤审计当前证据：

| 步骤 | 状态 | 当前证据与剩余缺口 |
|---|---|---|
| 1 建立基线 | 完成 | Phase 1-3 构建、单 Master 回归和跨 Master Service 冒烟已有记录。 |
| 2 接口独立仓库 | 完成 | `swarm_uav_interfaces` 已独立建仓，3 msg + 2 srv、README、构建与类型检查完成。 |
| 3 一次性类型迁移 | 完成 | 消费方已切换到新类型，旧共享定义和运行时 import 已删除。 |
| 4 executor 骨架 | 完成 | models、validation、TaskStore、StateReporter、MotionDriver 与包骨架已落地。 |
| 5 mock driver | 完成 | 非阻塞接收、身份隔离、幂等/冲突、busy、timeout、状态序号和 HOLD 抢占已自动化验证。 |
| 6 替换旧 mock | 完成 | integration launch 已使用新 executor；旧 `uav_mock_node.py` 已删除，等价回归通过。 |
| 7 跨 Master Phase 5 | 部分完成 | 进程内状态过滤、绝对 deadline、cancel/断线 HOLD 已自动化验证；仍缺完整 bridge 网络故障注入和端到端证据归档。 |
| 8 direct MAVROS | 完成 | fake-callback 单测与三机 PX4 SITL 主动飞行均已通过；默认不自动 arm/mode。 |
| 9 不上桨安全联调 | 未完成 | fake callback 与 SITL 不能替代真实硬件不上桨、断联和 PX4 failsafe 现场验证。 |
| 10 文档与证据 | 完成 | CONTEXT、Phase 状态、过程、测试证据、未决问题与交接均已收口；弱网/不上桨本身仍分别属于步骤 7/9。 |
| 11 独立提交 | 完成 | 两个新仓库与消费方已有独立 Conventional Commits；新仓库无 remote、未 push。 |
| 12 后续能力设计 | 未完成 | 仅保留稳定扩展边界，尚无 EGO/航线/编队/避碰的正式设计与仿真测试产物。 |

因此可声明 **Phase 4 首版及 PX4 SITL 主动飞行验证完成**；不可声明 **GCS_A + 机载完整弱网仿真最终验收完成**，也不可声明 **实机不上桨或上桨验收完成**。

## `implementation_plan_26080900.md` 完成度审计（2026-08-09）

该冻结计划已移至 `plans/`。本轮已完成共享安全接口、GCS_A 链路健康与租约协调、executor 本地 watchdog/锁存 HOLD、Group A Topic/Service 拓扑、自动化回归，以及 mock 双/三 Master 的断链与恢复闭环验证。

当前结论：

- 可声明 **mock 环境下 Phase 5 跨 Master 通信安全闭环完成**；
- 已使用真实 MAVROS + PX4 SITL（11300 仿真、单机 A01/UAV1）验证状态断流、Service 断链、租约过期、本机 HOLD、链路恢复与安全 END 门禁（2026-08-10，详见测试记录）；
- 尚未执行 Phase 6 实机不上桨门禁；软件 HOLD 不替代 PX4 datalink/offboard/RC failsafe；
- 在上述门禁完成前继续保持默认不自动 arm、不自动切 OFFBOARD，不得据此授予实机上桨许可。
