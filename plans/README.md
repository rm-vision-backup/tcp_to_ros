# plans 目录（入口）

> 本目录是 `tcp_to_ros` 项目的开发入口：维护 **Phase 编号约定**、**全局设计规约**、
> **冻结实施计划**与**开发里程碑**。新会话从此文件开始阅读。
>
> 场景操作手册：`docs/test_record_26081021.md`（场景 1/2/2+）与
> `docs/test_record_26081123.md`（场景 3 无头 15 机）。
> 测试证据按期归档：`docs/test_record_2608xxxx.md`（含早期 `docs/tcp_to_ros_测试记录_早期.md`）。

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

经研讨比较单纯 Topic / 跨机 Service / 跨机 Action / 现有多 Master 框架后冻结：

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

- `MOVE_TO`：首次按需起飞先完成相对 5m 检查点，再垂直到 15m、EGO 水平、垂直到目标 z；
- `FOLLOW_ROUTE`：leader 在 12m 执行 EGO，followers 在 12m 使用 PI+固定编队偏置；
- `FAULT_EXIT`：先垂直到 8m，再使用 EGO 水平离队。

### 安全基线（不可弃用）

- arm/OFFBOARD 软件所有权只属于本机 executor；仅未 arm 的 `MOVE_TO` 在整批 PREPARE/START 门禁后按需起飞。
- GCS_A 在任务前冻结 A01-A12 全球位置中位数与集群参考航向，A01-A15 原点回读全部确认前拒绝任务。
- 严格垂直段冻结 x/y/yaw；运行时水平中心距小于 1.0m 且垂直中心距小于 2.0m 时整批 HOLD。
- 软件 HOLD 不替代 PX4 datalink/offboard/RC failsafe；现场门禁完成前不得上桨。
- 无 RC 的 PX4 SITL 需设 `COM_RCL_EXCEPT=4` + `NAV_RCL_ACT=0`，并**先切 OFFBOARD 再 arm**。

## 实施计划（历史 + 进行中）

| 文件 | 内容 | 状态 |
|---|---|---|
| `implementation_plan_26080804.md` | Phase 4：共享接口迁移 + executor + `MOVE_TO/HOLD`（mock/direct MAVROS） | 已实施 |
| `implementation_plan_26080900.md` | Phase 5：跨 Master 链路健康 + 飞行安全租约 + 故障注入 | 已实施 |
| `implementation_plan_26081003.md` | P1：ego_planner_driver 轨迹内核 + 机间避碰集成 | 已实施 |
| `implementation_plan_26081023.md` | 四种动作命令（MOVE_TO/FOLLOW_ROUTE/FAULT_EXIT/HOVER）补齐 | 已实施 |
| `implementation_plan_26081202.md` | ego_planner_driver 状态机修复 + IDLE setpoint 流 + GCS_A 全局原点统一 | **已实施** |
| `implementation_plan_26081223.md` | 冻结集群坐标、固定编队、两阶段启动、按需起飞与分层轨迹 | **已实施** |
| `implementation_plan_26081406.md` | 删除 safety lease 租约机制，`link_health` + `safe_hold` 替代 | **已实施** |
| `implementation_plan_26082300.md` | 方案 B：机载分层航点 0.4m 密化 + C++ `planning_horizon` 滚动局部规划（对齐原版 ego-planner-swarm） | **部分实施（核心代码与 prepare 已完成；避碰及其余批次未验收）** |
| `implementation_plan_26082322.md` | 周期性零速修复：EGO/follower 分源 `PositionTarget` + relay 唯一出口、仲裁与失效 HOLD | **已实施（2026-08-24，含 prepare 无头验收）** |
| `implementation_plan_26082415.md` | min-snap 统一单段重构：轨迹平滑（`one_segment_traj_gen`）+ 双机不安全时本机 HOLD 暂停（备用多段 minSnapTraj 仅记录、不实施） | **待实施** |

> 被审计冻结基准不回填修改；本 README"当前状态"段为唯一实时权威。
>
> `implementation_plan_26081202.md` 为**已实施计划**（2026-08-12）：修复场景 3 测试暴露的 ego
> 状态机缺陷（IDLE 不发布 setpoint、COMPLETED/HOLD 不复位 IDLE、多机坐标系不一致），
> 并配套了当时的固定原点方案。该固定原点和旧自动 TAKEOFF 语义已由
> `implementation_plan_26081223.md` 的冻结标定与 executor 按需起飞取代。

## 开发里程碑

| Phase | 里程碑 | 证据 |
|---|---|---|
| 1 | 接口与映射冻结（msg/srv/YAML） | `docs/tcp_to_ros_测试记录_早期.md` |
| 2 | GCS_A 分发与汇总（Action + 聚合 + 失败即停 + HOLD） | `docs/tcp_to_ros_测试记录_早期.md` |
| 3 | bridge 扩展（Topic 限频修正 + 通用 Service 代理） | `docs/tcp_to_ros_测试记录_早期.md` |
| 4 | executor 首版 + 三机 PX4 SITL 主动飞行（2026-08-08） | `docs/tcp_to_ros_测试记录_早期.md` |
| 5 | 跨 Master 通信安全闭环（mock 断链 2026-08-09；真实 MAVROS/PX4 租约断链 + 安全 END 2026-08-10） | `docs/tcp_to_ros_测试记录_早期.md` |
| 5 | 四种动作命令实现 + 场景 2+ 15 mock 验收（2026-08-11 15:00） | `docs/test_record_26081115.md` |
| 5 | 场景 3 无头 15 机多 Master 端到端（2026-08-11 23:58） | `docs/test_record_26081123.md` |
| 5 | ego_planner_driver 状态机修复（2026-08-12 已实施：IDLE setpoint 流 + terminal→IDLE 自动回退 + GCS_A 全局原点统一） | `plans/implementation_plan_26081202.md` |
| 5 | 冻结坐标与固定编队、PREPARE/START、按需起飞、15/12/8m 分层动作（2026-08-13） | `plans/implementation_plan_26081223.md`、`docs/test_record_26081300.md` |
| 6 | 实机联调（不上桨 → 上桨） | 未执行 |

## 当前状态（实时）

- **已落地**：A01-A12 全员原点/航向门禁、A01-A15 原点回读确认、任务右-前-上变换、固定编队
  1/2 精确集合、内部 PREPARE/START/ABORT、executor 按需起飞、15/12/8m 分层动作、followers PI、
  EGO 硬 clearance 与垂直/PI 实时距离门禁。
- **起飞所有权**：`safe_valley_exp/scripts/offboard_takeoff_15.py` 仅做只读诊断；正式 arm、OFFBOARD、
  相对 5m 检查点和稳定 2s 都由 executor 在未 arm `MOVE_TO` 的 START 后执行。
- **坐标冻结**：A01-A12 连续采样 2s 后逐机求全球位置中位数和圆周平均航向，任一缺失、已 arm 或
  航向偏差超过 30° 均拒绝；冻结值在 GCS_A 进程内不变。15/15 原点回读达到水平/高度 0.1m 才就绪。
- **验证**：2026-08-13 工作空间 `catkin build` 6/6 成功，`catkin test`/`catkin_test_results --all build`
  合计 133 tests、0 errors、0 failures；包含 executor ROS Service/Topic rostest。
- **场景 3 批 1 结论（2026-08-12 晚）**：`prepare` FAILURE（5/12）且全机未动。
  根因=GCS_A backend `_execute()` 的 fail-fast 预检（`UAV_LINK_UNHEALTHY`）：任务前多机
  MAVROS heartbeat 曾 SUSPECT/LOST，`is_stably_healthy` 不满足 → 整组 set_aborted，
  **从未调用任何 `/UAV{n}/uav_task`、未触 HOLD**；executor 无任务记录、exec_state 均回 IDLE。
- **链路健康修复（2026-08-12 晚）**：① link_health 新增 `is_healthy_or_undiscovered`，
  UNDISCOVERED 不阻任务；② backend 预检替换为该方法，并新增前置等待（最多 10s，每秒重评）；
  ③ 放宽 heartbeat 阈值（suspect 3→10s、lost 5→15s）以便容忍 bridge 建链初期抖动；
  ④ `ego_swarm.state_timeout_s` 从 5s 放宽到 200s 避免长轨迹规划被 driver 误判超时。
- **未完成**：15 机 PX4 SITL 六批次端到端重跑；Phase 6 实机不上桨及上桨联调。
- **已知问题（测试流程）**：`gcs_a_backend.launch` 需 `catkin build tcp_to_ros` 后
  `roslaunch tcp_to_ros` 才能找到；跨 Master 建链初期多机 heartbeat 抖动易触发 fail-fast。
- **lease 删除决策（2026-08-14）**：`prepare` 多次失败根因集中在 safety lease
  （`LEASE_START_FAILED` / `LEASE_EPOCH_CONFLICT` / `LEASE_EXPIRED` latch）；lease 时延敏感
  （renew deadline 0.9s）且实机自组网电台（200-800ms 抖动）下必然失败，决定按
  `implementation_plan_26081406.md` 删除 lease 层，由 `link_health` + `safe_hold` 承担
  “节点丢失 → 停止 → 全悬停 → 手动接管”。
- **lease 删除已实施（2026-08-14）**：机载 `safety_lease.py`、watchdog 线程、lease 服务与
  `lease_guard` 门禁、driver `can_end_safety_lease`、GCS `SafetyLeaseCoordinator`、桥接拓扑
  `uav_safety_lease` 条目、验收 launch 参数与 verification 验证脚本全部移除；
  `UavSafetyLease.srv` 保留为 dead definition。`catkin build` 3/3 通过；GCS 22 项单元测试全过，
  机载 51 项中 49 过 + 2 项 pre-existing FakeRos 环境失败（baseline 复现）。待无头仿真重跑六批次。
- **方案 B 已实施部分（2026-08-23）**：`implementation_plan_26082300.md` 的核心代码与补充修复已落地：
  `EgoSwarmDriver` 生成含 start 的 15/12/8m 分层关键点并按最大 0.4m 间距密化；C++ 移除全局密化，
  使用 5m `planning_horizon`、周期重规划和按实际位置推进的滚动窗口；滚动段继承 vel/acc，最终到达
  发布 `COMPLETED`，位置容差调整为 0.6m；邻机 swarm rebound 硬推已接入。相关 Python/C++ 用例已补齐，
  当次记录为 6 包构建成功、133 tests、0 errors、0 failures（详见 `docs/test_record_26082300.md`）。
- **方案 B prepare 验收结果**：无头 15 机环境中 A01-A12 `prepare` 已 `SUCCESS`，12 机全部
  `COMPLETED`；完成 15m→水平→12m 分层，终点偏差 0.2-0.3m，速度峰值 1.05-1.12m/s。
- **方案 B 未完成部分**：prepare 实测机间最小距离曾为 0.64m，未达到 ≥1.0m 安全门禁；
  `coverage-segment-1`、`fault-exit`、`recovery`、`coverage-segment-2`、`hold-end` 尚未执行，
  Phase 6 实机验收也未执行。当前 `ego_planner_driver` 工作树另有未提交的速度平滑改动，不能视为
  已归档实施结果；周期性速度降零问题转由 `implementation_plan_26082322.md` 继续处理。
- **26082322 已实施（2026-08-24）**：EGO 改发 P/V/A/yaw `PositionTarget`（mask 2048）、follower
  改发显式 position/velocity/yaw 候选（mask 2496）、`setpoint_relay` 独占
  `/mavros/setpoint_raw/local` 并实现选源/generation 隔离/freshness/finite 校验/固定位置 HOLD，
  `replan_period_s` 恢复为 1.0 s；旧 `/setpoint` PoseStamped 链路与旁路发布已清理。详见
  `docs/test_record_26082400.md`。
- **min-snap 统一单段方案已登记（2026-08-24 15:52，`implementation_plan_26082415.md`，待实施）**：
  成功判据=单机飞行平滑（速度无剧烈震荡）+ 双机不安全时 HOLD 暂停；本次不实施机间避碰行为，
  未完成任务即 HOLD 作为避碰失败兜底，也不实施原版 2.1 完整安全机制。数据链：上层 keypoints
  密化 0.4m→**2m**（仅作窗口推进/末点参考，不进 B-spline）；C++ `planOnce` 引入原版 min-snap
  单段 `one_segment_traj_gen`（起点=旧轨迹 P/V/A、end_vel=切线巡航、仅最终段静止、梯形 T）→
  0.4m 弧长等距采样 point_set → `parameterizeToBspline` → L-BFGS（jerk + feasibility 软惩罚）；
  停用 rebound 硬推与时间重参数化（新增 `enable_rebound`/`enable_time_reparam` 默认 false）；
  执行层 EGO 巡航距离门禁扩展 `_distance_safe`（复用 `neighbor_odom_topics`，水平 <1.0m 且垂直
  <2.0m → 本机 HOLD 暂停）；`replan_period_s` 保持 1.0、PositionTarget 契约与 relay 唯一出口不回归。
  备用多段 `minSnapTraj` 方案已写入计划（含触发条件与闭式解实现），**仅记录不实施**——它是单段
  无法处理时的修改方案，非弓字 keypoints 后续功能；当前单段方案覆盖 MOVE_TO 与 FOLLOW_ROUTE。


## 文档索引

| 路径 | 内容 |
|---|---|
| `plans/README.md`（本文件） | Phase 编号、设计规约、计划索引、里程碑、当前状态 |
| `plans/implementation_plan_*.md` | 实施计划（历史冻结 + 进行中） |
| `CONTEXT.md` | 领域词汇表 |
| `docs/tcp_to_ros_工程过程.md` | 开发日志（含各阶段 bug 修复记录） |
| `docs/tcp_to_ros_测试记录_早期.md` | 早期 mock 闭环测试证据（已冻结，不再更新） |
| `docs/test_record_26081021.md` | 场景 1/2/2+ 操作手册 + 历史测试结果（场景 3 已迁移） |
| `docs/test_record_26081115.md` | 四种动作命令实现验证（mock + 单元级） |
| `docs/test_record_26081123.md` | 场景 3 无头 15 机多 Master 端到端测试报告 |
