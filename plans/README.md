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

- `MOVE_TO`：16m 高层机动，12m 中层完成；
- `FOLLOW_ROUTE`：12m 中层执行；
- `FAULT_EXIT`：8m 低层执行并最终降落。

### 安全基线（不可弃用）

- 默认不自动 arm、不自动切 OFFBOARD；飞行控制由操作员手动执行。
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

> 被审计冻结基准不回填修改；本 README"当前状态"段为唯一实时权威。
>
> `implementation_plan_26081202.md` 为**已实施计划**（2026-08-12）：修复场景 3 测试暴露的 ego
> 状态机缺陷（IDLE 不发布 setpoint、COMPLETED/HOLD 不复位 IDLE、多机坐标系不一致），
> 并配套 GCS_A 全局原点统一。arm+takeoff+OFFBOARD 自动化方案仅调研见附录 A（不实施）。

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
| 6 | 实机联调（不上桨 → 上桨） | 未执行 |

## 当前状态（实时）

- **已落地**：Phase 1-5；四种动作命令已在机载执行层实现；
  `ego_planner_driver` 状态机修复已实施（2026-08-12，计划 `implementation_plan_26081202.md`）：
  IDLE 持续 30Hz 发布 setpoint（每秒刷新 hover 目标）、COMPLETED/HOLD 冻结 2s 自动回 IDLE、
  GCS_A 全局原点统一（`/group_a/gp_origin` 周期下发 + 机载 `set_gp_origin` 起飞前持续设置）、
  最大飞行速度 1.0 m/s、`offboard_takeoff_15.py` 移除 `--hold` 保活逻辑。
- **验证**：ego 单测 + 全量回归 112 tests 0 failures；单 Master 生命周期冒烟
  （IDLE→EXECUTING→COMPLETED→IDLE→HOLD→IDLE，各状态 setpoint 流不断）通过。
- **待验证（由用户确认）**：场景 3 无头 15 机六批次重跑（计划第 10 步，本次未实施）。
- **未完成**：Phase 6 实机联调；ego_planner_driver 在真实 PX4 下的执行稳定性待场景 3 重跑确认。
- **已知问题（测试流程）**：`gcs_a_backend.launch` 需 `catkin build tcp_to_ros` 后才可通过
  `roslaunch tcp_to_ros` 找到；`--hold` 保活分支已从 `offboard_takeoff_15.py` 移除。

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
