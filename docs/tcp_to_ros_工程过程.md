# tcp_to_ros 工程过程

## 目标

在 `src/tcp_to_ros` 新建 Python ROS1 Noetic 功能包，作为 Group A 地面站主包，对接 `references/runtime` 的 TCP client，完成协议适配与整组任务分发验证。

> Phase 编号约定：本文档不自行定义 Phase 编号；文中"对应 Phase N"一律指 `tcp_to_ros_后续开发计划.md` 的 Phase N（1 接口与映射冻结、2 GCS_A 分发与汇总、3 bridge 扩展、4 机载执行适配层、5 仿真联调、6 实机联调）。

## 阶段一至四：第一版 mock 闭环（已完成）

- **阶段一 资料与现状摸底**：阅读 `容错自愈联合任务方案`、`实物无人机上下层TCP-IP接口对接方案`、runtime `tcp_transport.py`；检查 `safe_valley_exp` 与 `swarm_topology_bridge`。结论：现有包以 topic 为主，无现成整组任务 Action 接口，需自定义。
- **阶段二 接口与边界**：包名 `tcp_to_ros`；ROS 主接口采用 Action + Topic + Service；只做数据适配；mock backend 自动回复；验收目标为跑通 A 组六任务。
- **阶段三 落地**：`package.xml`、`CMakeLists.txt`、`setup.py`、`action/ExecuteGroupTask.action`、`msg/`、`srv/EmergencyHold.srv`、`scripts/tcp_server_node.py`、`scripts/mock_backend_node.py`、`scripts/task_event_injector.py`、`launch/tcp_to_ros_mock.launch`、`config/mock_backend.yaml`。
- **阶段四 构建与 mock 联调**：`catkin build`；`rosmsg/rossrv` 生成验证；A 组六任务正向通过；负路径（重复 `command_id`、错误 `group_id`、HOLD 打断活动任务）通过。

## 阶段五：Phase 1-3 前置方案冻结

完成后续真实分发链路技术研讨并冻结方案（对应后续开发计划 Phase 1/2/3）：

- GCS 本地整组生命周期使用 `ExecuteGroupTask` Action；任务接收与 HOLD 用跨机 Service，执行反馈与终态用跨机 Topic；
- `swarm_topology_bridge` 增加通用 Service 代理、保留 Topic 桥、不扩展完整 Action；
- 全结构化字段、唯一键 `(mission_id, command_id, uav_id)`、至少一次请求 + UAV 幂等执行 + `ALL` 失败即停；
- 并发确认参数：单次 1.0 秒、首次 + 2 次重试、间隔 0.2 秒、最迟 4 秒形成整组 `ACCEPTED/FAILED`；整组 `ACCEPTED` 仅在全部 `accepted=true` 后发送。

详细依据见 `tcp_to_ros_后续开发计划.md`。

## 阶段六，对应 Phase 1：接口与映射冻结

- 新增 `msg/UavTaskState.msg`（单机状态）、`srv/UavTask.srv`（单机任务）、`srv/UavHold.srv`（单机 HOLD）；
- 新增 `config/uav_identity_map.yaml`（A01-A15 → UAV1-UAV15 / hostname / IP，标注待复核）；
- `UavTask.srv` 复用 `TaskAssignment`/`Pose3DYaw`，全结构化字段，无 `payload_json`；
- `rosmsg/rossrv` 生成验证通过。

## 阶段七，对应 Phase 2：GCS_A 分发与汇总

- `scripts/tcp_to_ros/identity.py`（身份映射加载）；
- `scripts/gcs_a_backend_node.py`（本地 `ExecuteGroupTask` Action server：并发 Service 确认、4 秒确认预算、状态 Topic 聚合、幂等键、超时/cancel/HOLD、失败即停）；
- `scripts/uav_mock_node.py`（单机 mock 执行适配层：幂等缓存、状态重发、HOLD 安全停止）；
- launch/config：`gcs_a_backend.launch`、`gcs_a_backend_test.launch`、`gcs_a_backend_fail.launch`、`uav_mock_config.yaml`；
- 集成测试客户端 `test/test_gcs_a_backend_client.py`。

## 阶段八，对应 Phase 3：bridge 扩展与 Group A 拓扑

- 重写 `swarm_topology_bridge/scripts/bridge_node.py`：修正 Topic 限频（启动窗口首条不丢弃、逐 Topic 不限频），新增 ZMQ DEALER/ROUTER 通用 Service 请求/响应代理（唯一 request ID、并发、超时、迟到响应丢弃）；
- 通用化：bridge 不依赖任何业务包，按配置动态解析 Topic/Service 类型；修正 `roslib` Service 类型解析与 `genpy` 载荷反序列化；
- Group A 联调示例 `config/topology_group_a_sim.yaml` 与 `launch/topology_group_a_sim.launch` 归属 `tcp_to_ros` 地面站主包；
- 依赖方向：`swarm_topology_bridge` 为通用包；`tcp_to_ros`、`safe_valley_exp` 依赖 `swarm_topology_bridge`。

## 阶段九：集成验证

- 单 Master 语义联调（`gcs_a_backend_test.launch` + 测试客户端）：正常整组 `COMPLETED`；重复 `command_id` 返回缓存 `COMPLETED`（幂等）；UAV2 失败 → 整组 `FAILED`、错误码透传、活动 UAV 被 HOLD 确认；
- 双 Master 跨机 Service（GCS_A:11311 + UAV1:11312，经 `topology_group_a_sim.launch`）：GCS_A 侧注册 6 个跨机 Service 代理，`rosservice call /UAV1/uav_task` 转发到 UAV1 mock 返回 `accepted: True`；
- 待覆盖：状态 Topic 跨 Master 完整聚合闭环、4 秒确认边界端到端、cancel/TCP 断线 HOLD、弱网/重连下的超时与迟响应。

## 阶段十：主包回归验证与仓库结构调整（本会话）

- **回归验证**（`tcp_to_ros` 主包已实现功能实测）：
  - 正常整组 `MOVE_TO` → `COMPLETED`（completed_uavs=A01,A02,A03）；
  - 重复 `command_id` → 仍缓存 `COMPLETED`（幂等）；
  - 整组失败即停 → `FAILED` / failed_uavs=A02 / error=`MOCK_UAV2_FAIL`，活动 UAV 被 HOLD 确认。
- **仓库结构调整**：
  - `safe_valley_exp` 从本仓 submodule 中剔除，作为独立功能包维护；其内部以 `swarm_topology_bridge` **master** 分支子模块方式登记桥接依赖；
  - README/TODO 按中文为主重命名（`README.md`/`TODO.md` 中文、`README-en.md`/`TODO-en.md` 英文）；
  - 用 `git filter-branch` 清除了本仓历史中对 `src/safe_valley_exp` 的全部跟踪（gitlink 与 `.gitmodules` 声明），历史已重写。

## 构建与运行约定

- 只使用 `catkin build`，默认 `build/devel/logs`；基础环境 `source ~/.bashrc`，构建后额外 `source /home/ub20tg/catkin_swarm6-2/devel/setup.bash`；
- 当前 Agent 环境对 `~/.ros` 写入有限制，联调使用工作区内 `ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home`、`ROS_LOG_DIR=.../.ros_home/log`；
- 详细验证结果见 `tcp_to_ros_测试记录.md`。

## 阶段十一，对应 Phase 4/5：共享接口与单机执行器首版

- 共享 `Pose3DYaw/TaskAssignment/UavTaskState/UavTask/UavHold` 原字段原序迁移到独立纯接口仓库 `swarm_uav_interfaces`；`tcp_to_ros` 只保留本地 Action、整组状态与 EmergencyHold；
- 新建独立 `swarm_uav_executor`，每实例绑定 `(uav_id, exec_target)`，实现 TaskStore、状态报告、单活动任务、HOLD 抢占、mock 与 direct MAVROS driver；
- 新 executor mock 通过等价回归后，删除旧 `scripts/uav_mock_node.py`，避免 GCS 包与机载包并存两套执行语义；
- GCS_A 补齐参与机/exec_target/status_seq 状态过滤、共享绝对确认 deadline、迟响应隔离、Action cancel 与 TCP 断线 HOLD 补偿；
- 自动化证据：executor 22/22、tcp_to_ros 7/7，通过三包联合构建；PX4 SITL、真实弱网和不上桨现场记录仍待外部环境。