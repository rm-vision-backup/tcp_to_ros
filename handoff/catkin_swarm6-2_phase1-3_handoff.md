# catkin_swarm6-2 Phase 1-3 交接

> 交接日期：Phase 1/2/3 已实施并通过集成验证。本文件为下一会话交接参数；
> 领域术语见 `CONTEXT.md`，Phase 编号与接口/聚合/超时约束以 `docs/tcp_to_ros_后续开发计划.md` 为唯一权威。

## 当前状态

- Phase 1（接口与映射冻结）、Phase 2（GCS_A 分发与汇总）、Phase 3（bridge 扩展与 Group A 拓扑）已实施；
- 单 Master 语义联调通过：正常整组 `COMPLETED`、重复 `command_id` 幂等、失败即停 + HOLD 确认；
- 双 Master 跨机 Service 冒烟通过：GCS_A `rosservice call /UAV1/uav_task` → UAV1 mock 返回 `accepted: True`；
- 详细验证结果见 `docs/tcp_to_ros_测试记录.md`。

## 已冻结决策（摘要）

- 只考虑 Group A；整组生命周期用本地 `ExecuteGroupTask.action`；
- 任务接收与 HOLD 用跨机 Service，执行反馈与终态用跨机 Topic；
- `swarm_topology_bridge` 为通用包（不依赖业务包），增加通用 Service 代理、保留 Topic 桥、不扩展完整 Action；
- 单机任务全结构化字段（无 `payload_json`）；唯一键 `(mission_id, command_id, uav_id)`；
- 至少一次请求 + UAV 幂等执行 + `ALL` 失败即停；
- 并发确认：单次 1.0 秒、首次 + 2 次重试、间隔 0.2 秒、最迟 4 秒形成整组 `ACCEPTED/FAILED`；整组 `ACCEPTED` 仅在全部 `accepted=true` 后发送。

## 当前代码事实

- `tcp_to_ros`：地面站主包，依赖 `swarm_topology_bridge`；已含 `gcs_a_backend_node.py`、`uav_mock_node.py`、`topology_group_a_sim.*` 联调示例；
- `swarm_topology_bridge`：通用包，`bridge_node.py` 含 Topic 桥（限频已修正）与 Service 代理；
- `safe_valley_exp`：机载应用示例，依赖 `swarm_topology_bridge`，未参与分发层实现。

## 下一步顺序

1. 完善 Phase 5 仿真联调：状态 Topic 跨 Master 完整聚合闭环、4 秒确认边界端到端、cancel/TCP 断线 HOLD、弱网/重连下的超时与迟响应；
2. 待覆现场确认：实机 IP/hostname、A01-A15 映射复核（当前为待复核配置）；
3. 后续路线见 `docs/tcp_to_ros_后续开发计划.md` 的 Phase 4-6。

## Suggested skills

- `grill-with-docs`：维护领域语言、接口语义与项目文档一致性；
- `diagnose`：弱网、超时、重连或并发问题的运行时取证；
- `TRAE-code-review`：审查接口稳定性、线程安全、幂等与 HOLD 安全路径。

## 构建约束

- 只使用 `catkin build`，默认 `build/devel/logs`；
- 基础环境 `source ~/.bashrc`，构建后额外 `source /home/ub20tg/catkin_swarm6-2/devel/setup.bash`；
- 不提交或修改与任务无关的文件。