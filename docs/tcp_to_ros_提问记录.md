# tcp_to_ros 提问记录

本文件只保留问题与结论的历史记录，不重复承载方案细节。Phase 编号与接口/聚合/超时约束以 `tcp_to_ros_后续开发计划.md` 为唯一权威。

## 已确认问题

1. 包结构：第一版单包先行；包名 `tcp_to_ros`（语义 `tcp-to-ros`）。
2. ROS 接口：主接口 Action + Topic（持续状态） + Service（紧急悬停）。
3. 适配边界：`tcp_to_ros` 只做地面站接入、分发与聚合，不实现真实控制；机载统一执行由独立 `swarm_uav_executor` 通过可替换 driver 接入。
4. 第一版验收：先跑通 A 组六任务，由 mock 自动回复验证通信与语义对接。
5. mock 方式：参数化自动回复，覆盖成功、失败、超时场景。

## 当前假设

- Group A 地面站只服务本组 runtime；
- 默认本机或局域网可达，不在代码写死 IP；
- Group A 与 Group B 地面站暂不通信；
- 共享接口已抽离到 `swarm_uav_interfaces`；首版机载执行器只实现 `MOVE_TO + HOLD`，未来能力不得反向污染 GCS 或 bridge。

## 跨机通信方案研讨结论（摘要）

经研讨确认：只靠单向 Topic 无法满足 TCP `ACCEPTED` 的严格语义（无请求响应、无应用层确认和重发保证）。

最终确定 **GCS 本地 Action + 跨机 Service + 跨机 Topic**：

- 整组生命周期：本地 `ExecuteGroupTask.action`；
- 任务接收与 HOLD：跨机 Service；
- 执行反馈与终态：跨机 Topic；
- 不扩展完整跨机 Action；`swarm_topology_bridge` 增加通用 Service 代理。

方案的详细比较、冻结接口字段、聚合与超时约束见 `tcp_to_ros_后续开发计划.md`，不再在此重复。

## 通信安全闭环研讨结论（2026-08-09）

- 链路健康复用桥接后的 MAVROS state，上行心跳只证明最近一次观测链路可达，不保证后续反向 HOLD 可达；
- GCS_A 负责健康门禁、参与机 LOST 的整组失败及其他可达参与机 HOLD，故障机由本地飞行安全租约 watchdog 接管；
- 配置启用不等于必须上电，任务参与集只来自当前 `assignments`，飞行会话集可跨同 mission 的多个任务批次；
- 租约过期与旧 epoch 恢复均锁存，只有安全 END 才解锁未来任务；旧任务终态不得复活；
- mock 多 Master 闭环已经完成，但不替代 MAVROS/PX4 租约断链或 Phase 6 实机不上桨门禁。

## 后续待确认事项

1. GCS_A 最终实际 IP、hostname 和部署位置；
2. A01-A15 映射表的现场复核结果；
3. `detail_stage` 和 `error_code` 的完整枚举；
4. bridge Service 代理的端口分配和并发上限；
5. GCS_A/UAV 重启后是否持久化活动任务与终态；
6. 真实机载动作层对稳定悬停、航线完成和受控退出的判据；
7. PX4/MAVROS 在机载断联时的独立 failsafe 参数及现场验证结果；
8. MAVROS/PX4 租约断链使用无头 SITL 还是操作员启动图形化 SITL，以及原始日志/rosbag 归档位置；
9. 实机不上桨现场负责人、急停职责、坐标系/yaw/阈值与 MAVROS namespace 冻结结果；
10. EGO Planner、航线、leader-followers 编队和机间避碰的 driver/coordinator/provider 边界与仿真验收方案。
