# tcp_to_ros 工程过程

> 本文是开发日志（chronological journal）：记录每个阶段的做了什么、发现并修复的 bug。
> Phase 编号约定与全局设计规约见 `plans/README.md`（唯一权威）。
> 测试证据按日期归档在 `docs/test_record_2608xxxx.md` 与 `docs/tcp_to_ros_测试记录_早期.md`。

## 阶段一至四：第一版 mock 闭环（已完成）

- 资料摸底：阅读 `references/docs` 方案与 runtime `tcp_transport.py`；确认现有包无整组任务 Action 接口。
- 落地：`tcp_to_ros` 包 + `ExecuteGroupTask.action` + `GroupTaskState.msg` + `EmergencyHold.srv` + mock backend；跑通 A 组六任务正向与负路径（重复 `command_id`、错误 `group_id`、HOLD 打断）。

## 阶段五：Phase 1-3 前置方案冻结

冻结方案：**GCS 本地 Action + 跨机 Service + 跨机 Topic**；`swarm_topology_bridge` 增加通用 Service 代理；全结构化字段、唯一键 `(mission_id, command_id, uav_id)`、至少一次请求 + 幂等 + `ALL` 失败即停；并发确认 1.0s/3 次重试/0.2s 间隔/4.0s 整组 deadline。

## 阶段六，对应 Phase 1：接口与映射冻结

- 新增 `UavTaskState.msg`、`UavTask.srv`、`UavHold.srv`；`uav_identity_map.yaml`（A01-A15→UAV1-UAV15）。
- 全结构化字段，无 `payload_json`；`rosmsg/rossrv` 生成验证通过。

## 阶段七，对应 Phase 2：GCS_A 分发与汇总

- `gcs_a_backend_node.py`（并发确认、4s 预算、状态聚合、幂等、超时/cancel/HOLD、失败即停）+ `identity.py` + 集成测试客户端。

## 阶段八，对应 Phase 3：bridge 扩展与 Group A 拓扑

- 重写 `bridge_node.py`：修正 Topic 限频（首条不丢弃、逐 Topic 不限频），新增 ZMQ DEALER/ROUTER 通用 Service 代理（唯一 request ID、并发、超时、迟到响应丢弃）。
- bridge 不依赖业务包，动态解析类型；依赖方向：`swarm_topology_bridge` 通用，`tcp_to_ros`/`safe_valley_exp` 依赖它。

## 阶段九：集成验证

- 单 Master 语义联调（正常/幂等/失败即停+HOLD）与双 Master 跨机 Service 冒烟均通过。

## 阶段十：主包回归验证与仓库结构调整

- 回归：正常整组 COMPLETED、重复 command_id 幂等、失败即停 + 活动 UAV HOLD 确认。
- 仓库：`safe_valley_exp` 从本仓 submodule 剔除为独立包；`git filter-branch` 清除历史跟踪。

## 构建与运行约定

- 只使用 `catkin build`，默认 `build/devel/logs`；构建后 `source /home/ub20tg/catkin_swarm6-2/devel/setup.bash`。
- Agent 环境对 `~/.ros` 写入有限制，联调使用 `ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home`、`ROS_LOG_DIR=.../.ros_home/log`。
- 详细验证结果见 `tcp_to_ros_测试记录_早期.md`。

## 阶段十一，对应 Phase 4/5：共享接口与单机执行器首版

- 共享接口迁移到独立 `swarm_uav_interfaces`；新建 `swarm_uav_executor`（TaskStore、状态报告、单活动任务、HOLD 抢占、mock 与 direct MAVROS driver）。
- 等价回归后删除旧 `scripts/uav_mock_node.py`；GCS_A 补齐状态过滤、共享绝对 deadline、迟响应隔离、取消/断线 HOLD 补偿。
- 证据：executor 22/22、tcp_to_ros 7/7；三机 PX4 SITL 主动飞行验证 direct MAVROS `MOVE_TO + HOLD`。

## 阶段十二，对应 Phase 4/5：三机 PX4 SITL 主动飞行与计划审计

- GCS_A 与 UAV1..3 分属四个 ROS Master，三套 bridge + direct `MavrosPositionDriver` 接入 `iris_0..2`；操作员手动 arm/切 OFFBOARD。
- A01/A02/A03 并发 `MOVE_TO` 均 COMPLETED（实测位置接近目标）；HOLD 5s 位移 0.099/0.105/0.121 m < 0.5m 阈值。

## 阶段十三，对应 Phase 5：跨 Master 链路健康与飞行安全租约（2026-08-09）

- 新增 `UavLinkHealth`/`UavSafetyLease`；GCS_A 单调时钟维护 `DISCOVERED/HEALTHY/SUSPECT/LOST/RECOVERED`；executor 租约 watchdog 与过期本地 HOLD 锁存。
- **修复 bug**：进程级断链注入发现安全 END 后 executor 本地锁存没有释放路径；修复为仅 END 安全判据通过后解锁未来任务，旧任务不重启；END 回调失败返回 `LEASE_END_FAILED` 保持原状态（提交 `a486a8d`）。

## 阶段十四，对应 Phase 5：真实 MAVROS/PX4 租约断链与安全 END 门禁（2026-08-10）

- UAV1（11312）对接 `iris_0`；经 GCS 整组 Action 下发 `MOVE_TO(0,1,10)` 返回 COMPLETED，实测 `(0.019, 1.033, 10.011)`。
- 故障注入（停 UAV1 bridge）：`HEALTHY -> SUSPECT(3.066s) -> LOST(5.070s)`；`safety lease expired and local HOLD latched`；本机 HOLD 30Hz 持续发布未失控；重启 bridge 恢复 HEALTHY。
- 锁存验证：EXPIRED_HOLD 下 RENEW/START/END 全部被拒；安全 END 全套通过，旧任务 `FAILED/LEASE_EXPIRED` 不复活。
- 边界：PX4 自身 datalink/offboard/RC failsafe 仍属 Phase 6 实机无桨门禁。

## 阶段十五，对应 Phase 5：场景 3 无头 15 机多 Master 端到端（2026-08-11）

- 15 机无头 SITL（11300）+ 机载层（11311-11325）+ GCS_A（11310）+ TCP 39001 全链路建立；15/15 MAVROS、15/15 bridge 代理、15/15 OFFBOARD+armed。
- 无 RC SITL 起飞关键知识：需 `COM_RCL_EXCEPT=4`+`NAV_RCL_ACT=0`，且先切 OFFBOARD 再 arm；新增 `verification/ego_smoke/offboard_takeoff_15.py`。
- 六批次应答：链路层贯通但 ego driver 执行层不稳（5-11/12 机成功，根因 `/exec_state` 卡 HOLD 不复位 IDLE）。
- 详见 `docs/test_record_26081123.md`。