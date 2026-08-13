# 15 机 PX4/Gazebo 起飞与批次任务复测记录

时间：2026-08-14 06:36 CST  
记录人：____________  
测试环境：PX4 SITL + Gazebo GUI + MAVROS + ego_planner_driver + swarm_uav_executor + swarm_topology_bridge + GCS_A + TCP runtime

> 精简测试记录：每一步只记实际结果；未实测不写 PASS。

## 0. 测试目标与批次

目标链路：

```text
runtime TCP 39001 -> tcp_server_node -> GCS_A backend -> swarm_topology_bridge
  -> swarm_uav_executor -> ego_planner_driver -> MAVROS/PX4 SITL
```

| 批次 | 命令 | 参与机 | 状态 |
|---|---|---|---|
| prepare | `MOVE_TO` | A01-A12 | FAIL（见 §2 根因） |
| coverage-segment-1 | `FOLLOW_ROUTE` | A01-A12 | 未执行（pending） |
| fault-exit | `FAULT_EXIT` | A07-A12 | 未执行（pending） |
| recovery | `MOVE_TO` | A01-A06、A13-A15 | 未执行（pending） |
| coverage-segment-2 | `FOLLOW_ROUTE` | A01-A06、A13-A15 | 未执行（pending） |
| hold-end | `HOVER` | A01-A06、A13-A15 | 未执行（pending） |

批次继续条件：上一批 `SUCCESS`、参与机状态正常、无 failsafe。

## 1. 执行摘要

| 步骤 | 结果 |
|---|---|
| 构建（ego_planner_driver 增量） | PASS |
| 15 机仿真层（PX4 SITL + Gazebo） | PASS |
| 15 机机载层 | PASS（15/15 ready，MAVROS connected，exec_state=HOLD，setpoint 30Hz） |
| GCS_A bridge/backend/TCP | PASS（39001 LISTENING，标定 ready 15/15） |
| 标定（origin + 15/15 回读确认） | PASS（需 15 机全部落地 disarm 后完成） |
| 起飞 arm+OFFBOARD | PASS（15/15，软起飞至 z≈5m，exec_state=HOLD，无 disarm） |
| prepare | FAIL（见 §2） |
| 其余 5 批 | 未执行（prepare 未通过，不继续批次） |

## 2. prepare 失败过程与根因

prepare 多次尝试均失败，根因集中在**飞行安全租约（lease）机制**：

1. 首次：backend `begin_session`（lease START）失败 → `LEASE_START_FAILED`，任务从未下发给 driver；runtime 表现为 TIMEOUT（未收到 ACCEPTED/FAILED 反馈）。
2. executor 残留 ACTIVE lease（START 部分成功后未 END；armed 悬停下 END 被拒 `LEASE_END_UNSAFE`）→ 后续所有 START 返回 `LEASE_EPOCH_CONFLICT`。
3. 重启 executor 清 lease 后：backend renew 失败（`LEASE_CALL_TIMEOUT`，deadline 0.9s 过紧 + 高负载并发）→ lease TTL 过期 → executor 本地 watchdog 锁存 HOLD（`LEASE_EXPIRED` latched）→ 之后所有任务被拒，latch 无法解除（只能重启 executor）。

**结论**：lease 机制时延敏感（renew deadline 0.9s）、失败后锁存恢复困难，是本轮仿真测试通过的瓶颈；且在自组网电台（200-800ms 抖动）实机环境下必然失败。

**决定采用方案 A：删除 lease 层**，改由 backend 已有 `link_health` + `safe_hold` 作为"节点丢失 → 停止 → 全悬停 → 手动接管"的唯一机制。实施计划见 `plans/implementation_plan_26081406.md`（待实施）。

## 3. 测试中发现的环境约束

- 标定（`set_gp_origin`）必须在 UAV 落地 disarm 时进行；armed 时 `gp_origin_receiver` 拒绝修改原点，标定永远无法 ready。
- executor 节点在 launch 中 `required=true`，kill executor 会连带关闭整个机载层（MAVROS/driver/bridge），导致 offboard loss → LAND。
- backend 重复启动会导致 `gcs_a_calibration` 重复注册冲突，必须确保单实例。

## 4. 未完成状态

- prepare 未通过 → coverage-segment-1 / fault-exit / recovery / coverage-segment-2 / hold-end 全部 pending。
- 待方案 A（删除 lease）实施后：重启机载层 + backend（disarm 下标定）→ 起飞 → 重跑 6 批次。

## 5. 安全边界

- `POSE_STALE` 只代表规划输入过期，不等于飞机停止、不等于安全悬停。
- ego_planner_driver 执行中可能继续发布已生成轨迹；位姿失联、setpoint 中断、OFFBOARD 丢失行为还要结合 PX4 failsafe 实测。
- 本文 SITL 结果不等于实机上桨许可。
- 实机前必须另行冻结：`COM_OF_LOSS_T`、`COM_OBL_ACT`、RC/datalink failsafe、模式切换、急停职责、场地许可。
- 任何异常优先人工 HOLD/降落/急停；不要用宽泛清理命令误杀用户仿真。
