# 15 机 PX4/Gazebo 起飞与批次任务复测记录

时间：2026-08-14 06:36 CST  
记录人：____________  
测试环境：PX4 SITL + Gazebo GUI + MAVROS + ego_planner_driver + swarm_uav_executor + swarm_topology_bridge + GCS_A + TCP runtime

> 精简测试记录：每一步只记实际结果；未实测不写 PASS。
> 步骤序列由 git 历史写回（精简前 545 行操作手册，终端 A-G），结果沿用当日实测结论；仅检查的步骤简写预期。

## 0. 测试目标与批次

目标链路：

```text
runtime TCP 39001 -> tcp_server_node -> GCS_A backend -> swarm_topology_bridge
  -> swarm_uav_executor -> ego_planner_driver -> MAVROS/PX4 SITL
```

| 批次 | 命令 | 参与机 | 状态 |
|---|---|---|---|
| prepare | `MOVE_TO` | A01-A12 | FAIL（见 §3 根因） |
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
| prepare | FAIL（见 §3） |
| 其余 5 批 | 未执行（prepare 未通过，不继续批次） |

## 2. 测试步骤（步骤-预期-结果）

端口约定：仿真 Master `11300`、GCS_A Master `11310`、UAVn Master `11310+n`、Gazebo `11345`、TCP `39001`。

| # | 【步骤】 | 【预期】 | 【结果】 |
|---|---|---|---|
| 1 | 构建：`cd /home/ub20tg/catkin_swarm6-2 && source /opt/ros/noetic/setup.bash && catkin build --no-status && source devel/setup.bash`（本次为 ego_planner_driver 增量） | 构建成功、无 failed/error | PASS |
| 2 | ROS/PX4 环境：各终端 `source devel/setup.bash`、`source /home/ub20tg/PX4_Firmware/Tools/setup_gazebo.bash /home/ub20tg/PX4_Firmware /home/ub20tg/PX4_Firmware/build/px4_sitl_default`、追加 `ROS_PACKAGE_PATH`、设 `ROS_HOME`/`ROS_LOG_DIR` | `rospack find safe_valley_exp`、`rospack find tcp_to_ros` 成功 | PASS |
| 3 | 启动仿真层（终端 A）：`export ROS_MASTER_URI=http://localhost:11300; export GAZEBO_MASTER_URI=http://localhost:11345; roslaunch safe_valley_exp multi_uav_ego_15sim.launch gui:=true` | Gazebo GUI 打开、15 架 iris 生成、QGC 可见 15 系统且全部 disarm、无遗留任务/failsafe | PASS（15 机仿真层在线） |
| 4 | 启动 15 机载层（终端 B）：`bash src/safe_valley_exp/startup_offboard_ego.sh 1 2 … 15`（每机独立 Master 11311..11325：MAVROS + ego driver + executor + bridge，不 arm） | `/tmp/UAV1_offboard_ego.pid`..`UAV15` 存在、15 Master 在线、MAVROS connected | PASS（15/15 ready，MAVROS connected，exec_state=HOLD，setpoint 30Hz） |
| 5 | 机载只读检查：逐机 `rostopic echo -n 1 /mavros/state`、`/mavros/local_position/pose`、`/exec_state`；`rosservice list \| grep -E 'uav_task\|uav_task_control\|uav_hold\|uav_safety_lease'`（UAV1 端口 11311，其余 11312..11325 逐机记录） | `connected=true`、`armed=false`、`exec_state` 正常、位姿持续更新、setpoint≈30Hz、服务齐全 | PASS（15/15 状态/位姿/服务正常；此时 lease 尚未删除，`uav_safety_lease` 服务在列） |
| 6 | 启动 GCS bridge（终端 C，Master 11310）：`roslaunch tcp_to_ros topology_group_a_sim.launch uav_name:=GCS_A run_uav_mock:=false` | 15 机 topic/service 代理（含 `uav_task`、`uav_hold`、`uav_safety_lease`）可见，不启动 mock | PASS |
| 7 | 启动 GCS backend + 标定（终端 D）：`roslaunch tcp_to_ros gcs_a_backend.launch` | A01-A06 采样约 2s、全部 disarm、航向门禁通过；A01-A15 origin 回读确认；日志出现 `Group A calibration ready: 15/15 origins confirmed` | PASS（标定 ready 15/15；须 15 机全部落地 disarm 后才完成） |
| 8 | 启动 TCP server（终端 E）：`rosrun tcp_to_ros tcp_server_node.py _host:=0.0.0.0 _port:=39001 _action_name:=/group_a/execute_task _state_topic:=/group_a/group_task_state _emergency_hold_service:=/group_a/emergency_hold` | 39001 监听；Action/状态 Topic/EmergencyHold 可见；不发送任务 | PASS（39001 LISTENING） |
| 9 | 起飞前门禁（操作员确认）：QGC 15 机在线、`calibration_ready=true` 且 origin confirmed 15/15、`exec_state` 待命、setpoint≈30Hz、`COM_RCL_EXCEPT=4`、`NAV_RCL_ACT=0` | 所有门禁通过，方可起飞 | PASS（calibration ready 15/15，setpoint 30Hz，实际 exec_state=HOLD，RC 参数已配置） |
| 10 | 逐机 arm + OFFBOARD（终端 F）：`python3 src/safe_valley_exp/scripts/offboard_takeoff_15.py 1 2 … 15`（脚本不调 MAVROS takeoff，ego driver 执行软起飞） | 输出成功 15/15、QGC 全部 `armed=true / mode=OFFBOARD`、软起飞至 z≈5m、无 RTL/failsafe | PASS（15/15，软起飞至 z≈5m，exec_state=HOLD，无 disarm，无 failsafe） |
| 11 | 批次 1 prepare（终端 G）：`python3 references/runtime/task_test.py --backend tcp --group A --task prepare --tcp-host 127.0.0.1 --tcp-port 39001 --timeout-s <TIMEOUT_S>` | 退出码 0、`status=SUCCESS`、`failed_robot_ids=[]`、按需起飞到准备点 | FAIL（见 §3 根因） |
| 12 | 批次 2-6：coverage-segment-1 / fault-exit / recovery / coverage-segment-2 / hold-end | 上一批 `SUCCESS` 后逐批执行 | 未执行（批次继续条件未满足：prepare FAIL，不继续批次） |
| 13 | 清理：确认飞机稳定后，stop TCP server → GCS backend/标定/bridge → `bash src/safe_valley_exp/startup_offboard_ego.sh stop 1 2 … 15` → Gazebo/PX4 | 无残留进程、无占用端口 | PASS（prepare FAIL 后停止批次结束测试；会话结束时仿真进程全部停止，残留 0） |

## 3. prepare 失败过程与根因

prepare 多次尝试均失败，根因集中在**飞行安全租约（lease）机制**：

1. 首次：backend `begin_session`（lease START）失败 → `LEASE_START_FAILED`，任务从未下发给 driver；runtime 表现为 TIMEOUT（未收到 ACCEPTED/FAILED 反馈）。
2. executor 残留 ACTIVE lease（START 部分成功后未 END；armed 悬停下 END 被拒 `LEASE_END_UNSAFE`）→ 后续所有 START 返回 `LEASE_EPOCH_CONFLICT`。
3. 重启 executor 清 lease 后：backend renew 失败（`LEASE_CALL_TIMEOUT`，deadline 0.9s 过紧 + 高负载并发）→ lease TTL 过期 → executor 本地 watchdog 锁存 HOLD（`LEASE_EXPIRED` latched）→ 之后所有任务被拒，latch 无法解除（只能重启 executor）。

**结论**：lease 机制时延敏感（renew deadline 0.9s）、失败后锁存恢复困难，是本轮仿真测试通过的瓶颈；且在自组网电台（200-800ms 抖动）实机环境下必然失败。

**决定采用方案 A：删除 lease 层**，改由 backend 已有 `link_health` + `safe_hold` 作为"节点丢失 → 停止 → 全悬停 → 手动接管"的唯一机制。实施计划见 `plans/implementation_plan_26081406.md`（待实施）。

## 4. 测试中发现的环境约束

- 标定（`set_gp_origin`）必须在 UAV 落地 disarm 时进行；armed 时 `gp_origin_receiver` 拒绝修改原点，标定永远无法 ready。
- executor 节点在 launch 中 `required=true`，kill executor 会连带关闭整个机载层（MAVROS/driver/bridge），导致 offboard loss → LAND。
- backend 重复启动会导致 `gcs_a_calibration` 重复注册冲突，必须确保单实例。

## 5. 未完成状态

- prepare 未通过 → coverage-segment-1 / fault-exit / recovery / coverage-segment-2 / hold-end 全部 pending。
- 待方案 A（删除 lease）实施后：重启机载层 + backend（disarm 下标定）→ 起飞 → 重跑 6 批次。

## 6. 安全边界

- `POSE_STALE` 只代表规划输入过期，不等于飞机停止、不等于安全悬停。
- ego_planner_driver 执行中可能继续发布已生成轨迹；位姿失联、setpoint 中断、OFFBOARD 丢失行为还要结合 PX4 failsafe 实测。
- 本文 SITL 结果不等于实机上桨许可。
- 实机前必须另行冻结：`COM_OF_LOSS_T`、`COM_OBL_ACT`、RC/datalink failsafe、模式切换、急停职责、场地许可。
- 任何异常优先人工 HOLD/降落/急停；不要用宽泛清理命令误杀用户仿真。
