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

### 2.1 ROS/PX4 环境

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source /home/ub20tg/PX4_Firmware/Tools/setup_gazebo.bash \
  /home/ub20tg/PX4_Firmware \
  /home/ub20tg/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:/home/ub20tg/PX4_Firmware:/home/ub20tg/PX4_Firmware/Tools/sitl_gazebo"
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
mkdir -p "$ROS_LOG_DIR"
```

作用：加载 ROS、catkin、PX4/Gazebo、工作区日志路径。

【预期】`rospack find safe_valley_exp`、`rospack find tcp_to_ros` 均成功。

【结果】PASS

### 2.2 启动仿真层

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source /home/ub20tg/PX4_Firmware/Tools/setup_gazebo.bash \
  /home/ub20tg/PX4_Firmware \
  /home/ub20tg/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:/home/ub20tg/PX4_Firmware:/home/ub20tg/PX4_Firmware/Tools/sitl_gazebo"
export ROS_MASTER_URI=http://localhost:11300
export ROS_HOSTNAME=localhost
export GAZEBO_MASTER_URI=http://localhost:11345
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
mkdir -p "$ROS_LOG_DIR"
roslaunch safe_valley_exp multi_uav_ego_15sim.launch gui:=true
```

作用：启动 Gazebo 与 15 个 PX4 SITL（iris_0..iris_14 -> UAV1..UAV15 -> A01..A15）。

【预期】Gazebo GUI 打开、15 架 iris 生成、QGC 可见 15 系统且全部 disarm、无遗留任务/failsafe。

【结果】PASS（15 机仿真层在线）

### 2.3 启动 15 机载层（含机载只读检查）

```bash
cd /home/ub20tg/catkin_swarm6-2
bash src/safe_valley_exp/startup_offboard_ego.sh 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
```

作用：每机启动独立 ROS Master（UAVn -> 11310+n）、MAVROS、ego driver、executor、bridge；不 arm。

【预期】`/tmp/UAV1_offboard_ego.pid` 到 `/tmp/UAV15_offboard_ego.pid` 存在；15 个 Master 在线；MAVROS connected；全部 disarm。

【结果】PASS（15/15 ready，MAVROS connected，exec_state=HOLD，setpoint 30Hz）

机载只读检查（简单检查步骤，UAV1 示例；其他机把端口 `11311` 换成 `11312..11325`，逐机记录，不能拿 UAV1 代替全部机）：

```bash
export ROS_MASTER_URI=http://localhost:11311
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/local_position/pose
rostopic echo -n 1 /exec_state
rosservice list | grep -E 'uav_task|uav_task_control|uav_hold|uav_safety_lease'
```

【预期】`connected=true`、`armed=false`、`exec_state` 正常、位姿持续更新、setpoint≈30Hz、服务齐全。

【结果】PASS（15/15 状态/位姿/服务正常；此时 lease 尚未删除，`uav_safety_lease` 服务在列）

### 2.4 启动 GCS bridge

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11310
export ROS_HOSTNAME=localhost
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
roslaunch tcp_to_ros topology_group_a_sim.launch uav_name:=GCS_A run_uav_mock:=false
```

作用：注册跨 Master topic/service 代理（Topic 桥 + Service 代理）；不启动 mock。

【预期】GCS_A 可见 15 机状态、位姿、origin 及 `uav_task`、`uav_task_control`、`uav_hold` 服务代理。

【结果】PASS

### 2.5 启动 GCS backend + 标定（含起飞前门禁检查）

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11310
export ROS_HOSTNAME=localhost
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
roslaunch tcp_to_ros gcs_a_backend.launch
```

作用：启动 GCS_A 分发后端与全局原点/航向标定（`gcs_a_calibration` 单实例，禁止重复启动）。

【预期】A01-A06 采样连续约 2s、全部 disarm、航向门禁通过；A01-A15 origin 回读确认；日志出现 `Group A calibration ready: 15/15 origins confirmed`。

【结果】PASS（标定 ready 15/15；须 15 机全部落地 disarm 后才完成）

起飞前门禁检查（简单检查步骤）：

```bash
# QGC 15 机在线、位置正常、全部 disarm
# calibration_ready=true、origin confirmed 15/15
# setpoint≈30Hz；确认无 RC SITL 参数已配置：COM_RCL_EXCEPT=4、NAV_RCL_ACT=0
```

【预期】所有门禁通过，方可起飞。

【结果】PASS（calibration ready 15/15，setpoint 30Hz，实际 exec_state=HOLD，RC 参数已配置）

### 2.6 启动 TCP server

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11310
export ROS_HOSTNAME=localhost
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
rosrun tcp_to_ros tcp_server_node.py _host:=0.0.0.0 _port:=39001 \
  _action_name:=/group_a/execute_task \
  _state_topic:=/group_a/group_task_state \
  _emergency_hold_service:=/group_a/emergency_hold
```

作用：开放 runtime TCP 入口（39001）。

【预期】39001 监听；Action、状态 Topic、EmergencyHold 可见；不发送任务。

【结果】PASS（39001 LISTENING）

### 2.7 逐机 takeoff

```bash
cd /home/ub20tg/catkin_swarm6-2
python3 src/safe_valley_exp/scripts/offboard_takeoff_15.py 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
```

作用：脚本逐机调用 arm、再请求 OFFBOARD；ego driver 负责软起飞与状态转换（不调用 MAVROS takeoff）。

【预期】输出成功 15/15；QGC 全部 `armed=true / mode=OFFBOARD`；软起飞至 z≈5m；无 RTL/failsafe。

【结果】PASS（15/15，软起飞至 z≈5m，exec_state=HOLD，无 disarm，无 failsafe）

### 2.8 任务逐批次测试

命令模板：

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 references/runtime/task_test.py --backend tcp --group A --task <TASK> \
  --tcp-host 127.0.0.1 --tcp-port 39001 --timeout-s <TIMEOUT_S>
```

作用：经 TCP 发送一个完整 Group A 批次，等待整批终态。每批结束确认整批终态、逐机终态、位置、速度、mode、setpoint；runtime 打印的 JSON 需保存完整输出。

【预期】正常批次退出码 0、`status=SUCCESS`、`failed_robot_ids=[]`；失败批次必须记录 error 与 HOLD。

【结果】prepare：FAIL（见 §3 根因）；coverage-segment-1 / fault-exit / recovery / coverage-segment-2 / hold-end：未执行（prepare FAIL，批次继续条件未满足，不继续批次）。

#### 2.8.1 prepare 批次结构说明与 2026-08-22 有头模式复测

> 2026-08-22 有头模式（15 机 PX4 SITL + GUI）在坐标修复（`b4f99c3`，mission_anchor=[0,0,0]）
> 与航点密化（`0aaab11`，2m 间距）后重测 prepare。

**批次结构**：
- 命令：`MOVE_TO`；参与机：A01-A12；目标：任务坐标（右/前/上）经 `task_point_to_local_enu`
  （anchor=[0,0,0]，按参考航向旋转）转 EKF ENU 单点目标。
- A01 目标 ENU=(50,-17.5,12)；编队 4 列×3 行（x=45/50/55，y=-12.5/-17.5/-22.5/-27.5）。
- 期望动作结构：垂直 15m 层 → EGO 水平 → 垂直目标 z（当前实现为直线三维轨迹，未分层）。

**复测结果**：
- ✅ **编队形成**：12 机到达目标附近（终点偏差 1.3-1.6m，z≈12m），编队结构正确
  （坐标修复生效，不再飞向错误位置）。
- ⚠️ `status=TIMEOUT`（150s 绝对期限，`failed_robot_ids=[]`）：终点偏差 > 0.2m 到达容差，
  driver 未判 `COMPLETED`。
- ⚠️ **速度峰值 10.6 m/s**（预期 1.0 m/s）：全局规划 + 稀疏控制点导致 B-spline 时间压缩，
  PX4 位置环全速追。
- ⚠️ **无 15m 层**：z 一路爬升至目标 12m。
- ⚠️ **机间最小距离 0.93 m**（UAV6-UAV7，t≈8s，交叉飞行，<1m 水平门禁）。

**根因与修复方向**：见 `plans/implementation_plan_26082300.md`（方案 B：机载分层航点 2m 密化 +
C++ `planning_horizon` 滚动局部规划，对齐原版 ego-planner-swarm）。

### 2.9 测试结束清理

```bash
# 1) 确认所有飞机稳定悬停或安全降落；停止 TCP 发送
# 2) 依次停止：TCP server → GCS backend/标定/bridge（各自终端 Ctrl+C）
# 3) 机载层停止（按 PID，不用宽泛 killall）：
cd /home/ub20tg/catkin_swarm6-2
bash src/safe_valley_exp/startup_offboard_ego.sh stop 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
# 4) 操作者确认后停止 Gazebo/PX4 SITL
# 5) 清理后检查无残留：
ss -ltnp | grep -E '11300|11310|1131[1-9]|1132[0-5]|39001' || true
ps -ef | grep -E '[g]azebo|[p]x4|[m]avros|[r]osmaster|[e]go_planner|[u]av_executor' || true
```

【预期】机载 PID、MAVROS、executor、ego、bridge 退出；Gazebo/PX4 按操作者确认后退出；无残留进程、无占用端口。

【结果】PASS（prepare FAIL 后停止批次结束测试；会话结束时仿真进程全部停止，残留 0）

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
