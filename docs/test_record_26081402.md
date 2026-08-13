# 15 机 PX4/Gazebo 起飞与批次任务复测记录

时间：2026-08-14 ____:____ CST  
记录人：____________  
测试环境：PX4 SITL + Gazebo GUI + MAVROS + ego_planner_driver + swarm_uav_executor + swarm_topology_bridge + GCS_A + TCP runtime

> 本文是新一轮实测记录。每一步执行后填写“实际结果”。没有实测，不写 PASS。  
> 飞行步骤由操作员确认。助手不自动 arm、不自动切 OFFBOARD、不替代 QGC 操作。  
> 任一步失败、位置异常、模式异常、failsafe、任务失败：停止后续批次，先 HOLD/降落/清理。

## 0. 测试目标与批次

目标链路：

```text
runtime TCP 39001
  -> tcp_server_node
  -> GCS_A backend
  -> swarm_topology_bridge
  -> swarm_uav_executor
  -> ego_planner_driver
  -> MAVROS/PX4 SITL
```

| 批次 | 命令 | 参与机 | 主要预期 |
|---|---|---|---|
| prepare | `MOVE_TO` | A01-A12 | 按需起飞/进入 OFFBOARD，完成准备点 |
| coverage-segment-1 | `FOLLOW_ROUTE` | A01-A12 | A01 leader 航线，A02-A12 编队跟随 |
| fault-exit | `FAULT_EXIT` | A07-A12 | 先到 8m 离队层，再执行退出动作 |
| recovery | `MOVE_TO` | A01-A06、A13-A15 | 恢复编队位置 |
| coverage-segment-2 | `FOLLOW_ROUTE` | A01-A06、A13-A15 | 第二固定编队执行航线 |
| hold-end | `HOVER` | A01-A06、A13-A15 | 冻结当前位置并持续 HOLD |

批次继续条件：上一批 `SUCCESS`、参与机状态正常、无 failsafe、位置/模式符合预期。  
整批判据：全部参与机完成才算 `SUCCESS`；任一失败应记录 failed robot、error code、HOLD 结果。

## 1. 测试前填写

| 项目 | 填写 |
|---|---|
| PX4 commit/version | |
| ROS Noetic 环境 | |
| 工作区 | `/home/ub20tg/catkin_swarm6-2` |
| 仿真 Master | `11300` |
| GCS_A Master | `11310` |
| UAV Master | `UAVn -> 11310+n` |
| Gazebo Master | `11345` |
| TCP | `39001` |
| 参与机 | |
| 任务 timeout_s | |
| QGC 版本 | |
| 测试开始时间 | |

## 2. 前置构建与环境

### 2.1 构建

终端 A：

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
catkin build --no-status
source /home/ub20tg/catkin_swarm6-2/devel/setup.bash
```

作用：编译当前代码，加载最新消息、服务、launch、节点。  
预期：构建成功，无 failed/error。

实际结果：

```text
填写命令输出：
填写退出码：
```

通过：`[ ]`    日志：____________________________

### 2.2 ROS/PX4 环境变量

每个相关终端执行：

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
预期：`rospack find safe_valley_exp`、`rospack find tcp_to_ros` 均成功。

实际结果：

```text
rospack 输出：
ROS_HOME/ROS_LOG_DIR：
```

通过：`[ ]`    日志：____________________________

## 3. 启动仿真层

终端 A：

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

作用：启动 Gazebo 与 15 个 PX4 SITL。  
预期：15 架 iris 生成；仿真 TCP 为 `4560..4574`；MAVLink 远端端口为 `34580..34594`；无明显重叠、坠落。

实际结果：

```text
Gazebo 进程/窗口：
生成数量：____/15
PX4 心跳：____/15
异常：
```

通过：`[ ]`    日志：____________________________

### 3.1 打开 Gazebo/QGC，确认安全基线

仿真层启动命令保持运行后，操作员执行：

1. 确认 Gazebo GUI 已打开、15 架 iris 已生成。
2. 打开 QGroundControl。
3. 确认可见 15 个 PX4 系统，映射为 `iris_0..iris_14 -> UAV1..UAV15 -> A01..A15`。
4. 确认全部 `armed=false`。
5. 清除上一轮任务、HOLD、RTL、failsafe 状态。

作用：仿真启动后立即确认飞行界面、机号映射、安全初始状态。  
预期：15 机可见，全部未 arm，无遗留任务或 failsafe。

实际结果：

```text
Gazebo：
QGC 心跳数量：____/15
映射检查：
armed 状态：
failsafe/遗留任务：
```

通过：`[ ]`    操作者确认：____________________

## 4. 启动 15 个机载层

终端 B：

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
bash src/safe_valley_exp/startup_offboard_ego.sh 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
```

作用：每机启动独立 ROS Master、MAVROS、ego driver、executor、bridge；不 arm。  
预期：`/tmp/UAV1_offboard_ego.pid` 到 `/tmp/UAV15_offboard_ego.pid` 存在；15 个 Master 在线；MAVROS connected；`/exec_state=IDLE`；全部 disarm。

实际结果：

```text
PID 文件：____/15
ROS Master：____/15
MAVROS connected：____/15
exec_state=IDLE：____/15
armed=false：____/15
异常 UAV/日志：
```

通过：`[ ]`    日志目录：`/tmp/UAVn_offboard_ego.log`

## 5. 机载只读检查

UAV1 示例；其他机把端口 `11311` 换成 `11312..11325`，逐机记录，不能拿 UAV1 代替全部机：

```bash
export ROS_MASTER_URI=http://localhost:11311
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/local_position/pose
rostopic echo -n 1 /exec_state
rosservice list | grep -E 'uav_task|uav_task_control|uav_hold|uav_safety_lease'
```

作用：确认状态、位姿、setpoint、executor 服务都正常。  
预期：`connected=true`、`armed=false`、`mode` 为安全模式、`exec_state=IDLE`、位姿持续更新、setpoint 约 30Hz、服务齐全。

实际结果：

| UAV | connected | armed | mode | exec_state | pose 更新 | setpoint Hz | 服务齐 | 通过 |
|---|---|---|---|---|---|---:|---|---|
| UAV1 | | | | | | | | |
| UAV2 | | | | | | | | |
| UAV3 | | | | | | | | |
| UAV4 | | | | | | | | |
| UAV5 | | | | | | | | |
| UAV6 | | | | | | | | |
| UAV7 | | | | | | | | |
| UAV8 | | | | | | | | |
| UAV9 | | | | | | | | |
| UAV10 | | | | | | | | |
| UAV11 | | | | | | | | |
| UAV12 | | | | | | | | |
| UAV13 | | | | | | | | |
| UAV14 | | | | | | | | |
| UAV15 | | | | | | | | |

## 6. 启动 GCS_A bridge、backend、TCP

### 6.1 GCS_A bridge

终端 C：

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

作用：注册跨 Master topic/service 代理；不启动 mock。  
预期：GCS_A 看见 15 机状态、位姿、origin、`uav_task`、`uav_task_control`、`uav_hold`、`uav_safety_lease` 代理。

实际结果：

```text
代理数量：
topic 检查：
service 检查：
```

通过：`[ ]`    日志：____________________________

### 6.2 GCS_A backend + 标定

终端 D：

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11310
export ROS_HOSTNAME=localhost
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
roslaunch /home/ub20tg/catkin_swarm6-2/src/tcp_to_ros/launch/gcs_a_backend.launch
```

作用：启动 GCS_A 分发后端与全局原点/航向标定。  
预期：A01-A06 采样连续约 2s、全部 disarm、航向门禁通过；A01-A15 origin 回读确认；日志出现 `Group A calibration ready: 15/15 origins confirmed`。

实际结果：

```text
origin：
reference_heading：
calibration_ready：
origin confirmed：____/15
backend 参数/日志：
```

通过：`[ ]`    禁止 ready 前 arm/发任务。

### 6.3 TCP server

终端 E：

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

作用：开放 runtime TCP 入口。  
预期：39001 监听；Action、状态 Topic、EmergencyHold 可见；不发送任务。

实际结果：

```text
监听检查：
Action/Topic/Service：
```

通过：`[ ]`    日志：____________________________

## 7. 起飞与 OFFBOARD 复测

### 7.1 起飞前确认

操作员检查：

1. QGC 15 机在线、位置正常、全部 disarm。
2. `calibration_ready=true`、origin confirmed `15/15`。
3. `exec_state=IDLE`。
4. setpoint 约 30Hz。
5. PX4 无 RC SITL 场景确认 `COM_RCL_EXCEPT=4`、`NAV_RCL_ACT=0` 已配置；记录参数，不盲改实机参数。

作用：避免未标定、无 setpoint、RC failsafe、状态机非 IDLE 时起飞。  
预期：所有门禁通过。

实际结果：

```text
门禁结果：
COM_RCL_EXCEPT：
NAV_RCL_ACT：
操作员确认：
```

通过：`[ ]`

### 7.2 逐机 arm + OFFBOARD

操作员确认后，终端 F 执行：

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/safe_valley_exp/scripts/offboard_takeoff_15.py 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
```

作用：脚本按当前实现逐机调用 arm，再请求 OFFBOARD；ego driver 负责后续软起飞/状态转换。脚本不调用 MAVROS takeoff。  
预期：输出 `成功 15/15`；QGC 全部 `armed=true / mode=OFFBOARD`；ego driver 执行软起飞并回到 `IDLE` 或约定待命状态；无 RTL/failsafe。

实际结果：

```text
脚本输出：
成功数量：____/15
QGC armed/OFFBOARD：____/15
起飞后高度/位置范围：
exec_state：
failsafe：
```

通过：`[ ]`    操作者确认：____________________

> 若脚本失败：不要继续批次。记录失败 UAV，确认其模式/位置，执行 HOLD 或人工降落。

## 8. 六批次任务复测

每批执行前填写 command/参与机/timeout。每批结束确认整批终态、逐机终态、位置、速度、mode、setpoint。  
runtime 客户端会打印 JSON；保存完整输出，不只记 SUCCESS。

### 8.1 执行命令模板

终端 G：

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 references/runtime/task_test.py --backend tcp --group A --task <TASK> \
  --tcp-host 127.0.0.1 --tcp-port 39001 --timeout-s <TIMEOUT_S>
```

作用：经 TCP 发送一个完整 Group A 批次，等待整批终态。  
预期：正常批次退出码 0、`status=SUCCESS`、`failed_robot_ids=[]`；失败批次必须记录 error 与 HOLD。

### 8.2 批次记录表

| 顺序 | TASK | 命令 | 参与机 | timeout_s | command_id | 预期 | 实际结果/原始输出位置 | failed/error | HOLD 结果 | 通过 |
|---:|---|---|---|---:|---|---|---|---|---|---|
| 1 | `prepare` | `MOVE_TO` | A01-A12 | | | `SUCCESS`，按需起飞/到准备点 | | | | `[ ]` |
| 2 | `coverage-segment-1` | `FOLLOW_ROUTE` | A01-A12 | | | `SUCCESS`，A01 leader、followers 完成 | | | | `[ ]` |
| 3 | `fault-exit` | `FAULT_EXIT` | A07-A12 | | | `SUCCESS`，8m 离队层后退出 | | | | `[ ]` |
| 4 | `recovery` | `MOVE_TO` | A01-A06、A13-A15 | | | `SUCCESS`，恢复位置 | | | | `[ ]` |
| 5 | `coverage-segment-2` | `FOLLOW_ROUTE` | A01-A06、A13-A15 | | | `SUCCESS`，第二编队完成 | | | | `[ ]` |
| 6 | `hold-end` | `HOVER` | A01-A06、A13-A15 | | | `SUCCESS`，当前位置冻结/保持 | | | | `[ ]` |

### 8.3 每批执行后填写

```text
批次：____________________
开始/结束：________________
完整命令：

完整 stdout/stderr 文件：
退出码：
JSON status：
tcp command_id：
参与机：
逐机 ACCEPTED：
逐机 START/EXECUTING：
逐机 COMPLETED/FAILED：
GCS_A 最终结果：
位置/速度/模式：
setpoint Hz：
失败 UAV/error_code：
HOLD 请求与返回：
日志路径：
异常/恢复动作：
```

### 8.4 失败即停/HOLD 专项记录

只在正常批次完成后做故障测试。不要直接移动 Gazebo 模型制造碰撞。  
建议故障：暂停 Gazebo 时钟、注入可控超时、取消 Action、停止单个 bridge。一次只做一种。

```bash
# 示例：另开终端观察 GCS_A 侧整批状态
export ROS_MASTER_URI=http://localhost:11310
rostopic echo /group_a/group_task_state
```

作用：确认失败即停、活动机 HOLD、未参与机不被误 HOLD、无后台继续运动。  
预期：整批 `FAILED` 或 `TIMEOUT`；活动任务收到 HOLD；逐机终态可解释；恢复后旧任务不复活。

实际结果：

```text
故障类型：
注入时间：
受影响 UAV：
整批状态：
failed_uavs/error_code：
活动 UAV HOLD：
未参与 UAV 是否误 HOLD：
setpoint 是否继续/停止：
恢复动作：
旧任务是否复活：
日志：
```

通过：`[ ]`

## 9. 清理

### 9.1 正常清理

操作员先确认所有飞机稳定悬停或安全降落；停止 TCP 发送；再按顺序停止：

1. TCP server。
2. GCS_A backend/标定/bridge。
3. 15 个机载层。
4. Gazebo/PX4 SITL。

终端 B 执行机载清理：

```bash
cd /home/ub20tg/catkin_swarm6-2
bash src/safe_valley_exp/startup_offboard_ego.sh stop 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
```

作用：按脚本 PID 停止机载层，不用宽泛 `killall`/`pkill -f`。  
预期：机载 PID、MAVROS、executor、ego、bridge 退出；Gazebo/PX4 按操作者确认后退出。

实际结果：

```text
TCP stopped：
GCS stopped：
机载停止：____/15
Gazebo/PX4 stopped：
残留进程/端口：
```

通过：`[ ]`    日志：____________________________

### 9.2 清理后检查

```bash
ss -ltnp | grep -E '11300|11310|1131[1-9]|1132[0-5]|39001' || true
ps -ef | grep -E '[g]azebo|[p]x4|[m]avros|[r]osmaster|[e]go_planner|[u]av_executor' || true
```

预期：没有本轮残留进程；端口按实际保留情况记录。  
实际结果：

```text
端口检查：
进程检查：
```

通过：`[ ]`

## 10. 总结

| 验收项 | 结果 |
|---|---|
| 构建 | `[ ] PASS  [ ] FAIL` |
| 15 机仿真启动 | `[ ] PASS  [ ] FAIL` |
| 15 机机载层在线 | `[ ] PASS  [ ] FAIL` |
| 标定 ready 15/15 | `[ ] PASS  [ ] FAIL` |
| 起飞/arm/OFFBOARD | `[ ] PASS  [ ] FAIL` |
| prepare | `[ ] PASS  [ ] FAIL` |
| coverage-segment-1 | `[ ] PASS  [ ] FAIL` |
| fault-exit | `[ ] PASS  [ ] FAIL` |
| recovery | `[ ] PASS  [ ] FAIL` |
| coverage-segment-2 | `[ ] PASS  [ ] FAIL` |
| hold-end | `[ ] PASS  [ ] FAIL` |
| 失败即停/HOLD | `[ ] PASS  [ ] FAIL` |
| 清理无残留 | `[ ] PASS  [ ] FAIL` |

最终结论：

```text
通过批次：
失败批次：
主要失败根因：
是否允许进入下一轮/实机门禁：
补充说明：
```

## 11. 安全边界

- `POSE_STALE` 只代表规划输入过期，不等于飞机停止、不等于安全悬停。
- ego_planner_driver 执行中可能继续发布已生成轨迹；位姿失联、setpoint 中断、OFFBOARD 丢失行为还要结合 PX4 failsafe 实测。
- 本文 SITL 结果不等于实机上桨许可。
- 实机前必须另行冻结：`COM_OF_LOSS_T`、`COM_OBL_ACT`、RC/datalink failsafe、模式切换、急停职责、场地许可。
- 任何异常优先人工 HOLD/降落/急停；不要用宽泛清理命令误杀用户仿真。