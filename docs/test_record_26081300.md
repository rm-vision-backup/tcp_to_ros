# 冻结坐标、按需起飞与分层轨迹自动化验证记录

时间：2026-08-13 00:36 CST

## 验证范围

- A01-A12 全球位置中位数、圆周平均航向与 30° 门禁的纯函数测试。
- 两个固定编队的精确参与机集合与槽位测试。
- GCS_A 整批 PREPARE/START/ABORT、共享 deadline、聚合失败与 HOLD 测试。
- executor 任务登记、幂等、显式 START/ABORT、HOLD 和安全租约测试。
- EGO driver 的分层调度、followers PI、直接控制仲裁、规划结果 clearance 测试。
- ROS Service/Topic 集成 rostest。

## 执行结果

```text
catkin build --no-status
  6/6 packages succeeded

catkin test --no-status
  tcp_to_ros:          28 tests passed
  swarm_uav_executor:  75 tests passed
  ego_planner_driver:  30 tests passed

catkin_test_results --all build
  133 tests, 0 errors, 0 failures, 0 skipped
```

ROS 集成测试日志按 rostest 默认行为写入 `~/.ros/log`；用户已明确授权该目录读写。

## 尚未覆盖

自动化测试不等价于 15 机 PX4/Gazebo 动力学闭环或实机验证。交付后仍需按以下顺序验收：

1. 启动 A01-A15 机载 MAVROS、EGO、executor 和 topology bridge，保持 disarm。
2. 人工用 QGC 确认 PX4 正常及 RTK 已收敛。
3. 启动 GCS_A，确认 A01-A12 采样 2s、航向门禁通过，且 A01-A15 原点回读全部确认。
4. 再启动 TCP 上游，依次验证未 arm MOVE_TO 的 5m 检查点、15m 分层动作、两个固定编队、
   FAULT_EXIT 8m 离队和 timeout/HOLD 故障路径。

## 15 机 PX4/Gazebo 图形仿真测试手册

本节是本次交付后的逐步测试流程。测试目标是验证真实 PX4 SITL、Gazebo、MAVROS、每机独立
ROS Master、topology bridge、executor、EGO driver、GCS_A 标定和 TCP 上游的完整链路。

### 0. 测试约束和单步确认规则

本轮测试采用以下约束：

- 仿真初始编队是 Gazebo 世界坐标的 3 行 5 列：

  ```text
  y=10: A01 A04 A07 A10 A13
  y= 5: A02 A05 A08 A11 A14
  y= 0: A03 A06 A09 A12 A15
          x=0  x=5  x=10 x=15 x=20
  ```

- Gazebo 世界原点不作为任务原点。任务原点由 A01-A12 的 arm 前全球位置中位数冻结；任务
  坐标仍按右、前、上解释，`mission_enu=(10,5,0)` 对应冻结全球中位点。
- 所有 15 架仿真机在 GCS_A 标定完成前保持 disarm；机载 offboard/executor 可以在线，但不应
  收到任务前主动 arm。
- 发生位置异常、原点未确认、航向超过 30°、机间距离门禁触发或任一批次失败时，停止当前
  测试步骤，不继续发送下一条任务。
- 本会话遵循“单步确认”：每次实际执行命令前，先说明命令文本和作用，等待操作者明确回复
  “确认执行”；执行后只报告该命令结果，再等待下一步。QGC、Gazebo 等图形界面操作同样由
  操作者执行并确认，助手不代替鼠标键盘操作。

### 1. 目录、端口和终端分工

建议使用以下终端分工，避免混淆 `ROS_MASTER_URI`：

| 角色 | ROS Master | 主要内容 |
|---|---:|---|
| 仿真层 | 11300 | Gazebo + 15 个 PX4 SITL，`iris_0..iris_14` |
| UAV1..UAV15 | 11311..11325 | 每机 MAVROS、EGO、executor、relay、bridge、origin receiver |
| GCS_A | 11310 | GCS bridge、标定节点、GCS_A backend、TCP server |
| QGC | 图形界面 | 观察 15 架 PX4 心跳、位置、模式和 arm 状态 |

所有终端开始前都应 source 同一工作空间，并使用工作空间 ROS 日志目录：

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
mkdir -p "$ROS_LOG_DIR"
```

以上是准备命令示例；正式测试时仍按单步确认规则逐条执行，不要把多个启动命令拼成一条。

### 2. 图形界面准备（操作者执行）

1. 打开 Gazebo 图形界面，确认空世界窗口正常显示，并准备观察 15 架初始位置。
2. 打开 QGroundControl，确认可以看到 15 个 PX4 系统心跳；检查每架系统编号与 A01/UAV1 …
   A15/UAV15 的映射，不要在此阶段 arm。
3. 确认 QGC 中没有上一轮遗留的任务、HOLD、RTL 或 failsafe 状态；如有，先由操作者清理并
   回复状态。

完成以上图形界面动作后，操作者回复“界面已打开，15 机未 arm”。在收到确认前不执行 ROS
启动命令。

### 3. 启动 Gazebo 和 15 个 PX4 SITL

#### 3.1 启动仿真层

操作者确认界面准备完成后，执行以下单条命令。作用：在 ROS Master 11300 启动 Gazebo 和
15 个 PX4 SITL，按本记录约定生成 3×5 初始编队；`gui:=true` 保留图形界面。

```bash
export ROS_MASTER_URI=http://localhost:11300
export ROS_HOSTNAME=localhost
roslaunch safe_valley_exp multi_uav_ego_15sim.launch gui:=true
```

预期：Gazebo 出现 15 架 iris，PX4 SITL 分别使用仿真 TCP 4560..4574 和 MAVLink UDP 端口
34580..34594。此命令保持前台运行，不要关闭该终端。

操作者确认 Gazebo 中 15 架均已生成、没有明显重叠或坠落后，再继续下一步。

#### 3.2 启动 15 个机载 ROS Master 和机载程序

执行前说明：下面是一条批量启动脚本命令，会启动 15 个独立 ROS Master 对应的机载 launch；
它不会 arm。脚本为每架机生成其余 14 架的 odom 话题列表，用于距离门禁。

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
bash src/safe_valley_exp/startup_offboard_ego.sh 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
```

注意环境加载顺序：必须先 source `/opt/ros/noetic/setup.bash` 和工作空间 `devel/setup.bash`，
再 source PX4 的 `Tools/setup_gazebo.bash`，最后恢复 `ROS_PACKAGE_PATH`。如果只 source
catkin 工作空间，PX4 的 `px4`/Gazebo launch 依赖可能不可见，机载 roslaunch 会立即退出且不一定
留下有效日志。当前版本的 `startup_offboard_ego.sh` 也会在内部重复加载 PX4/Gazebo 环境，这是
为了保证脚本从新终端、后台终端或不同 shell 调用时行为一致；外部仍建议按上面的完整流程加载。

预期：`/tmp/UAV1_offboard_ego.pid` … `/tmp/UAV15_offboard_ego.pid` 被创建；每个机载 Master
分别监听 11311..11325。每机 executor 应保持 `IDLE`，MAVROS 应为 connected，飞机仍为
disarm。若某架 launch 退出或 MAVROS 不 connected，停止并记录对应 `/tmp/UAVn_offboard_ego.log`。

若需要手动验证脚本启动是否真正持续运行，应在一个保持打开的普通终端中执行上述命令；不要
把后台启动放入会在命令结束时回收子进程的临时执行环境。只读检查可使用 `ROS_MASTER_URI`
分别指向 11311..11325 的 `rosnode list`、`rostopic echo -n 1 /mavros/state` 和
`rostopic echo -n 1 /exec_state`。

操作者确认 15 个机载进程均在线、QGC 仍显示全部 disarm 后，再继续。

### 4. 机载层只读检查

该阶段只做状态读取，不发送任务、不调用 arm。每次检查命令单独执行并记录输出。

对 UAV1 示例（其他 UAV 将端口替换为 11312..11325）：

```bash
export ROS_MASTER_URI=http://localhost:11311
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/local_position/pose
rostopic echo -n 1 /exec_state
rosservice list | grep -E 'uav_task|uav_task_control|uav_hold|uav_safety_lease'
```

预期：`connected: true`、`armed: false`、`exec_state: IDLE`，local pose 持续更新；服务列表中
存在 `uav_task`、`uav_task_control`、`uav_hold` 和 `uav_safety_lease`。对 15 架逐一检查，不能
用 UAV1 正常代替其他机。

### 5. 启动 GCS_A bridge、标定和 TCP server

#### 5.1 GCS_A topology bridge

作用：在 GCS_A Master 11310 注册跨 Master topic/service 代理，但不启动 mock executor。

```bash
export ROS_MASTER_URI=http://localhost:11310
export ROS_HOSTNAME=localhost
roslaunch tcp_to_ros topology_group_a_sim.launch uav_name:=GCS_A run_uav_mock:=false
```

预期：GCS_A 能看到 `/UAV1..15/mavros/state`、global/local pose、`gp_origin_confirmed`，以及
15 架的 `uav_task`、`uav_task_control`、`uav_hold`、`uav_safety_lease` 代理。

#### 5.2 GCS_A backend 和标定节点

作用：启动 `gcs_a_calibration_node.py` 和 `gcs_a_backend_node.py`。标定节点在 A01-A12
全部 disarm、数据连续 2 秒且航向偏差不超过 30° 后冻结原点/航向，并向 A01-A15 等待
MAVROS 回读确认；backend 在 `/group_a/calibration_ready=true` 前拒绝任务。

```bash
roslaunch /home/ub20tg/catkin_swarm6-2/src/tcp_to_ros/launch/gcs_a_backend.launch
```

预期日志顺序：

1. GCS_A backend ready。
2. calibration frozen，打印 origin 和 reference heading。
3. 15 架 origin receiver 分别报告 `gp_origin confirmed`。
4. `Group A calibration ready: 15/15 origins confirmed`。

如果没有 ready，禁止手动 arm 或发送任务；优先检查 A01-A12 的 global position/local pose、
15 架 `gp_origin` 回读和 GCS bridge topic/service 代理。

#### 5.3 GCS_A TCP server

作用：只提供 TCP 上游入口，不改变任务坐标或编队语义。

```bash
rosrun tcp_to_ros tcp_server_node.py _host:=0.0.0.0 _port:=39001 \
  _action_name:=/group_a/execute_task \
  _state_topic:=/group_a/group_task_state \
  _emergency_hold_service:=/group_a/emergency_hold
```

预期：TCP 39001 监听，GCS_A Action `/group_a/execute_task` 已注册。到此为止仍不得有任务
进入；先确认 ready 状态。

### 6. 标定和坐标一致性检查

在 GCS_A Master 11310 上执行只读检查：

```bash
export ROS_MASTER_URI=http://localhost:11310
rostopic echo -n 1 /group_a/calibration_ready
rostopic echo -n 1 /group_a/reference_heading
rostopic echo -n 1 /group_a/gp_origin
for n in $(seq 1 15); do
  echo "UAV$n"
  rostopic echo -n 1 "/UAV$n/gp_origin_confirmed"
done
```

预期：ready 为 `data: true`，reference heading 为固定弧度值，原点为冻结的全球经纬高，15
个 confirmation 均为 `true`。不要求 `gp_origin` 连续发布；确认依据是本轮设置后的 MAVROS
回读和 latched confirmation。

### 7. TCP 上游任务测试顺序

每个任务前都先说明任务命令、参与机、预期动作和风险，等待操作者确认后才发送。`timeout_s`
必须足够覆盖整批 PREPARE、起飞检查点和运动过程；建议仿真初测使用 runtime 任务定义中的
timeout，不要在 backend 里使用预置固定超时。

#### 7.1 MOVE_TO：未 arm 的按需起飞

目的：验证参与机通过 PREPARE 后统一 START；未 arm 机由 executor 完成 OFFBOARD、arm、相对
上升 5m、实测稳定 2s，再升到 15m、水平 EGO、最后垂直到目标 z。

建议先用 runtime 的 `prepare-a` 或等价单个 MOVE_TO 任务，参与机至少包含 A01-A12；确认任务
坐标是任务坐标偏置，不要把 Gazebo 世界坐标直接当 local ENU。发送后观察：

- QGC：参与机从 disarm 变为 armed/OFFBOARD，A13-A15 若未参与则保持 disarm。
- executor：所有参与机先出现 ACCEPTED，再出现 START/执行状态。
- 飞行轨迹：每架先原地上升约 5m 并保持约 2s，然后进入 15m 层，再水平移动，最后到目标 z。
- GCS_A：整批只有全部完成才返回 COMPLETED；任一失败必须整批 FAILED 并 HOLD 活动机。

#### 7.2 已 arm MOVE_TO：跳过检查点

目的：验证同一 mission 中已经 armed/OFFBOARD 的飞机再次收到 MOVE_TO 时不重复执行 5m
takeoff checkpoint，直接进入 15m 分层和目标动作。该步骤必须在 7.1 完成且飞机仍处于可执行
状态后进行。

#### 7.3 FOLLOW_ROUTE：固定编队 1

目的：验证 `coverage-segment-1` 的精确 A01-A12 集合和 leader/follower 分层：A01 为 leader
在 12m 使用 EGO，A02-A12 在 12m 使用 PI 位置控制叠加固定槽位偏置。验证编队相对 A01 的
槽位为：

```text
A01 (0,0)   A04 (5,0)   A07 (10,0)  A10 (15,0)
A02 (0,-5)  A05 (5,-5)  A08 (10,-5) A11 (15,-5)
A03 (0,-10) A06 (5,-10) A09 (10,-10) A12 (15,-10)
```

runtime 下发的 yaw 和编队偏置不参与执行；GCS_A 以冻结 reference heading 转换到 local ENU。
验证 leader 完成最终航点后，followers 才能全部完成；followers 不调用 EGO 水平规划。

#### 7.4 FOLLOW_ROUTE：固定编队 2

目的：验证 `coverage-segment-2` 的精确 A01-A06、A13-A15 集合和 9 机槽位：

```text
A01 (0,0)   A04 (5,0)   A13 (10,0)
A02 (0,-5)  A05 (5,-5)  A14 (10,-5)
A03 (0,-10) A06 (5,-10) A15 (10,-10)
```

如果 runtime 发布的参与机集合与固定集合不一致，预期 GCS_A 在接受阶段返回
`FORMATION_PARTICIPANTS_MISMATCH`，不应有任何机 arm 或移动。

#### 7.5 FAULT_EXIT：8m 离队层

目的：验证故障退出先垂直到 8m，再由 EGO 水平移动到目标点；垂直阶段 x/y/yaw 冻结。发送
前先确认目标机和目标点，观察 QGC 高度变化及 GCS_A 终态。

#### 7.6 垂直距离门禁和整批 HOLD

只在前述正常任务通过后进行故障注入。不要直接移动 Gazebo 模型制造碰撞；优先使用安全的
仿真位置/话题注入方式，使任意两机在垂直阶段满足“水平中心距 <1m 且垂直中心距 <2m”。
预期当前整批立即失败并对已 START 的参与机执行 HOLD；未 START 或未参与机不应被错误 HOLD。

#### 7.7 timeout 和取消

使用较短但可控的 runtime `timeout_s` 或暂停 Gazebo 时钟，验证共享 deadline 到期后 GCS_A 返回
`TIMEOUT` 并整批 HOLD。取消任务时验证 Action cancel 同样触发 HOLD，不留下继续运动的后台
executor 线程。

### 8. 每一步的记录字段

每个单步至少记录：

- 操作时间、操作者确认文本、实际命令或 GUI 操作；
- Gazebo/QGC 中参与机数量、arm/mode、位置和是否出现 failsafe；
- GCS_A calibration ready、reference heading、origin confirmation 数量；
- TCP command_id、runtime plan/task 名称、参与机集合、timeout_s；
- PREPARE/START/ABORT、ACCEPTED、COMPLETED/FAILED/HOLD 结果；
- 失败时的 UAV、error_code、日志文件和后续恢复动作。

### 9. 安全停止和清理

正常完成后由操作者先停止上游任务发送，再确认所有飞机处于稳定悬停或已安全降落；不要在
飞行任务执行中直接关闭 GCS_A。随后按以下顺序逐项确认后执行清理：

1. 停止 TCP server。
2. 停止 GCS_A backend/calibration 和 GCS bridge。
3. 停止 15 个机载 launch：

   ```bash
   bash src/safe_valley_exp/startup_offboard_ego.sh stop 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15
   ```

4. 最后停止 Gazebo/PX4 仿真层，并确认 QGC 中所有系统已断开。

发生异常时优先使用 GCS_A emergency HOLD 或 PX4/QGC failsafe；不要用宽泛的 `killall` 或
`pkill -f` 清理，以免误杀操作者的其他 ROS/Gazebo 进程。

## 当前交互状态

本文档已写入测试步骤，但本次会话尚未启动 Gazebo、PX4、ROS 节点或 TCP 任务。当前等待操作者
完成第 2 节的 Gazebo/QGC 图形界面准备，并回复“界面已打开，15 机未 arm”。收到后从第 3.1
节开始，一次只执行一条命令；每条命令执行前都会再次说明作用并等待确认。
