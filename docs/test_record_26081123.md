# 场景 3 无头 15 机多 Master 隔离仿真端到端测试报告（2026-08-11 23:58）

> 依据 `docs/test_record_26081021.md` 场景 3 的架构（真实 MAVROS + PX4 SITL，多 Master
> 隔离仿真，经 `swarm_topology_bridge` 跨 Master 通信）扩展为**无头 15 机**：仿真层
> 11300 + GCS_A 11310 + 机载层 11311..11325，通过 `references/runtime/task_test.py`
> 发送全部 6 个批次。
>
> 本次测试同时验证了 `docs/test_record_26081115.md` 第 5 节备注中「未在本轮执行」的
> 多 Master 真实 MAVROS/PX4 SITL 端到端场景。

## 0. 测试场景判定

| 项 | 结论 |
|---|---|
| 场景编号 | **场景 3**（真实 MAVROS + PX4 SITL 多 Master 隔离仿真端到端） |
| 与文档差异 | 原文档为 4 机图形界面；本次为 **15 机无头模式**（`multi_uav_ego_15sim.launch gui:=false`） |
| 目标链路 | runtime(TCP 39001) → tcp_server → gcs_a_backend → swarm_topology_bridge(跨 Master) → swarm_uav_executor → MAVROS → ego_planner_driver → PX4 SITL |
| 测试目的 | 验证 15 机完整链路的批处理应答，并将 `test_record_26081115.md` 四种动作命令（MOVE_TO/FOLLOW_ROUTE/FAULT_EXIT/HOVER）在真实 SITL 上的执行情况 |

## 1. 测试环境（无头 15 机多 Master）

| 层 | Master | 内容 | 验证状态 |
|---|---|---|---|
| 仿真层 | 11300 | `multi_uav_ego_15sim.launch gui:=false`：iris_0..iris_14 全部 spawn，PX4 SITL 15 实例（sim TCP 4560-4574、MAVLink offboard UDP 34580-34594） | PASS |
| 机载层 | 11311-11325 | 每机独立 Master：`uav_offboard_ego.launch`（MAVROS + ego driver + pose/setpoint relay + `uav_executor_ego.launch` + bridge） | 15/15 MAVROS connected |
| GCS_A | 11310 | `topology_group_a_sim.launch uav_name:=GCS_A run_uav_mock:=false` + `gcs_a_backend.launch` + `tcp_server_node.py`(39001) | 15 个 `/UAV{n}/uav_task|uav_hold|uav_safety_lease` 跨 Master service 代理、15 个 `/UAV{n}/mavros/state` 桥接、5 个 Action topic |

启动步骤（按 `test_record_26081021.md` 场景 3.1 无头方式）：

```bash
# 终端 A：仿真层（master 11300）
source /opt/ros/noetic/setup.bash && source devel/setup.bash
source /home/ub20tg/PX4_Firmware/Tools/setup_gazebo.bash /home/ub20tg/PX4_Firmware /home/ub20tg/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:/home/ub20tg/PX4_Firmware:/home/ub20tg/PX4_Firmware/Tools/sitl_gazebo"
export ROS_MASTER_URI=http://localhost:11300
export ROS_HOSTNAME=localhost
export GAZEBO_MASTER_URI=http://localhost:11345
roslaunch safe_valley_exp multi_uav_ego_15sim.launch gui:=false

# 终端 B：15 个机载层（master 11311..11325）
bash src/safe_valley_exp/startup_offboard_ego.sh 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15

# 终端 C：GCS_A（master 11310，bridge + backend + TCP）
export ROS_MASTER_URI=http://localhost:11310
roslaunch tcp_to_ros topology_group_a_sim.launch uav_name:=GCS_A run_uav_mock:=false &
roslaunch /home/ub20tg/catkin_swarm6-2/src/tcp_to_ros/launch/gcs_a_backend.launch &
rosrun tcp_to_ros tcp_server_node.py _host:=0.0.0.0 _port:=39001 _action_name:=/group_a/execute_task _state_topic:=/group_a/group_task_state _emergency_hold_service:=/group_a/emergency_hold
```

> 注：`gcs_a_backend.launch` 在 devel/share 未安装（launch 文件未同步到 catkin install），
> 需用绝对路径 `roslaunch /home/ub20tg/catkin_swarm6-2/src/tcp_to_ros/launch/gcs_a_backend.launch`
> 或先执行 `catkin build tcp_to_ros` 使 launch 安装到 devel。

### 1.1 环境变量与日志约定

- 建议统一用以下命令完成 ROS 环境引用（`source devel/setup.bash` 会清空此前
  `export` 的 PX4 SITL 目录，故先 source 再 export）：
  ```bash
  source /opt/ros/noetic/setup.bash && source devel/setup.bash
  source /home/ub20tg/PX4_Firmware/Tools/setup_gazebo.bash /home/ub20tg/PX4_Firmware /home/ub20tg/PX4_Firmware/build/px4_sitl_default
  export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:/home/ub20tg/PX4_Firmware:/home/ub20tg/PX4_Firmware/Tools/sitl_gazebo"
  ```
- 建议将 ROS 运行日志重定向到工作空间内，便于查看/分析/删除：
  ```bash
  export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
  export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
  ```

### 1.2 端口与命名速查（15 机无头 SITL）

| 对象 | Master 端口 | ZMQ 偏移 | sim TCP | fcu（MAVLink offboard） | 机载服务 |
|---|---|---|---|---|---|
| 仿真层 Gazebo | 11300 | - | 4560..4574 | - | - |
| GCS_A 地面站 | 11310 | 0 | - | - | `/group_a/*` + TCP 39001 |
| UAV1 (iris_0) | 11311 | 100 | 4560 | udp://:24540@localhost:34580 | `/UAV1/*` |
| UAV2 (iris_1) | 11312 | 200 | 4561 | udp://:24541@localhost:34581 | `/UAV2/*` |
| UAV3 (iris_2) | 11313 | 300 | 4562 | udp://:24542@localhost:34582 | `/UAV3/*` |
| UAV4 (iris_3) | 11314 | 400 | 4563 | udp://:24543@localhost:34583 | `/UAV4/*` |
| UAV5 (iris_4) | 11315 | 500 | 4564 | udp://:24544@localhost:34584 | `/UAV5/*` |
| UAV6 (iris_5) | 11316 | 600 | 4565 | udp://:24545@localhost:34585 | `/UAV6/*` |
| UAV7 (iris_6) | 11317 | 700 | 4566 | udp://:24546@localhost:34586 | `/UAV7/*` |
| UAV8 (iris_7) | 11318 | 800 | 4567 | udp://:24547@localhost:34587 | `/UAV8/*` |
| UAV9 (iris_8) | 11319 | 900 | 4568 | udp://:24548@localhost:34588 | `/UAV9/*` |
| UAV10 (iris_9) | 11320 | 1000 | 4569 | udp://:24549@localhost:34589 | `/UAV10/*` |
| UAV11 (iris_10) | 11321 | 1100 | 4570 | udp://:24550@localhost:34590 | `/UAV11/*` |
| UAV12 (iris_11) | 11322 | 1200 | 4571 | udp://:24551@localhost:34591 | `/UAV12/*` |
| UAV13 (iris_12) | 11323 | 1300 | 4572 | udp://:24552@localhost:34592 | `/UAV13/*` |
| UAV14 (iris_13) | 11324 | 1400 | 4573 | udp://:24553@localhost:34593 | `/UAV14/*` |
| UAV15 (iris_14) | 11325 | 1500 | 4574 | udp://:24554@localhost:34594 | `/UAV15/*` |

> 端口公式：UAV{n}（n=1..15）→ Master `11310+n`、ZMQ 偏移 `n*100`、
> sim TCP `4560+(n-1)`、fcu 本地 `24539+n`、fcu 远端 `34579+n`。
> 任务层 `A{n}` 与执行层 `UAV{n}`、仿真容器 `iris_{n-1}` 一一对应。

## 2. 起飞 + OFFBOARD（新增脚本）

用户提示：PX4 无 RC 的 SITL 保持 OFFBOARD 需要在每个 master 端口的
`/mavros/setpoint_position/local` 以 >2Hz（推荐 20Hz+）持续发布目标位置，并需设置飞控参数
`COM_RCL_EXCEPT=4`（bit2 为 1：OFFBOARD 下豁免 RC 缺失 failsafe）。

依据官方文档（`https://docs.px4.io/v1.15/en/ros/mavros_offboard_python.html`、
parameter reference `COM_RCL_EXCEPT`）与 MCP 查询结果，编写并验证：

**新增脚本 `verification/ego_smoke/offboard_takeoff_15.py`**（关键点）：

1. 每机通过 `/mavros/param/set` 设置 `COM_RCL_EXCEPT=4`、`NAV_RCL_ACT=0`，并 `/mavros/param/push`；
2. 以 30Hz 持续发布 `/mavros/setpoint_position/local`（setpoint 流不能断）；
3. **先切 OFFBOARD 再 arm**（关键时序）：若先 arm，Command 在 AUTO 模式下被 RC 丢失
   failsafe 立即 disarm（日志证据：`Failsafe enabled: No manual control stick input` →
   `Disarmed by auto preflight disarming`）；`COM_RCL_EXCEPT bit2` 使 OFFBOARD 下豁免；
4. 输入 `--hold` 时到达高度后保持 setpoint 悬停，并监听 `/exec_state` 检测 ego driver
   离开 IDLE（任务开始）自动让位停止发布。

单机验证（UAV1/UAV2 等）与 15 机批量验证：

| 验证项 | 结果 |
|---|---|
| 单机（UAV1，先 OFFBOARD 后 arm） | `已进入 OFFBOARD 且 armed`，爬升至目标 z，PASS |
| 15 机批量 OFFBOARD+armed（`--hold` 保活） | **15/15 全部 `armed=True / mode=OFFBOARD`** |
| 参数设置 | 每机 `COM_RCL_EXCEPT=4`、`NAV_RCL_ACT=0`，`param push transfered=2` |

## 3. 六批次应答结果（`task_test.py --backend tcp`）

所有 UAV 已 OFFBOARD+armed 悬停后，依次发送 6 批次：

| 批次 | 命令 | 参与机 | 结果 | 成功机数 |
|---|---|---|---|---|
| prepare | MOVE_TO | A01..A12 | `FAILURE` | 5/12（A02,A03,A05,A06,A12） |
| coverage-segment-1 | FOLLOW_ROUTE | A01..A12 | `FAILURE` | 11/12（仅 A09 失败） |
| fault-exit | FAULT_EXIT | A07..A12 | `FAILURE` | 5/6（仅 A09 失败） |
| recovery | MOVE_TO | A01..A06 + A13..A15 | `FAILURE` | 8/9（仅 A05 失败） |
| coverage-segment-2 | FOLLOW_ROUTE | A01..A06 + A13..A15 | `FAILURE` | 8/9（仅 A05 失败） |
| hold-end | HOVER | A01..A06 + A13..A15 | `FAILURE` | 8/9（仅 A05 失败） |

## 4. 问题分析与结论

### 4.1 链路层已贯通（核心结论）

- 完整分发链路（runtime TCP → tcp_server → gcs_a_backend → bridge 跨 Master → 逐机
  executor → MAVROS → PX4 SITL）可用：GCS_A 能收到 15 机 `/UAV{n}/uav_task_state`、
  15 个跨 Master service 代理、bridge 转发 MAVROS state。
- coverage-segment-1 / fault-exit / recovery / coverage-segment-2 / hold-end 五个批次
  均只有单机失败，证明四种命令（MOVE_TO/FOLLOW_ROUTE/FAULT_EXIT/HOVER）均能被 executor
  接收并下发执行；相对 `test_record_26081115.md` 的 mock 全绿结果，真实 SITL 下的
  失败是执行层（ego driver）问题而非分发/链路问题。

### 4.2 失败根因（对应 `docs/test_record_26081115.md` 备注及用户提示）

用户明确提示：`src/ego_planner_driver` **未经过验证、可用性存疑**。实测证据与之吻合：

1. **`/exec_state` 状态机不稳定**：任务后常卡在 `HOLD` 不回到 `IDLE`，后续任务被 executor
   以 HOLD 状态拒绝/无响应，导致同一架（如 A09、A05）连续失败。
2. **--hold 保活脚本与 ego driver 抢占 `/mavros/setpoint_position/local`**：任务前保活脚本
   持续发布 setpoint，任务到达时 ego driver 同时发布，PX4 收到混合 setpoint 流，任务执行
   受干扰（prepare 大面积失败与此相关）。
3. **ego 轨迹规划在真实 PX4 下未充分联调**：C++ 规划层（`ego_planner_core`）在任务
   超时（30s）内未能稳定完成 `EXECUTING → COMPLETED`。

### 4.3 其他发现

- `src` 内功能包修改后先使用 `catkin build` 编译,再引用 `devel/setup.bash`，否则ros可能无法找到功能包，但是source会清空 `export` 引用的 PX4 SITL 目录，建议使用以下指令统一完成引用，或打包成脚本启用：
   ```bash
   source /opt/ros/noetic/setup.bash && source devel/setup.bash
   source /home/ub20tg/PX4_Firmware/Tools/setup_gazebo.bash /home/ub20tg/PX4_Firmware /home/ub20tg/PX4_Firmware/build/px4_sitl_default
   export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:/home/ub20tg/PX4_Firmware:/home/ub20tg/PX4_Firmware/Tools/sitl_gazebo"
   ```
   - 建议在新终端启动 ros 进程前，将 ros 日志地址重定向到工作空间内，便于查看、分析、删除，如下：
      ```bash
      export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
      export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
      ```
- 多进程并发启动 15 机起飞会导致部分失败（PX4 arm 并发时序竞争），顺序/逐机启动稳定成功。
- 清理时 `pkill -f` 会匹配到自身命令行被 SIGTERM，需用精确 PID（`pgrep ... | grep -v $$`）。

## 5. 交付物

- `verification/ego_smoke/offboard_takeoff_15.py`：15 机无头 SITL 批量起飞 + OFFBOARD 脚本
  （`COM_RCL_EXCEPT=4`、先 OFFBOARD 后 arm、30Hz setpoint 保活、`--hold` 检测 ego 接管自动让位）。
- 测试环境（仿真层 11300 + 机载层 11311-11325 + GCS_A 11310 + TCP 39001）全部进程已清理，
  关键端口（11300/11310/11311/11320/39001）已释放。

## 6. 后续建议

1. 优先修复/验证 `ego_planner_driver` 状态机（HOLD 后复位 IDLE、EXECUTING/COMPLETED 正确转移），
   这是批次未全绿的直接瓶颈。
2. 任务发送前终止所有 `--hold` 保活脚本，或改为脚本检测首次任务到达即退出，避免 setpoint 抢占。
3. 对 A05/A09 等反复失败机单独观察 `/exec_state` 与 MAVROS setpoint 接收，确认是否为单个
   SITL 实例抖动。
4. 在 `executor_defaults.yaml`/launch 中固化 `COM_RCL_EXCEPT=4` 相关说明（当前仅在验证脚本中处理）。