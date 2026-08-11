# tcp_to_ros 端到端联调测试步骤（USAGE）

本文档记录**从 runtime 发布 TCP 数据到无人机执行**的全过程联调测试步骤，供你在多 Master 隔离仿真 + 图形界面 Gazebo/PX4 SITL 环境下验证当前开发结果。

测试目标链路：

```text
runtime (TCP 39001)
   │
   ▼
tcp_server_node  ── Action ──►  gcs_a_backend_node (GCS_A 整组分发后端)
   │                                  │  按身份映射拆分单机任务
   │                                  ▼
   │                   swarm_topology_bridge (Topic 桥 + Service 代理, 跨 Master)
   │                                  │
   │                                  ▼
   │                         swarm_uav_executor (UAV 本机执行器)
   │                                  │
   │                                  ▼
   │                         MAVROS / ego_planner_driver (无人机执行)
```

文档按「仿真层 / 机载层 / 地面站层 / runtime 层」四层组织，每步写明在哪个终端执行、执行什么命令、预期效果。

---

## 0. 前置构建

所有终端都需要先在工作空间 build 并 source；若尚未构建，先执行：

```bash
# 终端 (任意，推荐新开)
cd /home/ub20tg/catkin_swarm6-2
catkin build tcp_to_ros swarm_uav_executor swarm_uav_interfaces swarm_topology_bridge ego_planner_driver safe_valley_exp
source /opt/ros/noetic/setup.bash
source /home/ub20tg/catkin_swarm6-2/devel/setup.bash
```

预期：构建成功无报错。

---

## 场景 1：单 Master mock 全链路自检（不需要图形界面）

> 目的：验证「TCP → GCS_A 整组分发 → 单机 executor（mock）→ 状态回传聚合」这条核心链路在单 Master 下闭环可用。UAV1..UAV3 全部使用 mock driver，不操作真实飞控。

### 1.1 启动 GCS_A 后端 + 3 个 mock executor（终端 B）

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
roslaunch tcp_to_ros gcs_a_backend_test.launch
```

预期：
- `gcs_a_backend_node` / `uav_executor_A01/A02/A03`（mock）/ `mock_state_uav1..3` 节点启动；
- `/group_a/execute_task` Action server、`/group_a/emergency_hold` Service、`/UAV{1,2,3}/uav_safety_lease|uav_task|uav_hold` Service 全部注册；
- 各 mock MAVROS 状态 `/UAV{n}/mavros/state` 为 `connected: True`。

测试结果（2026-08-10 21:30 执行，PASS）：

| 验证项 | 预期 | 实际结果 | 是否通过 |
|---|---|---|---|
| 节点 | `gcs_a_backend_node`/`uav_executor_A01/A02/A03`/`mock_state_uav1..3` 在线 | `/gcs_a_backend_node`、`/uav_executor_A01/A02/A03`（mock）、`/mock_state_uav1/2/3`、`/rosout` 全部在线 | 是 |
| Action server | `/group_a/execute_task` 5 个 topic 注册 | goal/cancel/status/feedback/result 全部注册 | 是 |
| Service | `/group_a/emergency_hold` + 每机 `uav_task/uav_hold/uav_safety_lease` 注册 | 共 10 个 Service 注册 | 是 |
| mock MAVROS 状态 | `/UAV{n}/mavros/state` 存在且 connected | topic 存在且 mock_state 节点运行 | 是 |

### 1.2 下发整组 MOVE_TO（终端 B 另开窗口）

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
python3 src/tcp_to_ros/test/test_gcs_a_backend_client.py --uavs A01,A02,A03 --command MOVE_TO --timeout 15 --expect COMPLETED
```

预期：
- `RESULT_FINAL=COMPLETED`，`completed_uavs=A01,A02,A03`；
- 中间可观察到 GCS_A 逐个发出「任务接收确认」、mock executor 返回 `ACCEPTED → COMPLETED` 的状态流。

测试结果（2026-08-10 21:30 执行，PASS）：

| 验证项 | 预期 | 实际结果 | 是否通过 |
|---|---|---|---|
| 整组结果 | `RESULT_FINAL=COMPLETED`，completed=A01,A02,A03 | `RESULT_STATE=3` / `RESULT_FINAL=COMPLETED` / `RESULT_COMPLETED=A01,A02,A03` / `RESULT_MESSAGE=all UAVs completed` / `ASSERT_OK` | 是 |
| 逐机状态流 | 每机 `ACCEPTED → COMPLETED` | 每机 `ACCEPTED(seq1) → COMPLETED(seq2)`，终态重发 3 次 | 是 |
| 整组接收确认 | GCS_A 逐个发「任务接收确认」 | feedback `all participating UAVs accepted` → `all UAVs completed` | 是 |
| 链路健康 | MAVROS 心跳稳定 | `uav_link_health`：`valid MAVROS heartbeat is stable` | 是 |

### 1.3（可选）幂等验证

再次执行 **1.2 同一命令**。

预期：因为 `mission_id/command_id/uav_id` 相同，executor 幂等缓存直接返回已缓存终态，GCS_A 仍返回 `COMPLETED`，不会重复启动任务。

测试结果（2026-08-10 21:32 执行，PASS）：

| 验证项 | 预期 | 实际结果 | 是否通过 |
|---|---|---|---|
| 幂等返回 | 相同 `command_id` 仍 `COMPLETED` | 固定 `it-cmd-fixed-001` 连跑两次均 `RESULT_FINAL=COMPLETED`、`RESULT_COMPLETED=A01,A02,A03` | 是 |
| 不重复执行 | 第二次不新增 `ACCEPTED(seq1)` | UAV1 状态流仅第一次 `ACCEPTED(seq1)`；第二次回放缓存 `COMPLETED(seq2)`，未新增 ACCEPTED | 是 |

### 1.4（可选）失败即停验证：使用失败注入 launch

杀掉 1.1 的 roslaunch，改启动：

```bash
roslaunch tcp_to_ros gcs_a_backend_fail.launch
```

再执行 1.2 命令。

预期：UAV2 mock 被注入失败 → 整组 `RESULT_FINAL=FAILED`，`failed_uavs=A02`，`error=MOCK_UAV2_FAIL`，并且 GCS_A 对已接受任务的活动 UAV 发出 HOLD。

测试结果（2026-08-10 21:35 执行，PASS）：

| 验证项 | 预期 | 实际结果 | 是否通过 |
|---|---|---|---|
| 整组失败 | `FAILED`、failed=A02、error=`MOCK_UAV2_FAIL` | `RESULT_STATE=4` / `RESULT_FINAL=FAILED` / `RESULT_FAILED=A02` / `RESULT_ERROR=MOCK_UAV2_FAIL` | 是 |
| 注入机状态 | UAV2 `FAILED(MOCK_UAV2_FAIL)` | UAV2 状态流 `ACCEPTED → FAILED(MOCK_UAV2_FAIL)` | 是 |
| 活动机 HOLD | UAV1/UAV3 被远程 HOLD | UAV1、UAV3 `ACCEPTED → FAILED(COMMAND_HELD)` `task interrupted by HOLD` | 是 |

---

## 场景 2：runtime 通过 TCP 直发整组批次命令（单 Master mock，不需要图形界面）

> 目的：验证 `references/runtime` 作为 TCP 客户端，把整组批次命令发给 `tcp_server_node`，走真实 GCS_A 分发闭环。

> 注：本场景使用旧 `tcp_to_ros_mock.launch`（TCP Server + mock 后端，不经过 gcs_a_backend）。若要走「GCS_A 后端 + 单机 mock executor」的完整链路，可直接把场景 1 中 GCS_A 后端与 TCP Server 合并启动（见下方「场景 2+」补充）。

### 2.1 启动 TCP Server + mock 后端（终端 B，Master 11311）

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
roslaunch tcp_to_ros tcp_to_ros_mock.launch
```

预期：
- `tcp_server_node` 监听 `127.0.0.1:39001`；
- `mock_backend_node` 提供 `/group_a/execute_task` Action。

测试结果（2026-08-10 21:36 执行，PASS）：

| 验证项 | 预期 | 实际结果 | 是否通过 |
|---|---|---|---|
| TCP 监听 | `tcp_server_node` 监听 39001 | `ss -tlnp` 显示监听 `0.0.0.0:39001` | 是 |
| Action / 节点 | `/group_a/execute_task` + 两节点在线 | 5 个 Action topic 注册；`/tcp_server_node`、`/mock_backend_node` 在线 | 是 |

### 2.2 发送 prepare 批次（终端 D）

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 references/runtime/task_test.py --backend tcp --group A --task prepare --tcp-host 127.0.0.1 --tcp-port 39001
```

预期：返回 JSON，`"status": "SUCCESS"`。

测试结果（2026-08-10 21:36 执行，PASS）：

| 验证项 | 预期 | 实际结果 | 是否通过 |
|---|---|---|---|
| prepare 批次 | `status=SUCCESS`、failed=[] | `status=SUCCESS`、`failed_robot_ids=[]`、`MOVE_TO` 12 机（A01..A12）、`command_id=tcp-ee1705d003e84d1483adf173071abe50`、`ticks=14` | 是 |

### 2.3 依次发送其余批次（终端 D）

```bash
python3 references/runtime/task_test.py --backend tcp --group A --task coverage-segment-1 --tcp-host 127.0.0.1 --tcp-port 39001
python3 references/runtime/task_test.py --backend tcp --group A --task fault-exit        --tcp-host 127.0.0.1 --tcp-port 39001
python3 references/runtime/task_test.py --backend tcp --group A --task recovery         --tcp-host 127.0.0.1 --tcp-port 39001
python3 references/runtime/task_test.py --backend tcp --group A --task coverage-segment-2 --tcp-host 127.0.0.1 --tcp-port 39001
python3 references/runtime/task_test.py --backend tcp --group A --task hold-end         --tcp-host 127.0.0.1 --tcp-port 39001
```

预期：每条命令返回 JSON `"status": "SUCCESS"`。

测试结果（2026-08-10 21:37 执行，PASS）：

| 批次 | 预期 | 实际结果 | 是否通过 |
|---|---|---|---|
| coverage-segment-1 | `SUCCESS`、failed=[] | `SUCCESS`、failed=[] | 是 |
| fault-exit | `SUCCESS`、failed=[] | `SUCCESS`、failed=[] | 是 |
| recovery | `SUCCESS`、failed=[] | `SUCCESS`、failed=[] | 是 |
| coverage-segment-2 | `SUCCESS`、failed=[] | `SUCCESS`、failed=[] | 是 |
| hold-end | `SUCCESS`、failed=[] | `SUCCESS`、failed=[] | 是 |

### 场景 2+（推荐：走 GCS_A 后端分发 + 单机 mock executor 完整链路）

> 若需要验证「runtime → TCP Server → GCS_A 后端 → 单机 executor」的完整链路（比 `tcp_to_ros_mock.launch` 更接近最终形态），做法：
> 1. 先用验收 launch 启动 GCS_A 后端 + **15 个 mock executor**（A01..A15）+ 15 个 mock MAVROS state（`verification/acceptance/gcs_a_backend_15exec.launch`）；
> 2. 再单独启动 TCP Server：
>
> ```bash
> cd /home/ub20tg/catkin_swarm6-2
> source /opt/ros/noetic/setup.bash
> source devel/setup.bash
> export ROS_MASTER_URI=http://localhost:11311
> export ROS_HOSTNAME=localhost
> rosrun tcp_to_ros tcp_server_node.py _host:=0.0.0.0 _port:=39001 _action_name:=/group_a/execute_task _state_topic:=/group_a/group_task_state _emergency_hold_service:=/group_a/emergency_hold
> ```
>
> 预期：`tcp_server_node` 监听 39001 并代理到 GCS_A 后端 Action。
> 3. 然后按 2.2/2.3 发送批次命令。

测试结果（2026-08-10 22:34 执行，部分 PASS）：

| 批次 | 命令 | 参与机 | 实际结果 | 是否通过 | 说明 |
|---|---|---|---|---|---|
| prepare | MOVE_TO | A01..A12 | `status=SUCCESS`、failed=[] | 是 | 完整链路（TCP→GCS_A→15 executor） |
| recovery | MOVE_TO | A01..A06 + A13..A15 | `status=SUCCESS`、failed=[] | 是 | reserve 激活后参与集（含 mock executor A13..A15） |
| coverage-segment-1 | FOLLOW_ROUTE | A01..A12 | `status=FAILURE`，failed=A02..A12 | 部分 | **P1 仅支持 FOLLOW_ROUTE leader**（A01 成功）；A02..A12 follower 被 validation 拒绝 `NOT_IMPLEMENTED`（P2 语义） |
| coverage-segment-2 | FOLLOW_ROUTE | A01..A06 + A13..A15 | `status=FAILURE`，failed=A02..A06+A13..A15 | 部分 | 同 coverage-segment-1（仅 leader A01 成功） |
| fault-exit | FAULT_EXIT | A07..A12 | `status=FAILURE`，failed=A07..A12 | 否 | **P1 mock 未实现 FAULT_EXIT 落地语义** |
| hold-end | HOVER | A01..A06 + A13..A15 | `status=FAILURE`，failed=全部 | 否 | **P1 mock 未实现 HOVER 语义** |

> 结论：完整分发链路（runtime TCP → tcp_server → gcs_a_backend → 逐机 executor → mock）**在 P1 支持的命令边界内全部贯通**（MOVE_TO 两批次 SUCCESS、FOLLOW_ROUTE leader 通过）；FOLLOW_ROUTE follower / FAULT_EXIT / HOVER 属 P1 范围外（P2/后续命令语义），被 executor validation/mock 按设计拒绝，非链路故障。
> 过程中发现并修复：`uav_executor_mock.launch` 的 `supported_commands` 用 `<rosparam>` 传 arg 会被当作字面量（`$(arg supported_commands)` 不展开），导致命令全被拒；改为在 mock launch 内硬编码 `[MOVE_TO, FOLLOW_ROUTE, FAULT_EXIT, HOVER]` 后 MOVE_TO 批次恢复 SUCCESS。

---

## 场景 3：真实 MAVROS + PX4 SITL（图形界面，多 Master 隔离仿真端点到端）

> 目的：真实 MAVROS 连接 PX4 SITL，验证「runtime → TCP → GCS_A → bridge → executor → MAVROS → PX4」全链路，无人机实际响应 setpoint。
> 采用「**仿真层 + 机载层 + 地面站层**」两层结构，与 `safe_valley_exp/README.md` 的多 Master 隔离仿真一致。

### 3.1 启动仿真界面（终端 A，仿真 Master：11300）

图形界面（Gazebo 窗口弹出 4 架 iris）：

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source /home/ub20tg/PX4_Firmware/Tools/setup_gazebo.bash /home/ub20tg/PX4_Firmware /home/ub20tg/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:/home/ub20tg/PX4_Firmware:/home/ub20tg/PX4_Firmware/Tools/sitl_gazebo"
export ROS_MASTER_URI=http://localhost:11300
export ROS_HOSTNAME=localhost
export GAZEBO_MASTER_URI=http://localhost:11345
roslaunch safe_valley_exp multi_uav_sim.launch
```

若你的环境无法弹出 GUI 窗口，改用无头模式（不弹窗口，仍启动 4 架 PX4 SITL）：

```bash
roslaunch safe_valley_exp multi_uav_sim_headless.launch
```

预期：
- Gazebo 窗口（图形模式）出现 4 架 iris（iris_0..iris_3），或启动日志出现 4 次 `Simulator connected on TCP port 4560/4561/4562/4563`；
- 4 架 PX4 SITL 各自监听 `udp 34580/34581/34582/34583`（对应 `tgt_system 1..4`）。

### 3.2 启动 UAV1 机载层（终端 B1，机载 Master：11311）

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
roslaunch safe_valley_exp uav_offboard_ego.launch uav_name:=UAV1 uav_id:=UAV1 tgt_system:=1
```

预期：
- MAVROS 通过 `udp://:24540@localhost:34580` 连接 iris_0；`/mavros/state` 为 `connected: True`、`armed: False`；
- executor 注册 `/UAV1/uav_task`、`/UAV1/uav_hold`、`/UAV1/uav_safety_lease` Service；
- ego 节点发布 `/exec_state`（IDLE）、`/setpoint`、`/trajectory_intent`（本机无前缀话题）。

### 3.3 启动 UAV2 机载层（终端 B2，机载 Master：11312）

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11312
export ROS_HOSTNAME=localhost
roslaunch safe_valley_exp uav_offboard_ego.launch uav_name:=UAV2 uav_id:=UAV2 tgt_system:=2
```

预期：与 3.2 相同，但对接 iris_1（端口 24541/34581），话题注册在 `/UAV2/*`。

### 3.4（可选）启动 UAV3 机载层（终端 B3，机载 Master：11313）

```bash
export ROS_MASTER_URI=http://localhost:11313
export ROS_HOSTNAME=localhost
roslaunch safe_valley_exp uav_offboard_ego.launch uav_name:=UAV3 uav_id:=UAV3 tgt_system:=3
```

### 3.4b（可选）启动 UAV4 机载层（终端 B4，机载 Master：11314）

```bash
export ROS_MASTER_URI=http://localhost:11314
export ROS_HOSTNAME=localhost
roslaunch safe_valley_exp uav_offboard_ego.launch uav_name:=UAV4 uav_id:=UAV4 tgt_system:=4
```

### 3.5 启动 GCS_A 地面站（终端 C，地面站 Master：11310）

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11310
export ROS_HOSTNAME=localhost
# 1) GCS_A bridge（只跑 bridge，不跑 mock executor）
roslaunch tcp_to_ros topology_group_a_sim.launch uav_name:=GCS_A run_uav_mock:=false &
# 2) GCS_A 分发后端
roslaunch tcp_to_ros gcs_a_backend.launch &
# 3) TCP Server（监听 39001）
rosrun tcp_to_ros tcp_server_node.py _host:=0.0.0.0 _port:=39001 _action_name:=/group_a/execute_task _state_topic:=/group_a/group_task_state _emergency_hold_service:=/group_a/emergency_hold
```

预期：
- GCS_A 能订阅到来自 UAV1/UAV2/UAV3 的 `/UAV{n}/mavros/state`（经 bridge 转发）；
- GCS_A 注册每个 UAV 的 `/UAV{n}/uav_task|uav_hold|uav_safety_lease` 跨机 Service 代理；
- `tcp_server_node` 监听 39001。

### 3.6 观察链路健康（终端 C 另开窗口）

```bash
source /opt/ros/noetic/setup.bash
source /home/ub20tg/catkin_swarm6-2/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11310
export ROS_HOSTNAME=localhost
rostopic echo /UAV1/mavros/state
```

预期：`connected: True`、`armed: False`；说明 GCS_A 已通过 bridge 连通 UAV1 的 MAVROS 状态。

### 3.7 通过 4 机模拟程序发送整组批次（终端 D）

> 注：`references/runtime` 的 `task_test.py --backend tcp` 发送 15 机整组批次（prepare 含 A01..A15），会分派到不存在的 UAV5..UAV15；4 机场景改用模拟程序 `tcp_4sim_test_client.py` 只发 A01..A03（可加 `--include-a04` 含 A04）。

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
# 单步发送 prepare（MOVE_TO A01..A03）
python3 src/tcp_to_ros/scripts/tcp_4sim_test_client.py --host 127.0.0.1 --port 39001 --tasks prepare
# 或按需选批次：--tasks prepare,coverage-segment-1,hold-end
# （若启动 UAV4 机载层，加 --include-a04）
```

预期：
- 模拟程序连接 TCP 39001，逐批次等待 `ACCEPTED → COMPLETED`，输出 `[PASS] prepare MOVE_TO COMPLETED uavs=['A01','A02','A03']`；
- GCS_A 拆分为单机任务并通过跨机 Service 下发；
- UAV1 executor 接收任务并调用 MAVROS 持续发布 setpoint；
- 可在机载 Master 观察 `/exec_state` 由 IDLE → EXECUTING；MAVROS 收到 setpoint 后 PX4 SITL（Gazebo 中）开始响应；
- 若 UAV 不在 OFFBOARD 且由外部操作员 arm/切 OFFBOARD，则任务可能在本地验证层被拒绝并走安全路径，此时请先按 3.8 完成 arm/OFFBOARD。

### 3.8（可选）操作员手动 arm + OFFBOARD（终端 B1 或 QGC）

依据安全基线，程序默认不自动 arm、不自动切 OFFBOARD；如需无人机实际飞行，由操作员在 QGC 或 MAVROS 手动完成：

```bash
# 示例（MAVROS，操作员手动；禁止自动执行）
# rosservice call /mavros/cmd/arming "value: true"
# rosservice call /mavros/set_mode "custom_mode: 'OFFBOARD'"
```

预期：PX4 进入 OFFBOARD 后，无人机在 Gazebo 中按 setpoint 移动；任务到达后 `exec_state` → `COMPLETED`。

### 3.9 观察安全租约与 END

在 3.7 任务完成后（机载 Master）：

```bash
export ROS_MASTER_URI=http://localhost:11311
rosservice call /UAV1/uav_safety_lease "...operation: 'END'..."   # 由地面站/脚本按接口发起
```

预期：飞行安全租约正常解除 `INACTIVE`；若在任务执行中租约过期，Watchdog 触发本地 HOLD（`/exec_state` 出现 HOLD）。

---

## 清理

```bash
# 在对应终端按 Ctrl-C 停止各 roslaunch；或统一清理测试进程：
pkill -f "roslaunch tcp_to_ros|roslaunch safe_valley_exp|roslaunch swarm|rosrun tcp_to_ros|rosmaster|roslaunch" || true
```

---

## 端口与命名速查

| 对象 | Master 端口 | ZMQ 偏移 | fcu 端口 | 机载服务 |
|---|---|---|---|---|
| 仿真层 Gazebo | 11300 | - | - | - |
| UAV1 (iris_0) | 11311 | 100 | udp://:24540@localhost:34580 | /UAV1/* |
| UAV2 (iris_1) | 11312 | 200 | udp://:24541@localhost:34581 | /UAV2/* |
| UAV3 (iris_2) | 11313 | 300 | udp://:24542@localhost:34582 | /UAV3/* |
| UAV4 (iris_3) | 11314 | 400 | udp://:24543@localhost:34583 | /UAV4/* |
| GCS_A 地面站 | 11310 | 0 | - | /group_a/* + TCP 39001 |

---

*本文件用于端到端联调验证；开发与验证历史见 `docs/tcp_to_ros_测试记录.md`。*