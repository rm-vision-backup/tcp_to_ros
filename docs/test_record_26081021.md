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

## 场景 3：真实 MAVROS + PX4 SITL（多 Master 隔离仿真端到端）

> **已迁移**：场景 3 的完整操作步骤与测试结果迁移至
> `docs/test_record_26081123.md`（场景 3 无头 15 机多 Master 隔离仿真端到端测试报告，
> 2026-08-11 23:58 生成），**以该文件为准**。
>
> 迁移原因：原场景 3 描述的是 4 机图形界面、逐个启动机载层的方式；
> 已被 15 机无头模式 + `multi_uav_ego_15sim.launch gui:=false` + `startup_offboard_ego.sh`
> 批量启动取代。

---

## 清理

```bash
# 在对应终端按 Ctrl-C 停止各 roslaunch；或统一清理测试进程：
pkill -f "roslaunch tcp_to_ros|roslaunch safe_valley_exp|roslaunch swarm|rosrun tcp_to_ros|rosmaster|roslaunch" || true
```

---

## 端口与命名速查（场景 1/2 单 Master 或双 Master mock）

> 场景 3（真实 MAVROS + PX4 SITL 多 Master）的完整 15 机端口表见 `docs/test_record_26081123.md` §1.2。

| 对象 | Master 端口 | ZMQ 偏移 | 机载服务 |
|---|---|---|---|
| GCS_A 地面站 | 11311（单 Master）/11310（多 Master 时） | 0 | /group_a/* + TCP 39001 |
| UAV1..UAVn（mock，单 Master） | 同 GCS_A | n*100 | /UAV{n}/* |

> 场景 1/2 使用单 Master（默认 11311），UAV mock 执行器与 GCS_A 同 Master 部署；
> 多 Master 形态的逐机端口分配以 `docs/test_record_26081123.md` §1.2 为准。

---

*本文件用于端到端联调验证；开发与验证历史见 `docs/tcp_to_ros_测试记录_早期.md` 与 `docs/test_record_2608xxxx.md` 系列。*
