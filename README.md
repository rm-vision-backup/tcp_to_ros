# tcp-to-ros

`tcp_to_ros` 是 Group A 地面站的 ROS1 Noetic 主功能包：作为 TCP Server 接收 `references/runtime` 的整组批次命令，经本地 `ExecuteGroupTask` Action 拆分，通过通用通信包 `swarm_topology_bridge` 的跨机 Service / Topic，把单机任务可靠分发到各 UAV（GCS_A）并汇总整组结果。本包 `package.xml` / `CMakeLists.txt` 已声明对 `swarm_topology_bridge` 的依赖。

## 组成与文件

- `action/ExecuteGroupTask.action`：GCS 本地整组任务 Action（goal 为整组 `assignments`，feedback/result 表示整组 ACCEPTED/COMPLETED/FAILED）。
- `msg/`：`Pose3DYaw`（位姿）、`TaskAssignment`（逐机任务项）、`UavTaskState`（单机状态三态）、`GroupTaskState`（整组状态聚合）。
- `srv/`：`UavTask.srv`（单机任务接收 Service，跨 Master 代理）、`UavHold.srv`（单机 HOLD Service，跨 Master 代理）、`EmergencyHold.srv`（本地整组紧急悬停）。
- `scripts/`：
  - `tcp_server_node.py`：TCP Server，仅做协议适配，把整组命令交给 Action；不承载逐机路由与聚合。
  - `gcs_a_backend_node.py`：GCS_A 分发后端（`ExecuteGroupTask` Action server）：按身份映射拆分单机任务、并发 Service 确认（单次 1.0s、首次+2 重试、0.2s 间隔、4s 预算）、订阅并聚合单机状态、至少一次请求+幂等、超时/cancel/HOLD、`ALL` 失败即停。
  - `uav_mock_node.py`：单机 mock 执行适配层（提供 `UavTask`/`UavHold` Service、发布 `UavTaskState` Topic、幂等缓存与终态重发）。
  - `mock_backend_node.py` / `task_event_injector.py`：早期整组 mock 与手动 HOLD 注入（旧闭环，仍在但分发层已改用 `gcs_a_backend_node.py`）。
  - `tcp_to_ros/`：`bridge.py`（TCP→Action 转换）、`protocol.py`（JSON 校验）、`dedup.py`、`state.py`、`identity.py`（身份映射加载）。
- `config/`：`uav_identity_map.yaml`（A01..A15→UAV1..UAV15/hostname/IP，待复核）、`topology_group_a_sim.yaml`（Group A 多 Master 桥接/服务代理配置）、`mock_backend.yaml`、`uav_mock_config.yaml`。
- `launch/`：`gcs_a_backend.launch`（仅 GCS_A 后端）、`gcs_a_backend_test.launch`（GCS_A 后端 + UAV1..3 mock，单 Master 语义联调）、`gcs_a_backend_fail.launch`（含失败注入）、`tcp_to_ros_mock.launch`（旧 mock 闭环）、`topology_group_a_sim.launch`（Group A 多 Master 仿真拓扑）。
- `test/test_gcs_a_backend_client.py`：集成/回归测试客户端。

## 构建

只使用 `catkin build`，默认 `build/devel/logs`：

```bash
source ~/.bashrc
catkin build tcp_to_ros
source /home/ub20tg/catkin_swarm6-2/devel/setup.bash
```

若默认 `build`/`devel`/`logs` 被旧 `catkin_make` 产物占用，先删除并重新初始化 `src` 后编译。若无法写默认 `~/.ros`，改用工作区目录：

```bash
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
mkdir -p "$ROS_LOG_DIR"
```

## 运行方式

### 1. 单 Master 语义联调（GCS_A 后端 + UAV1..3 mock）

```bash
roslaunch tcp_to_ros gcs_a_backend_test.launch
```

另开终端跑测试客户端（期望 `COMPLETED`）：

```bash
python3 src/tcp_to_ros/test/test_gcs_a_backend_client.py --uavs A01,A02,A03 \
  --command MOVE_TO --timeout 15 --expect COMPLETED
```

验证要点（已回归通过）：

- 正常整组：`RESULT_FINAL=COMPLETED`，completed_uavs=A01,A02,A03；
- 重复 `command_id`：仍返回缓存 `COMPLETED`（幂等）；
- 失败即停：改用 `gcs_a_backend_fail.launch`（UAV2 mock 注入失败），期望 `FAILED` / failed_uavs=A02 / error=`MOCK_UAV2_FAIL`，并对活动 UAV 执行 HOLD 确认。

### 2. Group A 多 Master 仿真拓扑

```bash
# UAV1 / UAV2 / UAV3 机载 Master
export ROS_MASTER_URI=http://localhost:11311
roslaunch tcp_to_ros topology_group_a_sim.launch uav_name:=UAV1 uav_id:=A01
# GCS_A 地面站 Master
export ROS_MASTER_URI=http://localhost:11314
roslaunch tcp_to_ros topology_group_a_sim.launch uav_name:=GCS_A run_uav_mock:=false
# GCS_A 侧再单独启动分发后端
roslaunch tcp_to_ros gcs_a_backend.launch
```

- 依赖解析：跨 Master 的 `UavTask`/`UavHold` Service 与 `UavTaskState` Topic 由 `swarm_topology_bridge` 的通用桥（Topic 桥 + Service 代理）转发；
- 端口规划与命名约定见 `config/topology_group_a_sim.yaml`（port_offset 区分单机多 Master，实机 port_offset=0）。

### 3. 与 runtime 的 TCP 联调（旧 mock 闭环）

```bash
roslaunch tcp_to_ros tcp_to_ros_mock.launch
python3 references/runtime/task_test.py --backend tcp --group A --task prepare \
  --tcp-host 127.0.0.1 --tcp-port 39001
# 其余任务 coverage-segment-1 / fault-exit / recovery / coverage-segment-2 / hold-end 同理
```

## mock 配置

`config/mock_backend.yaml`（旧闭环）与 `config/uav_mock_config.yaml`（单机 mock）支持按命令/策略覆写 `accept_delay_s`、`result_delay_s`、`send_accepted_feedback`、`final_status`、`failed_uavs`、`error_code`、`message` 等，用于覆盖成功、失败、超时、慢确认等场景。

## 关联文档

- 方案与 Phase 编号：`docs/tcp_to_ros_后续开发计划.md`
- 工程过程与回归记录：`docs/tcp_to_ros_工程过程.md`
- 验证结果：`docs/tcp_to_ros_测试记录.md`
- 会话交接：`handoff/handoff_swarm6-2_26080719.md`
