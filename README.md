# tcp-to-ros

`tcp_to_ros` 是 Group A 地面站的 ROS1 Noetic 主功能包：作为 TCP Server 接收 `references/runtime` 的整组批次命令，经本地 `ExecuteGroupTask` Action 拆分，通过通用通信包 `swarm_topology_bridge` 的跨机 Service / Topic，把单机任务可靠分发到各 UAV（GCS_A）并汇总整组结果。本包 `package.xml` / `CMakeLists.txt` 已声明对 `swarm_topology_bridge` 的依赖。

## 组成与文件

- `action/ExecuteGroupTask.action`：GCS 本地整组任务 Action（goal 为整组 `assignments`，feedback/result 表示整组 ACCEPTED/COMPLETED/FAILED）。
- `msg/GroupTaskState.msg`：GCS 本地整组状态聚合；共享位姿、assignment 与单机状态由 `swarm_uav_interfaces` 定义。
- `srv/EmergencyHold.srv`：GCS 本地整组紧急悬停；跨机 `UavTask`/`UavHold` 由 `swarm_uav_interfaces` 定义。
- `scripts/`：
  - `tcp_server_node.py`：TCP Server，仅做协议适配，把整组命令交给 Action；不承载逐机路由与聚合。
  - `gcs_a_backend_node.py`：GCS_A 分发后端（`ExecuteGroupTask` Action server）：按身份映射拆分单机任务、并发 Service 确认（单次 1.0s、首次+2 重试、0.2s 间隔、4s 预算）、订阅并聚合单机状态、至少一次请求+幂等、超时/cancel/HOLD、`ALL` 失败即停。
  - `gcs_a_calibration_node.py`：使用 A01-A12 arm 前全球位置中位数与圆周平均航向冻结共享坐标；向 A01-A15 下发原点并等待 MAVROS `gp_origin` 回读，15/15 确认后才开放任务。
  - 单机 mock 与 direct MAVROS 执行适配由独立包 `swarm_uav_executor` 提供，避免地面站包拥有机载控制语义。
  - `mock_backend_node.py` / `task_event_injector.py`：早期整组 mock 与手动 HOLD 注入（旧闭环，仍在但分发层已改用 `gcs_a_backend_node.py`）。
  - `tcp_to_ros/`：`bridge.py`（TCP→Action 转换）、`protocol.py`（JSON 校验）、`dedup.py`、`state.py`、`identity.py`（身份映射加载）。
- `config/`：`uav_identity_map.yaml`（A01..A15→UAV1..UAV15/hostname/IP，待复核）、`formation_offsets.yaml`（固定 leader、两个编队槽位与 mission anchor）、`topology_group_a_sim.yaml`（Group A 多 Master 桥接/服务代理配置）、`mock_backend.yaml`。
- `launch/`：`gcs_a_backend.launch`（仅 GCS_A 后端）、`gcs_a_backend_test.launch`（GCS_A 后端 + UAV1..3 mock，单 Master 语义联调）、`gcs_a_backend_fail.launch`（含失败注入）、`tcp_to_ros_mock.launch`（旧 mock 闭环）、`topology_group_a_sim.launch`（Group A 多 Master 仿真拓扑）。
- `test/test_gcs_a_backend_client.py`：集成/回归测试客户端。

## 部署位置与最小功能包集合

`src/` 下与 Group A 正式链路相关的功能包按运行位置部署如下。GCS_A 与每台 UAVn 使用独立 ROS Master 时，每台主机都应在本机工作空间完成构建并 source 对应的 `devel/setup.bash`；不能只在 GCS_A 构建后通过网络共享 ROS 类型。

| 功能包 | GCS_A 地面站 | 每台 UAVn 机载电脑 | 部署职责 |
|---|---:|---:|---|
| `tcp_to_ros` | 必需 | 不需要 | 运行 TCP Server、GCS_A 整组 Action 后端、身份映射、逐机链路健康监测、飞行安全租约协调和整组结果聚合。 |
| `swarm_uav_executor` | 不需要 | 必需 | 每机运行一个绑定本机 `(uav_id, exec_target)` 的任务执行器，提供任务/HOLD/安全租约 Service、状态 Topic 和本地租约 watchdog；通过本机 MAVROS driver 控制本机。 |
| `swarm_topology_bridge` | 必需 | 必需 | 在 GCS_A 和每台 UAVn 的 ROS Master 上各运行一个 bridge 实例，按同一 Group A 拓扑转发 Topic，并代理跨 Master Service。 |
| `swarm_uav_interfaces` | 必需 | 必需 | 提供两端共同使用的 ROS msg/srv 契约；所有 GCS_A、bridge 和 UAVn 主机必须安装、构建并 source 完全相同的接口版本。 |

正式部署的最小包集合为：

```text
GCS_A:
  tcp_to_ros
  swarm_topology_bridge
  swarm_uav_interfaces

每台 UAVn:
  swarm_uav_executor
  swarm_topology_bridge
  swarm_uav_interfaces
```

补充说明：

- MAVROS/PX4 及其系统依赖不属于上述四个自研包，但 direct MAVROS executor 运行时必须在对应 UAVn 机载电脑提供本机 MAVROS/PX4 链路；GCS_A 只订阅经 bridge 转发后的 `/{exec_target}/mavros/state`，不直接控制远端 MAVROS。
- `tcp_to_ros/launch/topology_group_a_sim.launch` 是为了在同一开发机上编排多个 ROS Master、bridge 和 mock executor 的联调入口，因此模拟 UAV 角色时会从 `tcp_to_ros` 启动测试节点。该仿真便利入口不代表实机 UAVn 需要安装或运行 `tcp_to_ros`；实机应分别从 `swarm_topology_bridge` 和 `swarm_uav_executor` 启动机载组件，并加载部署用 Group A 拓扑配置。
- `safe_valley_exp` 不属于 Group A 正式链路的必需部署集合。它只是应用示例，用于提供无人机机载 offboard 程序以及仿真/实机兼容参考；不得把它视为 `tcp_to_ros`、安全租约闭环或 `swarm_uav_executor` 的运行依赖。若某架 UAV 选择复用其中的示例算法，应作为独立机载应用按其自身文档部署，并继续遵守单机控制权和现场安全门禁。

## 构建

只使用 `catkin build`，默认 `build/devel/logs`：

```bash
source ~/.bashrc
catkin build tcp_to_ros
source /home/ub20tg/catkin_swarm6-2/devel/setup.bash
```

若默认 `build`/`devel`/`logs` 被旧 `catkin_make` 产物占用，先删除并重新初始化 `src` 后编译。

### 环境变量（ROS 日志重定向）

为避免 ROS 运行日志散落在 `~/.ros/log`，本工作空间统一将运行日志重定向到 `.ros_home/`，启动任何 ROS 节点前请先执行：

```bash
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
mkdir -p "$ROS_LOG_DIR"
```

确保当前工作空间所有 ROS 运行日志均写入 `.ros_home/log`。

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

> 真实 MAVROS/PX4 验证（2026-08-10，操作员控制飞行、经授权切 OFFBOARD）：`MOVE_TO` 完成并到达、UAV1 bridge 断链注入触发 `HEALTHY→SUSPECT→LOST` 与本地租约过期 HOLD、链路恢复后旧租约锁存（RENEW/START/END 全拒）、地面安全 END 门禁全套通过。原始证据见 `docs/tcp_to_ros_测试记录_早期.md`。

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

- 依赖解析：跨 Master 的 `UavTask`/`UavTaskControl`/`UavHold` Service，以及任务状态、MAVROS state/global/local pose、原点回读 Topic，由 `swarm_topology_bridge` 通用桥转发；
- 端口规划与命名约定见 `config/topology_group_a_sim.yaml`（port_offset 区分单机多 Master，实机 port_offset=0）。

正式任务启动顺序为：机载程序在线且人工用 QGC 确认 PX4/RTK → 启动 GCS_A → 完成 12 机原点/航向冻结与 15 机原点回读 → TCP 上游发送任务。TCP 任务坐标按右-前-上解释，`mission_enu=(10,5,0)` 映射到冻结全球中位点；runtime yaw 和 runtime 编队形状不参与执行。

### 3. 与 runtime 的 TCP 联调（旧 mock 闭环）

```bash
roslaunch tcp_to_ros tcp_to_ros_mock.launch
python3 references/runtime/task_test.py --backend tcp --group A --task prepare \
  --tcp-host 127.0.0.1 --tcp-port 39001
# 其余任务 coverage-segment-1 / fault-exit / recovery / coverage-segment-2 / hold-end 同理
```

## mock 配置

`config/mock_backend.yaml` 保留早期整组 mock；当前单机 mock 由 `swarm_uav_executor` 提供，可通过 launch 参数配置接受/执行延时、终态、错误码与 HOLD 行为，用于成功、失败、超时和安全路径测试。

## 关联文档

- 方案与 Phase 编号：`plans/README.md`
- 工程过程与回归记录：`docs/tcp_to_ros_工程过程.md`
- 验证结果：`docs/tcp_to_ros_测试记录_早期.md`
- 冻结实施计划：`plans/`
- 会话交接：`handoffs/`
