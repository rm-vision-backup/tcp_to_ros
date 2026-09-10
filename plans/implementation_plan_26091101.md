# 实施计划：原点校准集合参数化与实机多主部署适配

## 1. 目标

完成以下实机部署适配：

1. 将统一原点坐标参考的 UAV 集合改为参数。参数值使用执行层身份，例如：

   ```text
   UAV1 UAV2 UAV3 UAV4 UAV5 UAV6
   ```

   或：

   ```text
   UAV1 UAV2 UAV3
   ```

2. 更新 `swarm_topology_bridge/config/topology.yaml`，使其描述实机 GCS_A 与实际 UAV 的物理 IP；所有节点使用 `port_offset=0`。
3. 检查并补齐工作空间对实机部署的必要配置、launch、启动环境、MAVROS 接线和验收项。

实机网络地址约定：

```text
UAVn = 192.168.5.(10*n+1)
UAV1 = 192.168.5.11
UAV12 = 192.168.5.121
UAV16 = 192.168.5.161
GCS_A = 192.168.5.241
```

该地址约定用于实机 inventory、`uav_identity_map.yaml` 和实机 `topology.yaml`；实施前仍需现场逐机核对地址可达性。

地面站部署：

```text
runtime
 tcp_to_ros
swarm_topology_bridge
swarm_uav_interfaces
```

每台机载电脑部署：

```text
swarm_topology_bridge
swarm_uav_interfaces
swarm_uav_executor
ego_planner_driver
MAVROS
```

机载程序通过新增的 systemd service 配置启动；MAVROS 使用：

```text
/opt/ros/noetic/share/mavros/launch/px4.launch
```

## 2. 当前事实与问题

### 2.1 原点校准集合

`src/tcp_to_ros/scripts/gcs_a_calibration_node.py` 当前读取私有参数：

```text
~calibration_uavs
~all_uavs
~identity_map
```

但 `gcs_a_backend.launch` 没有向校准节点传入这些参数，运行时使用 Python 内置默认值：

```text
calibration_uavs = A01..A06
all_uavs         = A01..A15
identity_map     = A01->UAV1 ... A15->UAV15
```

实机身份范围为 A01..A16，其中 A16→UAV16 是备用机；仿真测试流程仍固定使用 GCS_A 与 UAV1–UAV15，不启动或依赖 UAV16。若任务中实际启用 A16，上游 runtime 通过身份映射将其分配至 UAV16——**runtime 侧 roster/配置扩展由上游负责人完成，不在本计划范围**；本计划只保证 `tcp_to_ros` 的 A16→UAV16 映射、解析与路由可用。

其中：

- `calibration_uavs` 决定参与 global/local pose 采样和原点/航向计算的 UAV；
- `all_uavs` 当前决定等待 `gp_origin_confirmed` 的 UAV 集合；
- 仿真和实机都不要求所有已登记 UAV 上电或响应，原点计算、发布和 ready 只要求 `calibration_uavs` 在线并完成必要采样；
- 原点发布后持续广播，后续上电或接入的 UAV 直接使用广播原点执行重置和确认；
- `all_uavs` 不再作为仿真或实机 ready 的全量阻塞集合，可作为当前在线/已知 UAV 清单或兼容参数保留；
- A01–A12 的任务编组和 A13–A15 reserve 不应被误认为原点计算集合。

仿真和实机共用正式身份文件 `src/tcp_to_ros/config/uav_identity_map.yaml`，统一扩展为 A01..A16。该文件只承担 tcp_to_ros 的任务层 Axx→执行层 UAVn 映射；机载 ROS 运行时使用 UAVn：

```yaml
uavs:
  - {uav_id: A01, exec_target: UAV1, ...}
  # ...
  - {uav_id: A16, exec_target: UAV16, ...}
```

A16 是实机备用机；仿真拓扑和仿真测试仍只启动 GCS_A 与 UAV1–UAV15。当前实际 EGO 机载运行记录显示 `uav_id=UAVn`、`exec_target=UAVn`；Axx 仅出现在 tcp_to_ros 的任务层 assignment 和身份映射中。校准节点当前需要的是 `{A01: UAV1, ...}` 形式，不能直接把完整身份 YAML 加载到 `~identity_map`。

### 2.2 topology 当前状态

`src/swarm_topology_bridge/config/topology.yaml` 当前只有 UAV6–UAV10 的五机示例：

- 没有 GCS_A；
- 没有完整 UAV1–UAV15；
- topics 不包含生产任务状态、轨迹 intent 和任务 service；
- services 只有 `/test_srv/UAV6..UAV10`；
- 仍是测试拓扑，不能支撑完整实机 Group A。

`topology_group_a_sim.yaml` 当前包含 GCS_A 与 UAV1–UAV15，使用 `127.0.0.1` 和仿真 `port_offset`；计划将仿真拓扑扩展到 GCS_A 与 UAV1–UAV16（UAV16 独立 `port_offset`、不启动，用途见 §4.2），但仿真测试流程仍只启动和使用 UAV1–UAV15。

### 2.3 实机网络与启动

`swarm_topology_bridge` 通过 ZeroMQ 跨物理机通信；每台机器仍需自己的 ROS Master。bridge 不设置或管理：

```text
ROS_MASTER_URI
ROS_IP
ROS_HOSTNAME
```

现有 `uav_offboard_ego.launch`、`startup_offboard_ego.sh` 仍包含 localhost、仿真 UDP、逐机 SITL Master 和 PX4 Gazebo 启动假设。工作空间当前没有机载 systemd service 配置；实机 Startup Applications 由现场手动设置，不纳入本计划。

实机 ROS 网络环境沿用系统安装后的默认值（`ROS_MASTER_URI`/`ROS_IP`/`ROS_HOSTNAME`，本机默认 Master 11311），该用法已在实机 `safe_valley_exp` 工作空间验证；仅仿真因单机模拟多机需要额外配置 11310–11325。本计划不要求实机额外设置这些变量。

机载实机入口需要独立于仿真入口，不能继续隐式加载仿真 topology。

现有 `uav_identity_map.yaml` 仅覆盖 A01–A15，源码计划需扩展 A16→UAV16；其中 UAVn 地址按 `192.168.5.(10*n+1)`，GCS_A 地址为 `192.168.5.241`。GCS_A IP 尚未纳入当前 bridge 实机拓扑。

## 3. 原点校准参数化

### 3.1 参数语义

**文件：**

- `src/tcp_to_ros/scripts/gcs_a_calibration_node.py`
- `src/tcp_to_ros/launch/gcs_a_backend.launch`
- 必要时新增 `src/tcp_to_ros/config/calibration_defaults.yaml`

统一采用执行层 UAV 名称作为校准节点输入：

```yaml
calibration_uavs: [UAV1, UAV2, UAV3, UAV4, UAV5, UAV6]
```

校准节点参数和 ROS topic/service 统一使用 UAVn。当前实际 EGO 机载测试中，`uav_id` 与 `exec_target` 均以 `UAVn` 输入；不得为机载程序或 bridge 增加 Axx 表达，Axx 只由 `tcp_to_ros` 处理上游 runtime 任务。若校准节点内部仍存在 Axx 变量，先核对是否只是任务层兼容代码；没有实际调用需求时删除或改为 UAVn。

参数必须支持任意非空、无重复的实际校准集合，例如 UAV1–UAV6 或 UAV1–UAV3。

`all_uavs` 单独表示已知或当前部署 UAV 集合。实机不要求所有已登记 UAV 上电或响应；原点计算、发布和 ready 只要求 `calibration_uavs` 在线并完成必要采样。原点发布后持续广播，后续上电或接入的 UAV 直接使用广播原点重置和确认。

ready 的判定集合只取必要集合（`calibration_uavs`）：当前实现用 `all_uavs` 建立 `confirmed` 并 `all(confirmed)` 判 ready，需同步改为按必要集合判定，避免实机分批上电时被未上电机阻塞。`all_uavs` 仅作已知/在线清单展示与兼容，不作为 ready 门槛。

实机存在 UAV 缺席、损坏或由 UAV16 替换的情况，因此校准流程必须按当前在场集合工作，不能无条件等待 A01..A16 全部响应。

仿真配置和仿真测试仍使用 GCS_A 与 UAV1–UAV15；A16 只作为实机身份映射和备用机部署对象，不得因身份映射扩展而被仿真拓扑、仿真 launch 或 15 机测试自动加入。注意区分执行层 UAV16：仿真 topology 含 UAV16（独立 `port_offset`、不启动，见 §4.2）用于验证 ready 不被缺席机阻塞，但仿真 launch/测试不启动它，任务层 A16 也不进入仿真。

当实际参与集合少于完整 16 架时，必须分别验证：`calibration_uavs` 在线且满足采样条件即可冻结原点、缺席 UAV 不阻塞 ready、runtime assignments 能只引用当前可用 Axx，以及 A16 替换任务能被正确解析和路由。

### 3.2 统一身份来源

校准节点和其他机载包的 ROS 运行身份统一使用 UAVn。`tcp_to_ros` 单独处理上游 runtime 的 Axx，并通过 `uav_identity_map.yaml` 转换为执行层 UAVn；Axx 不进入机载 ROS topic、service、launch 参数或业务消息。

校准节点读取正式身份配置中的 UAVn 连接信息，校准集合参数直接使用 UAVn。不新增机载侧 Axx 反向映射；只有审计发现现有源码仍有实际 Axx 依赖时，才单独处理该依赖。

启动时校验：

- `calibration_uavs` 每个 UAVn 成员存在于身份配置；
- `all_uavs` 每个 UAVn 成员存在于身份配置；
- `calibration_uavs` 必须是当前在线并参与计算的集合；
- `all_uavs` 可包含暂未上电的已知/部署 UAV，但不得作为仿真或实机 ready 的全量响应门槛；
- UAVn 名称无重复；
- `exec_target` 非空且唯一；
- 计算集合和确认集合不为空；
- 明确 `enabled` 对校准集合的作用，并在配置中显式体现，不静默混用。

### 3.3 launch 接线

在 `gcs_a_backend.launch` 的 `gcs_a_calibration` 节点作用域下显式加载：

```text
~calibration_uavs
~all_uavs
~identity_map 或身份文件路径
```

校准节点私有参数必须加载到该节点，而不是 backend 节点。校准节点的默认值不再作为生产部署的隐式配置来源。

日志/终端按实际参数动态输出，便于实机分批上电时人工核对：

- 计算集合；
- 航向误差阈值；
- 确认集合及确认数量；
- **逐机 ready 状态**：按 UAVn 输出已确认/未确认清单（含缺席机标记），供分批上电时人工确认任务相关 UAVn 是否都已 ready。

同步更新 `tcp_to_ros/README.md`、`tcp_to_ros/CONTEXT.md` 中将 A01–A06 写成固定实现的描述，改为“由 `calibration_uavs` 指定”；保留当前默认配置的示例值。

## 4. 实机 topology.yaml

### 4.1 实机 inventory

在生成 topology 前冻结一份实机清单，至少包含：

```text
name: GCS_A / UAV1 ... UAV16
uav_id: A01 ... A16（UAV 节点；A16 为实机备用机）
exec_target
hostname
physical_ip
ros_master_uri
mavros transport
mavros tgt_system
```

`uav_identity_map.yaml` 与 `topology.yaml` 中的 `exec_target`、物理 IP 必须一致。实机地址按以下约定使用：`UAVn = 192.168.5.(10*n+1)`，即 UAV1=`192.168.5.11`、UAV12=`192.168.5.121`、UAV16=`192.168.5.161`，GCS_A=`192.168.5.241`。GCS_A 地址必须明确加入 topology。实机地址核对由现场完成，不在本计划中展开部署步骤。

### 4.2 topology 内容

将 `src/swarm_topology_bridge/config/topology.yaml` 更新为实机生产配置，覆盖：

- GCS_A=`192.168.5.241`；
- 实际部署的 UAV 节点，实机身份范围为 UAV1–UAV16，其中 UAV16 是备用机；
- 仿真 topology 同样包含 GCS_A 与 UAV1–UAV16，但仿真测试流程仍只启动和使用 GCS_A 与 UAV1–UAV15；其中仿真 UAV16 必须给独立 `port_offset: 1600`（不得用默认 0——与 GCS_A 同 IP、同 offset 会端口重叠并被误识别为 GCS_A 的数据），其用途是以 UAV16 缺席验证“统一原点不因未上电机阻塞 ready”；任务层 A16 不进入仿真 topology/launch；
- 物理 IP；
- 所有实机节点 `port_offset: 0`；
- 冻结端口：`base_port: 4200`、`service_base_port: 14000`（与仿真一致，已实机验证无冲突；防火墙按对应 TCP 范围放行）。

topics 至少覆盖当前生产链路需要的：

```text
mavros/state
mavros/time_reference
mavros/global_position/global
mavros/local_position/pose
mavros/local_position/odom
mavros/rc/in
uav_task_state
trajectory_intent
gp_origin
gp_origin_confirmed
gps_bias
```

实际 topic 名称、类型和 GCS_A/UAV 节点接收关系以当前 bridge、backend、executor、planner 的订阅/发布代码为准，不能从测试服务列表推导。

services 至少覆盖每架 UAV 的生产服务：

```text
/UAVn/uav_task
/UAVn/uav_task_control
/UAVn/uav_hold
```

测试用 `/test_srv/UAVn` 不作为生产 topology 内容。

`topology` 接收矩阵必须覆盖 GCS_A 与实际 UAV 间的必要业务流。默认先复用当前 Group A 的全互收语义，再按实机带宽/CPU 测量结果冻结；每个 `topology` key 必须在 `uavs[].name` 中存在，不能遗漏 GCS_A 或产生孤立节点。

### 4.3 bridge 启动

确认 `swarm_topology_bridge` 每台机器启动一个 bridge，并使用统一实机 `topology.yaml`。仿真与实机的身份启动方式不同：

- 仿真启动脚本循环启动 UAV1–UAV15，并向每个 bridge 传入对应身份和仿真端口参数；
- 实机每台电脑只启动本机程序，机载 bridge 按 hostname 判断本机身份（**机载电脑 hostname 固定为 `UAVn`，为部署环境条件，已在实机 `safe_valley_exp` 工作空间验证；因此不需要额外 `uav_name` 参数**）；实机启动脚本不循环启动其他 UAV；
- GCS_A bridge 使用 GCS_A 主机身份；机载 bridge 使用各自的 UAV hostname。hostname 必须能唯一匹配 topology 中的节点，无法唯一匹配时启动失败；
- `topology.yaml`、hostname 和身份映射中的节点名称必须一致，避免同一物理主机被识别为错误 UAV。

仿真中 GCS_A bridge、backend、calibration 位于同一 ROS Master（11310），每架机载 bridge、MAVROS、executor、planner 位于对应的独立 ROS Master（11311–11325）。该结构用于模拟实机每台电脑一个本地 ROS Master。

实机中地面站和每台机载电脑均只运行自身本地 ROS Master，使用默认端口 11311；不在机载启动脚本中按仿真方式额外设置 11311–11325，也不循环启动其他无人机。每台机载 bridge 与本机 MAVROS、executor、planner 连接本机 ROS Master；GCS_A bridge 与 GCS_A backend/calibration 连接地面站本机 ROS Master。

仿真 `topology_group_a_sim.yaml` 保留为仿真配置，不与实机 `topology.yaml` 混用。

## 5. 实机 launch 与 MAVROS

### 5.1 GCS_A

补充或整理一个明确的 GCS_A 实机启动入口，组合：

```text
swarm_topology_bridge
 gcs_a_calibration_node
 gcs_a_backend_node
 tcp_server/runtime 对接所需节点
```

backend、calibration 和 bridge 使用同一 GCS_A ROS Master；bridge 必须先于依赖远端 topic/service 的业务节点进入就绪状态。

### 5.2 机载电脑

新增独立实机入口：`src/swarm_uav_executor/launch/uav_offboard_ego_real.launch`。该文件不与仿真 `uav_offboard_ego.launch` 混用，负责启动单台实机的机载程序，并将实机 `topology.yaml` 透传给 bridge；不能继续让实机入口默认加载仿真 topology。

补充实机参数化入口，组合：

```text
MAVROS（include px4.launch）
gp_origin_receiver
gps_bias_node
swarm_topology_bridge
ego_planner_driver
pose/setpoint relay
swarm_uav_executor
```

实机机载程序沿用 hostname 判断本机身份，不把 `uav_id=Axx` 作为 ROS 内或跨 ROS 的启动参数。当前实际 EGO 机载测试记录中，`uav_id` 与 `exec_target` 均为 `UAVn`；Axx 只由 `tcp_to_ros` 处理上游 runtime 任务。

保留本机默认 ROS topic 契约（`/mavros/*`、`/local_pose`、setpoint 唯一出口），邻机数据由 bridge 重发布。

MAVROS 使用 `/opt/ros/noetic/share/mavros/launch/px4.launch`，沿用已配好的默认参数和实机连接配置；不在本计划中重复显式配置默认值。仅检查现有机载启动入口是否正确 include 该 launch，并保证实际部署使用的 FCU 连接参数不会继续沿用 SITL 的 localhost/仿真 UDP 假设。

## 6. 开机自启动

为 `swarm_uav_executor/services/` 增加 systemd service 配置文件和 README，说明配置安装、`start`、`enable`、`status`、`stop`、`disable` 等操作。实机上的 Startup Applications 由现场手动设置，不纳入本计划。

实机启动脚本只启动本机的机载程序，不循环启动其他 UAV。机载程序启动顺序沿用仿真中单架 UAV 的顺序：MAVROS、原点接收器、GPS bias、bridge、ego planner、pose/setpoint relay、executor；实际依赖关系以现有 launch 为准。地面站启动 GCS_A bridge、calibration、backend 和 runtime/tcp_to_ros。

地面站和每台机载电脑均使用本机默认 ROS Master 端口 11311；不在实机脚本中按仿真方式分配 11310–11325。ROS Master、hostname 身份发现和 bridge 连接使用系统已有默认配置。

启动脚本至少负责：

1. service 使用固定的本机工作空间和 ROS 环境；
2. 按仿真中单架 UAV 的机载程序顺序启动本机 MAVROS、原点接收器、GPS bias、bridge、ego planner、relay 和 executor；
3. 保存 stdout/stderr 和启动失败原因；
4. 通过 systemd 提供 `start`、`enable`、`status`、`stop`、`disable` 及失败重启策略。

开机自启动不自动执行 arm、OFFBOARD、起飞或任务 START。当前 takeoff 程序尚未适配实机部署，实机测试阶段暂时使用遥控器完成 arm 与 OFFBOARD；后续再单独适配实机 takeoff 流程。

现有 `startup_offboard_ego.sh` 仅作为 15 机 SITL 循环启动参考，不能直接作为实机启动脚本。

## 7. backend、身份和部署路径检查

检查并按实机路径修正：

- `gcs_a_backend_node.py` 的 identity map 默认路径，使用 ROS package 路径或要求 launch 显式传入；不依赖工作区绝对路径；
- `diagnostic_log_dir`（`uav_offboard_ego.launch:26-27`、新增 `uav_offboard_ego_real.launch`、`ego_planner_driver.launch:30`）不得硬编码固定 home 绝对路径；仿真入口维持当前工作空间约定，实机入口由 launch arg/环境给出（默认 `$HOME/catkin_swarm6-2/runtime_logs/ego_planner`）；
- backend 的 Axx→UAVn 映射与 topology IP/exec_target 一致；A01–A16 均可解析，A16→UAV16 仅在实机备用/替换任务中使用；
- GCS_A bridge 重发布的 `/UAVn/uav_task_state`、pose、MAVROS state 与 backend 订阅一致；
- 每架机的 `/UAVn/uav_task`、`/UAVn/uav_task_control`、`/UAVn/uav_hold` 由本机 executor 注册并被 bridge 代理；
- `gp_origin_receiver` 的固定 `/GCS_A/group_a/gp_origin` 契约与 GCS_A bridge/校准发布一致；
- `swarm_uav_interfaces`、`mavros_msgs`、`geographic_msgs`、Python ZeroMQ 依赖在 GCS_A 和全部机载电脑安装且版本一致。

## 8. 工作空间内测试

本计划修改完成后，通过工作空间已有的自动测试和新增针对性测试验证：

- 校准参数读取、集合校验、UAVn 身份配置和部分 UAV 在场场景；
- A01–A16 身份映射与 A16→UAV16 替换任务解析；
- 仿真仍只使用 GCS_A 与 UAV1–UAV15；
- 实机 topology 的 GCS_A、UAV1–UAV16、物理 IP、`port_offset=0`、生产 topic/service 和拓扑 key 完整性；
- hostname 身份发现、单机默认 ROS Master、systemd service 配置和本机启动顺序；
- MAVROS `px4.launch` include 与现有 launch 参数接线；
- 仅 `calibration_uavs` 在线并响应时即可完成原点计算、发布和 ready；
- 终端/日志按 UAVn 输出逐机 ready（已确认/未确认）状态，实机分批上电时可人工核对任务相关 UAVn 均已 ready；
- 原点发布持续存在，后续接入 UAV 可完成原点重置和确认；
- 缺席 UAV 不因未上电而阻塞 ready 或当前任务；仿真以“拓扑含 UAV16 但不启动”复现该场景；
- A16 替换实际任务成员时，runtime 在 tcp_to_ros 内完成 A16→UAV16 映射，机载 ROS 侧只接收和使用 UAV16；bridge 和 executor 路由一致；
- 校准默认值、launch/include、bridge config 透传和 MAVROS include 修改后，运行官方 15 机 prepare SITL 回归，确认 GCS_A 与 UAV1–UAV15 链路、原点校准、任务执行和最终结果无非预期回归。

实机飞行、遥控器 arm/OFFBOARD、上桨测试及其现场日志不纳入本计划的工作空间测试验收。

## 9. 修改文件范围

预计涉及：

```text
src/tcp_to_ros/scripts/gcs_a_calibration_node.py
src/tcp_to_ros/launch/gcs_a_backend.launch
src/tcp_to_ros/config/uav_identity_map.yaml（扩展 A16→UAV16）
src/tcp_to_ros/scripts/gcs_a_backend_node.py
src/tcp_to_ros/README.md
src/tcp_to_ros/CONTEXT.md
src/tcp_to_ros/test/test_calibration.py
src/tcp_to_ros/test/test_parameter_wiring.py
src/ego_planner_driver/launch/ego_planner_driver.launch（诊断日志路径参数化）
src/swarm_topology_bridge/config/topology.yaml
src/swarm_topology_bridge/launch/（实机入口或参数接线）
src/swarm_topology_bridge/README.md
src/swarm_uav_executor/launch/uav_offboard_ego.launch（仿真入口，保持独立）
src/swarm_uav_executor/launch/uav_offboard_ego_real.launch（新增实机单机入口，透传实机 topology config）
src/swarm_uav_executor/services/（systemd service 与安装说明）
```

仅在审计确认存在实际接线缺口时修改对应文件；仿真 spawn、普通任务编组、yield 优先级和 planner 避碰算法不因本计划改变。
