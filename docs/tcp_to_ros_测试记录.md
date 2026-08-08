# tcp_to_ros 测试记录

## 文档说明

本文按时间追加保存验证证据。开头“测试范围/当前限制/下一阶段进入条件”仅描述第一版 mock 闭环的历史基线，不代表当前项目状态；当前能力与边界以文末最新日期章节为准。

## 目标

- 记录 `tcp_to_ros` 第一版适配层联调结果；
- 确认 Group A TCP Server 与 ROS 接口闭环可用；
- 作为后续接入 `swarm_topology_bridge` 与机载执行层的基线记录。

## 测试范围

本轮测试只覆盖 `src/tcp_to_ros` 第一版 mock 闭环：

- Group A TCP Server
- ROS Action `/group_a/execute_task`
- ROS Topic `/group_a/group_task_state`
- ROS Service `/group_a/emergency_hold`
- mock backend 自动回复

本轮测试不覆盖：

- 真实无人机控制
- `safe_valley_exp` 机载算法执行判据
- Group A 与 Group B 地面站互联
- 基于 `swarm_topology_bridge` 的跨机任务分发

## 测试环境

- 工作空间：`/home/ub20tg/catkin_swarm6-2`
- ROS 版本：Noetic
- 构建方式：`catkin build`
- source 方式：
  - `source ~/.bashrc`
  - `source /home/ub20tg/catkin_swarm6-2/devel/setup.bash`
- 启动方式：
  - `roslaunch tcp_to_ros tcp_to_ros_mock.launch`

## 测试步骤

### 正向测试方法

目的：验证 `tcp_to_ros` 是否正确承接 Group A 的 6 个 TCP 任务、mock backend 与 Action/Topic/Service 主链路，并让 `references/runtime/task_test.py` 返回 `SUCCESS`。

前提：先完成构建与启动（`roslaunch` 会自动启动 `rosmaster`，无需手动 `roscore`；默认监听 `127.0.0.1:39001`）：

```bash
cd /home/ub20tg/catkin_swarm6-2
source ~/.bashrc
catkin build tcp_to_ros
source /home/ub20tg/catkin_swarm6-2/devel/setup.bash
roslaunch tcp_to_ros tcp_to_ros_mock.launch
```

另开终端执行：

```bash
cd /home/ub20tg/catkin_swarm6-2
source ~/.bashrc
source /home/ub20tg/catkin_swarm6-2/devel/setup.bash

python3 references/runtime/task_test.py --backend tcp --group A --task prepare --tcp-host 127.0.0.1 --tcp-port 39001
python3 references/runtime/task_test.py --backend tcp --group A --task coverage-segment-1 --tcp-host 127.0.0.1 --tcp-port 39001
python3 references/runtime/task_test.py --backend tcp --group A --task fault-exit --tcp-host 127.0.0.1 --tcp-port 39001
python3 references/runtime/task_test.py --backend tcp --group A --task recovery --tcp-host 127.0.0.1 --tcp-port 39001
python3 references/runtime/task_test.py --backend tcp --group A --task coverage-segment-2 --tcp-host 127.0.0.1 --tcp-port 39001
python3 references/runtime/task_test.py --backend tcp --group A --task hold-end --tcp-host 127.0.0.1 --tcp-port 39001
```

预期：每条命令退出码为 `0`、输出 JSON 的 `status` 为 `SUCCESS`、`failed_robot_ids` 为空。

### 正向测试结果

| 测试项 | 预期 | 实际结果 | 是否通过 | roslaunch log |
|---|---|---|---|---|
| prepare | `SUCCESS` | `SUCCESS` | 是 | `accepted runtime connection from ('127.0.0.1', 37994)` |
| coverage-segment-1 | `SUCCESS` | `SUCCESS` | 是 | `accepted runtime connection from ('127.0.0.1', 48054)` |
| fault-exit | `SUCCESS` | `SUCCESS` | 是 | `accepted runtime connection from ('127.0.0.1', 38084)` |
| recovery | `SUCCESS` | `SUCCESS` | 是 | `accepted runtime connection from ('127.0.0.1', 38606)` |
| coverage-segment-2 | `SUCCESS` | `SUCCESS` | 是 | `accepted runtime connection from ('127.0.0.1', 54030)` |
| hold-end | `SUCCESS` | `SUCCESS` | 是 | `accepted runtime connection from ('127.0.0.1', 38308)` |


### 负向测试方法

目的：验证协议拒绝路径、`command_id` 去重缓存与 `HOLD` 对活动任务的打断逻辑。

前提：同样先启动 `tcp_to_ros_mock.launch`。

另开终端执行：

```bash
cd /home/ub20tg/catkin_swarm6-2
source ~/.bashrc
source /home/ub20tg/catkin_swarm6-2/devel/setup.bash

python3 src/tcp_to_ros/test/test_wrong_group.py
python3 src/tcp_to_ros/test/test_duplicate_command.py
python3 src/tcp_to_ros/test/test_hold_interrupt.py
```

预期：

| 脚本 | 预期输出 |
|---|---|
| `test_wrong_group.py` | 退出码 `0`；返回 `FAILED`；`error_code=WRONG_GROUP` |
| `test_duplicate_command.py` | 退出码 `0`；第一次 `ACCEPTED`，第二次 `COMPLETED`，第三次缓存终态 `COMPLETED` |
| `test_hold_interrupt.py` | 退出码 `0`；`FOLLOW_ROUTE` 先 `ACCEPTED`，`HOLD` 返回 `COMPLETED`，原任务再次查询返回 `FAILED` 且 `error_code=COMMAND_HELD` |

### 负向测试结果

| 测试项 | 预期 | 实际结果 | 是否通过 | roslaunch log |
|---|---|---|---|---|
| 错误 group_id | `FAILED / WRONG_GROUP` | `FAILED & WRONG_GROUP` | 是 | `accepted runtime connection from ('127.0.0.1', 39254)` |
| 重复 command_id | 返回缓存终态 `COMPLETED` | `ACCEPTED` - `COMPLETED` - `COMPLETED` | 是 | `accepted runtime connection from ('127.0.0.1', 51180)` |
| HOLD 打断活动任务 | `HOLD=COMPLETED`，原任务 `FAILED / COMMAND_HELD` | `ACCEPTED` - `COMPLETED` - `FAILED & COMMAND_HELD` | 是 | `accepted runtime connection from ('127.0.0.1', 35944)` |

## 结论

本轮测试通过，说明 `tcp_to_ros` 第一版已经完成以下能力验证：

- 能正确接收 Group A 整组 TCP 命令；
- 能完成协议校验、`group_id` 校验与 `command_id` 去重；
- 能将整组任务转换到 ROS 接口并得到 mock backend 响应；
- 能对 `HOLD` 和活动任务打断做出符合预期的状态处理；
- 能向 runtime 返回符合文档约定的 `ACCEPTED / COMPLETED / FAILED`。

当前结论只说明 **TCP-ROS 适配层闭环成立**，不代表真实机载执行链路已完成。

## 当前限制

以下为第一版 mock 闭环完成时的历史限制；当前状态以文末最新验证章节为准。

- 仍未接入真实机载程序；
- 当前成功判据来自 mock backend，不来自 MAVROS 或机载动作层；
- 地面站尚未通过 `swarm_topology_bridge` 向各 UAV 分发单机任务；
- 还没有 A01 到 `UAV*` / hostname 的正式映射配置。

## 下一阶段进入条件

进入下一阶段开发前，需要明确：

1. 地面站在 ROS 网络中的节点身份，建议使用 `GCS_A`；
2. `A01 ... A15` 到 `UAV1 ... UAV15` / hostname 的映射表；
3. 单机统一任务接口字段；
4. 单机状态回传字段；
5. `swarm_topology_bridge` 的 Group A 地面站拓扑配置。

---
---

# Phase 1-3 集成验证记录（本会话）

## 测试范围

在上一轮 mock 闭环基线之上，验证 GCS_A 整组分发后端、单机 mock 执行适配层与跨 Master Service 代理：

- `src/tcp_to_ros/scripts/gcs_a_backend_node.py`
- `src/tcp_to_ros/scripts/uav_mock_node.py`
- `src/swarm_topology_bridge/scripts/bridge_node.py`（Topic 限频修正 + Service 代理）

## 测试步骤

### 单 Master 语义联调 测试方法

在**一个** ROS Master 内同时启动 GCS_A 分发后端与 UAV1..3 的机载 mock（`gcs_a_backend_test.launch` 会拉起 `gcs_a_backend_node` + 3 个 `uav_mock_node`）。`roslaunch` 会自动启动 `rosmaster`，无需手动 `roscore`。

**终端 1（启动）**：

```bash
source ~/.bashrc
source /home/ub20tg/catkin_swarm6-2/devel/setup.bash
roslaunch tcp_to_ros gcs_a_backend_test.launch
```

**终端 2（运行集成测试客户端）**：

```bash
source ~/.bashrc
source /home/ub20tg/catkin_swarm6-2/devel/setup.bash
cd /home/ub20tg/catkin_swarm6-2

# 1. 正常整组 MOVE_TO → 期望 COMPLETED
python3 src/tcp_to_ros/test/test_gcs_a_backend_client.py --command-id ver-ok-001 \
  --timeout 15 --expect COMPLETED

# 2. 相同 command_id 再发一次 → 期望仍 COMPLETED（幂等，不重新执行）
python3 src/tcp_to_ros/test/test_gcs_a_backend_client.py --command-id ver-ok-001 \
  --timeout 15 --expect COMPLETED
```

**终端1**

```bash
# 3. 失败即停：改用注入 UAV2 失败的 launch（UAV2 mock final_status=FAILED，UAV1/UAV3 慢执行）
roslaunch tcp_to_ros gcs_a_backend_fail.launch
```
**终端2**
```bash
python3 src/tcp_to_ros/test/test_gcs_a_backend_client.py --command-id ver-fail-001 \
  --timeout 15 --expect FAILED
```

参数说明：`--command-id` 指定整组命令 ID（相同 ID 触发幂等缓存）；`--timeout 15` 是测试客户端等待上限；`--expect` 是断言的目标终态（`COMPLETED`/`FAILED`）。

**预期输出**：

- 1、2 均打印 `RESULT_FINAL=COMPLETED`、`RESULT_COMPLETED=A01,A02,A03`（正常整组 + 重复幂等）；
- 3 打印 `RESULT_FINAL=FAILED`、`RESULT_FAILED=A02`、`RESULT_ERROR=MOCK_UAV2_FAIL`，且 GCS_A 对活动 UAV（A01/A03）下发 HOLD 并逐机确认。

### 单 Master 测试结果

| 场景 | 预期 | 实际结果 | 是否通过 |
|---|---|---|---|
| 正常整组 MOVE_TO | `COMPLETED`，completed=A01,A02,A03 | `RESULT_FINAL=COMPLETED`，completed_uavs=A01,A02,A03 | 是 |
| 重复 `command_id` | 返回缓存 `COMPLETED`，不重新执行 | 第二次仍 `COMPLETED`（UAV 幂等缓存 + GCS_A ACCEPTED 前终态缓冲回放）| 是 |
| 单机失败即停 | 整组 `FAILED`，failed_uavs=A02，错误码透传，活动 UAV 被 HOLD | `RESULT_FINAL=FAILED`，failed_uavs=A02，error=`MOCK_UAV2_FAIL`，A01/A03 HOLD 确认成功 | 是 |


### 多 Master 跨机 Service 代理 测试方法

先停掉上一节的单 Master launch（`Ctrl-C`），再开两个终端分别作为 UAV1 与 GCS_A 的 Master。关键点：每个 Master 各自 `source` 并设置 `ROS_MASTER_URI`；UAV1 以 `uav_name:=UAV1` 启动（bridge + 机载 mock 一起），GCS_A 以 `run_uav_mock:=false` 启动（只跑 bridge），并**再单独**启动 GCS_A 分发后端 `gcs_a_backend.launch`。

**终端 1（UAV1 机载 Master，端口 11312）：**

```bash
source ~/.bashrc
source /home/ub20tg/catkin_swarm6-2/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11312
roslaunch tcp_to_ros topology_group_a_sim.launch uav_name:=UAV1 uav_id:=A01
```

**终端 2（GCS_A 地面站 Master，端口 11311）：**

```bash
source ~/.bashrc
source /home/ub20tg/catkin_swarm6-2/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311

# 1) GCS_A 侧桥（仅 bridge；机载 mock 由 UAV1 终端提供）
roslaunch tcp_to_ros topology_group_a_sim.launch uav_name:=GCS_A run_uav_mock:=false
# 2) 另起 GCS_A 分发后端（同 Master；因上面 roslaunch 占用前台，另开终端 shell: 保持 ROS_MASTER_URI 后再启动）
roslaunch tcp_to_ros gcs_a_backend.launch
```

**验证（在 GCS_A Master，即终端 2 所在）另开终端：**

```bash
export ROS_MASTER_URI=http://localhost:11311
# 应见 6 个代理：/UAV1..3/uav_task 与 /UAV1..3/uav_hold
rosservice list | grep uav_                  # 或 rosservice list | grep -E "uav_task|uav_hold"
# 跨 Master 调用 UAV1 的 uav_task，应返回 accepted=True / status=ACCEPTED
rosservice call /UAV1/uav_task "protocol_version: '1.0'
mission_id: 'mm-001'
group_id: 'GroupA'
command_id: 'mm-cmd-001'
uav_id: 'A01'
exec_target: 'UAV1'
command: 'MOVE_TO'
timeout_s: 10.0
leader_id: ''
assignment:
  uav_id: 'A01'
  formation_follow: false
  target_id: ''
  target_pose: {x: 1.0, y: 2.0, z: 3.0, yaw: 0.0}
  waypoints: []"
```

**预期**：

- `rosservice list` 在 GCS_A Master 上列出 `/UAV1..3/uav_task`、`/UAV1..3/uav_hold` 六个代理（bridge 按 `topology_group_a_sim.yaml` 的 `services` 声明动态注册）；
- `rosservice call /UAV1/uav_task` 经 GCS_A→bridge DEALER→ZMQ→UAV1 ROUTER→uav_mock_node 返回 `success: True`、`status: ACCEPTED`（即下表「多 Master 跨机 测试结果」中的 `accepted: True / status=ACCEPTED`）。

### 多 Master 跨机 测试结果

| 场景 | 预期 | 实际结果 | 是否通过 |
|---|---|---|---|
| GCS_A bridge 注册 `/UAV1..3/uav_task`、`/UAV1..3/uav_hold` 代理 | 6 个 Service 代理可用 | `rosservice list` 显示全部 6 个代理 | 是 |
| GCS_A `rosservice call /UAV1/uav_task` | 转发到 UAV1 mock，返回 accepted=true | `accepted: True`，status=`ACCEPTED` | 是 |

## 过程中修复的问题

1. rospy `ServiceProxy` 无原生调用超时且不允许位置+关键字混用：改为工作线程 + Event 软超时。
2. 幂等重复请求的缓存终态可能早于 `accepted_sent`：GCS_A 缓冲 ACCEPTED 前到达的终态并在 ACCEPTED 后回放。
3. `UavHold` 响应无 `status` 字段：mock 改用 `UavHoldResponse` 构造。
4. bridge 启动崩溃：`roslib.services` 不存在，改用 `roslib.message.get_service_class`。
5. bridge Service 载荷反序列化：`genpy.Message.deserialize` 需 bytes，不能用 `BytesIO`。
6. 依赖方向修正：`swarm_topology_bridge` 为通用包，移除其对 `tcp_to_ros` 的依赖；Group A 联调示例移入 `tcp_to_ros` 包；`tcp_to_ros`/`safe_valley_exp` 依赖 `swarm_topology_bridge`。

## 待后续覆盖

- 状态 Topic 跨 Master 转发（UAV 状态经 bridge 到 GCS_A 聚合）完整闭环；
- 完整 4 秒确认边界（慢确认拒绝/重试耗尽）；
- cancel 与 TCP 断线触发 HOLD 的端到端验证；
- 弱网/重连下 bridge Service 代理的超时与迟响应。

## 关联文档

- 测试步骤：`tcp_to_ros_测试步骤.md`
- 工程过程：`tcp_to_ros_工程过程.md`
- 后续开发计划：`tcp_to_ros_后续开发计划.md`

---

# Phase 4 首版与 Phase 5 安全边界回归（2026-08-08）

## 实施范围

- 共享 ROS 类型迁移到独立 `swarm_uav_interfaces`，旧共享 msg/srv 定义从 `tcp_to_ros` 删除；
- 新建独立 `swarm_uav_executor`，实现本机身份隔离、幂等、`MOVE_TO + HOLD`、mock 与 direct MAVROS driver；
- GCS_A 消费新接口，聚合跨机状态，补齐共享确认 deadline、状态去重、取消与 TCP 断线 HOLD 补偿；
- 旧 `tcp_to_ros/scripts/uav_mock_node.py` 已删除，integration launch 使用新 executor mock。

## 自动化验证结果

| 验证层 | 命令/范围 | 结果 |
|---|---|---|
| 构建 | `catkin build swarm_uav_interfaces swarm_uav_executor tcp_to_ros` | 4 个依赖包全部成功，0 warning/failed |
| executor | `catkin run_tests swarm_uav_executor` | 22 tests，0 error，0 failure |
| GCS_A 安全边界 | `catkin run_tests tcp_to_ros` | 7 tests，0 error，0 failure |
| ROS 单 Master | `uav_executor_rostest.test` | Service 接收与终态 Topic 通过 |

executor 测试覆盖：身份不匹配、同键幂等/冲突、忙时拒绝、HOLD 抢占、timeout、driver failure、HOLD failure、MAVROS disconnect、pose stale、setpoint/frame/yaw、连续稳定窗口、HOLD 捕获当前 pose，以及禁止自动 arm/mode。

GCS_A 安全边界测试覆盖：非参与机/错误 exec_target 状态丢弃、`status_seq` 重复与乱序丢弃、HOLD 仅发给已接受且未终止 UAV、Service 迟响应不复活、所有确认 worker 共用绝对 deadline、EmergencyHold 取消 Action 并报告未确认 UAV、TCP 断线取消 Action 并请求整组 HOLD。

## 验收结论与边界

- 可声明：GCS_A 核心链路达到机载接入基线；Phase 4 `MOVE_TO + HOLD` 首版代码、mock、direct MAVROS fake-callback 测试及单 Master ROS 闭环完成。
- 不可声明：GCS_A 全部最终验收完成，或 direct MAVROS 已通过真实飞行验证。
- PX4 SITL 三机主动飞行验证已完成，详见下一节；该结果不等价于实机上桨验收。
- 完整多 Master 弱网验收仍需归档：状态 Topic 经 bridge 的端到端聚合、网络重连/丢包、Service 代理超时与迟响应日志。
- 上桨前仍必须现场冻结 A01-A15/hostname/IP、MAVROS namespace、坐标系/yaw、位置与 yaw 阈值、PX4 failsafe、急停职责及场地许可。

## PX4 SITL 三机主动飞行验证（2026-08-08）

### 环境与安全前提

- 仿真由 `safe_valley_exp/multi_uav_sim.launch` 启动 4 架 iris，本轮仅使用 `iris_0..2`，分别绑定 `A01/UAV1`、`A02/UAV2`、`A03/UAV3`；`iris_3` 不参与。
- UAV1..3 分别使用独立 ROS Master `11312..11314`，GCS_A 使用 `11311`；三套 bridge、`swarm_uav_executor` 与 direct `MavrosPositionDriver` 均启动成功。
- 启动参数确认 `driver=mavros_position`、`auto_arm=false`、`auto_offboard=false`；三机由操作员手动解锁、起飞至约 10 m 并稳定悬停。
- 切换 OFFBOARD 前先确认三机 `/mavros/setpoint_position/local` 均持续约 30 Hz；随后经操作员授权调用 `/mavros/set_mode`。

### MOVE_TO 结果

任务 `mission_id=sitl-mavros-1786191967` 并发下发三个 30 s `MOVE_TO`：

| UAV | 目标 `(x,y,z,yaw)` | Service 确认 | OFFBOARD 前 setpoint | 终态与实测位置 |
|---|---|---|---|---|
| A01/UAV1 | `(1,0,10,0)` | `accepted=True / ACCEPTED` | 29.997 Hz | `COMPLETED`；约 `(1.02,-0.03,10.01)` |
| A02/UAV2 | `(0,1,10,0)` | `accepted=True / ACCEPTED` | 30.000 Hz | `COMPLETED`；约 `(0.03,0.98,10.03)` |
| A03/UAV3 | `(-1,0,10,0)` | `accepted=True / ACCEPTED` | 29.984 Hz | `COMPLETED`；约 `(-0.96,-0.02,10.03)` |

三机均保持 `armed=True / mode=OFFBOARD`。原始 `/uav_task_state` 记录同一任务键的 `ACCEPTED → COMPLETED`，终态重复发布符合 `StateReporter` 设计。

一次临时验收脚本误将成功终态写成 `SUCCEEDED`，且未去除 `rostopic echo` YAML 字符串的双引号，因而在动作实际完成后误报失败。项目接口、executor、GCS_A 与既有测试的权威成功终态均为 `COMPLETED`；该误报不属于产品代码故障。

### HOLD 结果

- 误报路径触发三机安全补偿 `/UAVn/uav_hold`，三次调用均返回 `accepted=True / HOLD accepted`。
- HOLD 捕获本机当前位置后继续以约 30 Hz 发布 setpoint，三机保持 armed/OFFBOARD。
- 5 秒位置位移分别为 `0.099 m`、`0.105 m`、`0.121 m`，均小于本次 SITL 验收阈值 `0.5 m`。
- executor 与 bridge 运行日志未发现 fatal、traceback、exception、driver unhealthy 或 setpoint/OFFBOARD failure。

### SITL 结论

- direct MAVROS 的三机身份隔离、非阻塞任务接收、持续 position setpoint、OFFBOARD `MOVE_TO` 到达判据、终态 Topic 和高优先级 HOLD 捕获均通过主动飞行验证。
- 本轮不覆盖自动 arm/自动切 OFFBOARD（设计上默认禁止）、真实机体坐标系与 yaw 标定、实机 PX4 failsafe、上桨安全责任、弱网/重连以及 EGO Planner/航线/编队/机间避碰。
