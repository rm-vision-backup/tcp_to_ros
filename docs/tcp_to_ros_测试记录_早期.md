# tcp_to_ros 测试记录（早期）

> **本文为最早的测试记录归档（Phase 1-5 mock / SITL 基线证据），已冻结，不再更新。**
> 后续测试证据按期归档在以下文件（按时间顺序）：
> - `docs/test_record_26081021.md`（场景 1/2/2+ 操作手册 + 历史测试结果）
> - `docs/test_record_26081115.md`（四种动作命令实现验证）
> - `docs/test_record_26081123.md`（场景 3 无头 15 机多 Master 端到端测试报告）
> 文件索引见 `plans/README.md`。


## 文档说明

本文按时间追加保存功能测试的操作步骤和结果，用于验证证据。
此后每次实验验证，都同时记录启动步骤/流程和结果（启动命令、环境变量、端口、基线确认、操作动作、结果与清理）。
开头“测试范围/当前限制/下一阶段进入条件”仅描述第一版 mock 闭环的历史基线，不代表当前项目状态；当前能力与边界以文末最新日期章节为准。

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

- 测试步骤：`test_record_26081021.md`（场景 1/2/2+ 操作手册）
- 工程过程：`tcp_to_ros_工程过程.md`
- Phase 编号/设计规约/文档索引：`plans/README.md`

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

---

# Phase 5 跨 Master 通信安全闭环（2026-08-09）

## 自动化回归

| 范围 | 结果 |
|---|---|
| `catkin build tcp_to_ros --no-deps` | 成功 |
| `catkin run_tests tcp_to_ros --no-deps` | 23 tests，0 error，0 failure |
| `catkin build swarm_uav_executor --no-deps` | 成功 |
| `catkin run_tests swarm_uav_executor --no-deps` | 36 tests，0 error，0 failure |
| launch XML、YAML、Python 语法、`git diff --check` | 全部通过 |

executor 测试由 34 项增至 36 项，新增安全 END 解锁和 END 回调失败保持锁存的回归覆盖。

## 双 Master Topic 与 Service 冒烟

- 使用独立 ROS Master 和隔离 ZMQ 端口启动 UAV1 与 GCS_A；UAV1 本地 `/mavros/state` 成功转发为 GCS_A 的 `/UAV1/mavros/state`。
- GCS_A `/group_a/uav_link_health` 报告 `A01/HEALTHY/connected=true`。
- 从 GCS_A Master 调用 `/UAV1/uav_safety_lease`，跨 bridge 的 `START → RENEW → END` 均返回 `accepted=true`，状态依次为 `ACTIVE → ACTIVE → INACTIVE`。
- 验证环境已有 11312-11314/SITL 进程占用默认 ZMQ 端口；本轮保留既有工作负载，使用临时端口副本完成隔离验证，没有修改产品默认端口。

## 三 Master 双参与机断链注入

参与机为 A01/A02，mock 动作延长至 20 秒。两机达到稳定 HEALTHY 并接受任务后，停止 A01 bridge，切断其状态 Topic 和 Service 链路：

- A01 健康状态按阈值经过 `HEALTHY → SUSPECT → LOST`；
- Action 返回 `FAILED`，`failed_uavs=A01`，`error_code=UAV_LINK_LOST`；
- A01 未收到跨机远程 HOLD，租约 TTL 到期后本地任务进入 `FAILED/LEASE_EXPIRED`；
- A02 仍可达，收到 GCS_A 远程 HOLD，任务进入 `FAILED/COMMAND_HELD`；
- 续租 Service 断开可能先于 5 秒健康 LOST 判据形成 `UAV_LINK_LOST`，最终健康 Topic 仍完整记录 SUSPECT/LOST；两条路径均保持同一安全失败语义。

## 恢复与旧任务不复活

节点级序列为 `START(epoch-old) → task c1 → TTL 过期 → RENEW(epoch-old) → END(epoch-old) → START(epoch-new) → task c2`：

- c1 过期后保持 `FAILED/LEASE_EXPIRED`；旧 epoch 的 RENEW 返回 `LEASE_EXPIRED`；
- mock driver 明确满足安全 END，END 返回 `INACTIVE`；
- 新 epoch START 后 c2 可进入 `ACCEPTED`；再次查询 c1 仍为原失败终态，未恢复旧目标；
- 故障注入由此发现并修复 executor 安全 END 后未释放本地任务锁存的问题，修复提交为 `a486a8d`。

## 提交与验收边界

- `swarm_uav_interfaces`: `a1a4b14 feat(safety): define UAV safety contracts`
- `swarm_uav_executor`: `c0ab554`, `1341d5c`, `a486a8d`
- `tcp_to_ros`: `c82e458`, `959b2dd`, `71a2dd7`
- 可声明：mock 环境下 Group A 跨 Master 健康监测、租约代理、参与机 LOST 聚合、本地/远程 HOLD 分工及安全恢复闭环通过。
- 不可声明：MAVROS/PX4 租约故障注入、Phase 6 实机无桨门禁或实机上桨许可通过。软件 HOLD 不替代 PX4 datalink/offboard/RC failsafe；现场门禁完成前保持默认不自动 arm、不自动切 OFFBOARD。

---

# 真实 MAVROS/PX4 租约断链、OFFBOARD MOVE_TO 与安全 END 门禁验证（2026-08-10）

## 环境与部署

- 仿真由用户启动在 `safe_valley_exp/multi_uav_sim.launch` 的 **ROS Master 11300**（此版本已包含 `single_vehicle_spawn_xtd.launch`，4 架 iris）；本轮仅使用 `iris_0 ↔ A01/UAV1`，其余 3 架不参与。
- UAV1 独立 ROS Master 11312 + MAVROS `udp://:24540@127.0.0.1:34580`、`tgt_system:=1`；MAVROS 自动对接新仿真的 `iris_0 (34580)`。
- 测试基础设施：UAV1 bridge（11312，`run_uav_mock:=false`）+ GCS_A bridge（11300，`run_uav_mock:=false`）+ `uav_executor_mavros.launch`（`A01/UAV1`，`interfaces_version:=sitl-lease-validation`）+ `gcs_a_backend.launch`（11300）。
- 安全基线：`driver=mavros_position`、`auto_arm=false`、`auto_offboard=false`、executor launch 强制 `safety_lease/required_for_tasks=true`；全程未自动 arm、未自动切 OFFBOARD；飞行控制由操作员手动执行。

## 启动与操作流程

本节整理本轮仿真测试的完整启动与操作步骤，作为后续同类验证的复现基准。启动顺序自上而下，每步可用则继续下一步；飞行相关步骤必须由操作员控制并逐项确认。

### 0. 前置与安全基线

- 工作空间：`/home/ub20tg/catkin_swarm6-2`；ROS Noetic；`catkin build` 后 `source devel/setup.bash`。
- 所有测试命令均需：`source /opt/ros/noetic/setup.bash && source /home/ub20tg/catkin_swarm6-2/devel/setup.bash`。
- 全程遵守：不自动 arm、不自动切 OFFBOARD；arm/takeoff 由操作员手动；测试进程不触碰用户仿真。

### 1. 仿真层（操作员手动启动，本会话主仿真端口为 11300）

```bash
roscd px4/launch   # 或直接使用 safe_valley_exp/launch/multi_uav_sim.launch
export ROS_MASTER_URI=http://localhost:11300
export ROS_HOSTNAME=localhost
export GAZEBO_MASTER_URI=http://localhost:11345
roslaunch px4 multi_uav_sim.launch
```

`multi_uav_sim.launch` 会启动 4 架 PX4 SITL（iris_0..3），`iris_0` 固定 UDP `34580`。启动后确认 4 架均 disarmed；本轮仅使用 `iris_0 ↔ A01/UAV1`。

### 2. UAV1 机载层（独立 Master 11312 + MAVROS）

先启动 UAV1 的 ROS Master：

```bash
set +x
# 终端 1：roscore
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_IP=127.0.0.1
roscore -p 11312
```

再启动 MAVROS 对接 `iris_0`（端口与 `uav_offboard_sim.launch` 的 fcu_url 公式一致）：

```bash
# 终端 2：MAVROS
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_IP=127.0.0.1
roslaunch mavros px4.launch \
  fcu_url:="udp://:24540@127.0.0.1:34580" \
  tgt_system:=1
```

基线确认：`/mavros/state` 应 `connected=True / armed=False / mode=POSCTL`。

### 3. 测试基础设施（UAV1 bridge + executor；GCS_A bridge + backend）

UAV1 侧（Master 11312；`run_uav_mock:=false` 禁用 mock）：

```bash
# 终端 3：UAV1 bridge（ZMQ topic 4300/4301，service router 14100）
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_IP=127.0.0.1
roslaunch tcp_to_ros topology_group_a_sim.launch uav_name:=UAV1 uav_id:=A01 run_uav_mock:=false

# 终端 4：UAV1 executor（不 arm、不切 OFFBOARD；租约必需）
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_IP=127.0.0.1
roslaunch swarm_uav_executor uav_executor_mavros.launch \
  uav_id:=A01 exec_target:=UAV1 state_topic:=/uav_task_state \
  interfaces_version:=sitl-lease-validation
```

GCS_A 侧（仿真层 Master 11300；GCS_A 不跑 mock）：

```bash
# 终端 5：GCS_A bridge（从 UAV1 接收 topic/转 service）
export ROS_MASTER_URI=http://127.0.0.1:11300
export ROS_IP=127.0.0.1
roslaunch tcp_to_ros topology_group_a_sim.launch uav_name:=GCS_A uav_id:=A01 run_uav_mock:=false

# 终端 6：GCS_A 分发后端（整组 Action）
export ROS_MASTER_URI=http://127.0.0.1:11300
export ROS_IP=127.0.0.1
roslaunch tcp_to_ros gcs_a_backend.launch
```

启动后只读基线：`/UAV1/mavros/state` 经桥转发、`/group_a/uav_link_health` 报告 `A01/HEALTHY connected=True age<0.5s`、`/mavros/setpoint_position/local` 约 30 Hz、GCS 侧可见 `/UAV1/uav_task|uav_hold|uav_safety_lease` 服务。

### 4. 飞行操作（操作员控制）

1. 操作员在 QGC/遥控上手动 arm + takeoff 至约 10 m（`AUTO.LOITER`）。
2. 确认 `/mavros/setpoint_position/local` 持续约 30 Hz。
3. 经操作员授权后切换 OFFBOARD：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11312
rosservice call /mavros/set_mode "{custom_mode: 'OFFBOARD'}"
```

4. 下发 `MOVE_TO`（经 GCS 整组 Action，自动 START+续租）：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11300
python3 /tmp/swarm6-2_runtime_validation/send_move_to_goal.py <mission_id> <command_id> <x> <y> <z>
```

### 5. 租约断链验证（故障注入与恢复）

```bash
# 注入：仅停止 UAV1 bridge（agent 进程，不动 MAVROS/仿真）
kill <uav1_bridge_pid>         # 本会话为 394038
# 等待约 9 s：越过 3 s SUSPECT / 5 s LOST / 5 s 租约 TTL
# 观察：/group_a/uav_link_health 出现 SUSPECT→LOST；
#       executor 日志出现 "safety lease expired and local HOLD latched"
# 恢复：重启 UAV1 bridge（同第 3 节命令），链路恢复 HEALTHY

# 锁存验证（可选，直接调 executor 租约 Service）：
# RENEW/START/END 在 EXPIRED_HOLD 下均返回 rejected（LEASE_SEQ_STALE / LEASE_EPOCH_CONFLICT）
```

### 6. 安全 END 门禁验证（地面 disarm 全套）

```bash
# 1) 操作员降落并 disarm（armed=False / AUTO.LAND）
# 2) 重启 executor 解除旧 latch（停掉第 3 节 executor 后重新拉起）
# 3) 地面执行租约 Service 序列（START → 同 epoch 拒 → END 成功 → 已 END 拒 → 新 epoch 可启）
```

### 7. 清理

仅清理测试进程：bridge（UAV1/GCS_A）、executor、backend、MAVROS、roscore 11312；保留操作员的 11300 仿真（gazebo + 4 架 px4）。用 PID 文件或精确 PID 停止，避免 `pkill -f` 匹配到自身命令行。

## OFFBOARD MOVE_TO 主链路

操作员手动 arm + takeoff 至约 10 m（`AUTO.LOITER`），确认 `/mavros/setpoint_position/local` 持续约 30 Hz 后，经操作员授权调用 `/mavros/set_mode custom_mode:=OFFBOARD`。

任务 `lease-dl-validate/move3`（`MOVE_TO` 目标 `(0,1,10,0)`，45 s）：

| 项 | 结果 |
|---|---|
| Goal | `GOAL_SENT lease-dl-validate/move3 target=(0,1,10)` |
| 终态 | `final=COMPLETED completed=[A01] failed=[]` |
| setpoint | 持续 30 Hz，目标 `(0,1,10)` |
| 实测位置 | `(0.019, 1.033, 10.011)`（符合 0.2 m/0.15 rad 容差） |
| 模式 | `armed=True / mode=OFFBOARD` |

整链路（GCS Action → GCS_A bridge → ZMQ → UAV1 bridge → executor → driver → MAVROS → PX4）在 OFFBOARD 下验证通过。

同日早前两次尝试（move1/move2）未飞的真因：当时飞机处于 `AUTO.LOITER`，PX4 不执行外部 position setpoint；且测试客户端约 40 s 自动取消触发 executor 高优先级本地 HOLD（捕获位姿持续 30 Hz）。该行为恰证明 HOLD 捕获机制真实工作，不属于产品故障。

## 租约断链验证（故障注入）

基线：`/group_a/uav_link_health` 报告 `A01/HEALTHY connected=True`。

注入：停止 UAV1 bridge（PID 394038，仅 agent 进程；MAVROS/仿真未动）。等待约 9 s 后的证据：

| 验证点 | 证据 |
|---|---|
| GCS 链路过渡 | `HEALTHY → SUSPECT（age=3.066s）→ LOST（age=5.070s）`，reason=`valid MAVROS heartbeat timed out` |
| 执行端租约 | `safety lease expired and local HOLD latched mission=lease-dl-validate epoch=89d68abe-3632-450c-b4df-2fb3e205b134`（logerr） |
| 本地安全兜底 | HOLD 捕获位姿持续约 30 Hz 发布 setpoint，`armed=True / mode=OFFBOARD` 未失控（实测悬停稳定） |

恢复：重启 UAV1 bridge 后链路恢复 `HEALTHY（age≈0.48s）`，bridge 端口 4300/4301/14100 回归。

锁存验证（直接调 executor 租约 Service）：

- 旧 epoch RENEW → `accepted=False / EXPIRED_HOLD / LEASE_SEQ_STALE（lease_seq moved backwards）`
- 新 epoch START（同 mission）→ `accepted=False / EXPIRED_HOLD / LEASE_EPOCH_CONFLICT（a lease epoch is already active）`
- END（EXPIRED_HOLD 下）→ 同样被锁存拒绝 `LEASE_SEQ_STALE`
- 结论：断链 → 租约过期 → `EXPIRED_HOLD` 后 RENEW/START/END 全部不可解除，唯一恢复路径是外部人工干预（重启 executor），旧任务零复活路径。与 mock 阶段验证一致并首次在真实 MAVROS/PX4 链路复现。

## 安全 END 门禁验证（地面 disarm 全套）

操作员降落后 `armed=False / AUTO.LAND`；重启 executor（解除旧 latch），在稳定 disarmed 下执行：

| 步骤 | 预期/实际 |
|---|---|
| `START` 新 session（epoch-A，disarm 稳定） | `accepted=True / ACTIVE` |
| 同 epoch 再 `START` | 拒绝 `LEASE_EPOCH_CONFLICT` |
| `END`（disarm 稳定 ≥3 s） | `accepted=True / INACTIVE`（记录被清除） |
| 已 END 的 epoch-A 再 `START` | 拒绝 `LEASE_EPOCH_CONFLICT`（ended epoch cannot restart） |
| 新 epoch-B `START` | `accepted=True / ACTIVE` |
| 旧 mission 任务复现（`lease-dl-validate` 新 command_id） | 拒绝 `LEASE_EXPIRED`（local safety HOLD is latched） |

验证覆盖：安全 END 清除旧 epoch/释放本地锁存、新 epoch 可启动、旧任务不复活。

## 结论与边界

- 本轮完成了 handoff 与后续开发计划中标记为"待执行"的 **MAVROS/PX4 租约断链验证**与**安全 END 门禁验证**；均为真实 MAVROS + PX4 SITL 链路，非 mock。
- 全程未触发也未验证 PX4 自身 datalink/offboard/RC loss failsafe（按本轮分工，不进行 PX4 failsafe 警告类验证）；PX4 版本确认为 `v1.13.2-dirty (xtdrone/dev)`，其 failsafe 参数（`COM_DL_LOSS_T/NAV_DLL_ACT/NAV_RCL_ACT/COM_OBL_ACT/COM_OBL_RC_ACT/CBRK_FLIGHTTERM`）已从源码核对并留存，仅供现场实机门禁参考。
- 仍不可声明：Phase 6 实机无桨门禁或实机上桨许可。软件 HOLD 不替代 PX4 datalink/offboard/RC failsafe；现场门禁完成前保持默认不自动 arm、不自动切 OFFBOARD。
- 本轮未修改任何仓库代码；仅启动/停止测试进程并在测试后全清理（仅保留用户 11300 仿真）。

---

# EGO-Swarm P1 静态复核与单 Master 冒烟（2026-08-10 18:00）

## 自动化回归

| 范围 | 结果 |
|---|---|
| `catkin build swarm_uav_interfaces ego_planner_driver swarm_uav_executor tcp_to_ros` | 成功 |
| `catkin run_tests swarm_uav_interfaces ego_planner_driver swarm_uav_executor tcp_to_ros` | 成功 |
| `catkin_test_results --all build` | **74 tests, 0 errors, 0 failures, 0 skipped**（较交接基线 72 增 2，来自 FOLLOW_ROUTE 语义测试） |

## 静态复核修复

- **FOLLOW_ROUTE 语义（P1 leader-only）**：`build_goal` 现保留 leader_id；`validate_task_request` 对 `leader_id != uav_id` 拒绝为 `NOT_IMPLEMENTED`（P2 follower 语义显式前移）；driver 改为以 `waypoints` 判空而非 `leader_id`，消除语义反向耦合。
- **HOLD fail-safe**：C++ 节点无 fresh local pose 时不再退回轨迹终点，改为 `steady_target_.reset()` 停止发布 setpoint，避免发布不安全 setpoint。
- **Plan epoch**：以优化完成时刻为执行 epoch 并平移轨迹 t/duration，补偿优化耗时避免侵占执行时间轴。

## 单 Master EGO 运行时冒烟（PASS）

- 环境：`roscore(11311)` + `roslaunch ego_planner_driver ego_planner_driver.launch uav_id:=A01 exec_target:=UAV1`；冒烟脚本 `/tmp/ego_smoke2.py`（订阅 setpoint 回灌为 local_pose 模拟理想位控）。
- **发现并修复命名空间 bug**：C++ 节点接口 topic 与身份参数原建在 private handle，落在 `/UAV1/ego_planner_driver/*`，与 Python 驱动期望的 `/UAV1/*` 不一致；修复为公开 handle + launch `<group ns>` 放身份参数（提交 `ec9ac63`）。
- 结果：`states: [IDLE, EXECUTING, COMPLETED, HOLD]`，setpoint 79 帧，`SMOKE PASS`。

## 边界

- 双/三 Master reciprocal 与 PX4 SITL/MAVROS 回归因当前环境缺 PX4-Autopilot/px4 可执行文件无法执行，**P1 仍不可声明完成**。
- 无残留 ROS 进程；测试后已全清理。

# EGO-Swarm P1 多 Master/SITL 组合验证（2026-08-10 20:05）

## 验证结论

- 单 Master 双 ego reciprocal、双 Master reciprocal、三 Master reciprocal、PX4 SITL/MAVROS + SafetyLeaseWatchdog + ego driver 组合全部 PASS。
- P1 未完成项中：PX4 SITL/MAVROS 回归、双/三 Master reciprocal、SafetyLeaseWatchdog 与 ego driver 组合回归、END gate MAVROS 条件、MotionGoal.waypoints 收窄、FAULT_EXIT/HOVER layer height 约束均已完成。
- 修复真实 bug 2 个：`uav_executor_ego.launch` 的 `supported_commands` 用 `<param>` 传字符串（应为 `<rosparam>` 列表）导致 UNSUPPORTED_COMMAND；该 launch 缺公开命名空间身份参数（应 `/UAV1/uav_id`）导致 DRIVER_NOT_READY。
- 本次未修改规划/交接文档新增完成声明；P1 是否可声明完成按用户确认的下一步（README/CONTEXT/测试记录收口）评估。

## 验证步骤与证据

### 1. 单 Master 双 ego reciprocal（11311，脚本 verification/ego_smoke/smoke_reciprocal.py）

- 平行同飞（5,0,15）/（5,5,15）：两机互收 neighbor_intent（UAV1←A2、UAV2←A1），均 EXECUTING/HOLD。
- 错位 3s：两机 EXECUTING→COMPLETED。
- `RECIPROCAL SMOKE PASS`。

### 2. 双 Master reciprocal（UAV1:11311 + UAV2:11312，经 swarm_topology_bridge 专用拓扑 topology_multimaster.yaml）

- 两次运行分别验证 UAV1←A2×28 与 UAV2←A1（双向跨 Master intent 互达）。
- relay：本机 /UAVx/trajectory_intent → /trajectory_intent（bridge 采集），邻居 /<nb>/trajectory_intent → /UAVx/neighbor_intent（C++ onNeighborIntent 输入）。

### 3. 三 Master reciprocal（UAV1/2/3:11311/11312/11313 全连通）

- UAV1←A3、UAV2←{A1,A3}、UAV3←A1，全部 EXECUTING，`CROSS-MASTER PASS`×3。
- 验证了交接要求的三机错位/换位、全连通与绝对 stamp 转本机相对时轴（replan 成功）。

### 4. PX4 SITL/MAVROS + SafetyLeaseWatchdog + ego driver 组合（单机 UAV1）

- 仿真层：`multi_uav_sim_headless.launch`（新增，gui:=false）启动 4 架 PX4 SITL（iris_0..3，Onboard 端口 34580-34583）+ Gazebo 无头；MAVROS 150 baud UDP 对接 iris_0。
- 机载层：`uav_offboard_ego.launch`（新增）MAVROS + `uav_executor_ego.launch`。
- `combined_sitl_check.py`（verification/ego_smoke/）：
  - LEASE START（ACTIVE）→ task MOVE_TO（ACCEPTED）→ ego IDLE→EXECUTING→COMPLETED→HOLD（setpoint 105 帧）→ RENEW → END（INACTIVE）。
  - `COMBINED PASS`。
- 第一次组合运行还复现了 watchdog 锁存：TTL 过期进入 EXPIRED_HOLD 后 RENEW/START/END 均被拒（LEASE_EPOCH_CONFLICT），唯一恢复路径是重启 executor——与 2026-08-10 mavros_position 路径结论一致，现于 ego driver 路径复现。

### 5. 静态收口项

- END gate：`EgoSwarmDriver` 新增 `mavros_state_topic` 可选配置；配置后 `can_end_safety_lease` 强制 MAVROS state 新鲜、connected、stable disarmed（镜像 MavrosPositionDriver）。默认配置下保持纯 ego 基线（fresh pose + 非 EXECUTING）。`executor_defaults.yaml` 与 `uav_executor_ego.launch` 暴露参数。新增 4 项单元测试（disconnected/armed/not-stable/stable），`test_ego_swarm_driver.py` 11/11 通过。
- `MotionGoal.waypoints` 收窄为 `Tuple[Tuple[float,float,float,float], ...]`。
- FAULT_EXIT/HOVER/FOLLOW_ROUTE layer height 约束：FAULT_EXIT z≈8m、HOVER 1..16m、FOLLOW_ROUTE waypoints z≈12m；新增 4 项 validation 测试。
- 回归：`pytest src/swarm_uav_executor/test/` = 54 passed（1 项 rostest 需单独跑）；`rostest swarm_uav_executor uav_executor_rostest.test` = SUCCESS。

### 6. 环境清理

- 已清理 roscore/gzserver/px4/MAVROS/executor/ego/relay/bridge 全部进程；无残留。
- 本阶段在 `verification/ego_smoke/`（工作空间顶层，不属任何 git 仓库）新增验证脚本：smoke_single_master.py、smoke_reciprocal.py、multimaster_relay.py、mm_check.py、combined_sitl_check.py、topology_multimaster.yaml、multimaster_bridge.launch、run_multimaster.sh、README.md。

# EGO 无前缀机载形态改造验证（2026-08-10 20:40）

## 改造背景与目标

用户要求：机载端 ego_planner_driver 本机**无 namespace 前缀**（与 MAVROS `/mavros/*` 一致），机间区分交给 swarm_topology_bridge 的源前缀（`/<邻居>/trajectory_intent`）。仿真与实机程序层面统一，仅切换 launch 即可。

## 改动清单

- `ego_planner_driver.launch`：删除节点 `ns`（全局运行）、`<group ns>` 身份参数改全局 `/uav_id`/`/exec_target`。
- `uav_executor_ego.launch`：删除 `ego_namespace` 与节点 `ns`、删除 `trajectory_intent` remap、身份参数全局化、`ego_swarm/namespace=""`。
- `uav_offboard_ego.launch`：移除 `ego_namespace` 传参。
- `executor_defaults.yaml`：`ego_swarm.namespace=""`。
- `EgoSwarmDriver`：默认 `namespace=""`；空 namespace 直接发布 `/setpoint`、`/exec_state` 等；非空保留兼容（旧单 Master 多 UAV）。`_node_ready` 在空 namespace 时检查全局 `/uav_id`。
- 验证脚本：`smoke_single_master.py`/`combined_sitl_check.py` NS=""；`multimaster_relay.py` 简化为仅桥接邻居→本机 `/neighbor_intent`（本机 `/trajectory_intent` 由 bridge 直接采集，不再搬运）；`mm_check.py` 无前缀；`run_multimaster.sh` relay 参数更新。

## 验证结果（全部 PASS）

1. 单元测试：`test_ego_swarm_driver.py` + `test_validation.py` = 23 passed。
2. 单 Master 无前缀冒烟：话题确认 `/exec_state`、`/setpoint`、`/trajectory_intent`、`/neighbor_intent` 等无前缀；`/uav_id=A01` 全局参数可发现；IDLE→EXECUTING→COMPLETED→HOLD，setpoint 81 帧，SMOKE PASS。
3. 双 Master 无前缀 reciprocal（11311/11312 + bridge）：双向互达（UAV1←A2、UAV2←A1），CROSS-MASTER PASS×2。
4. PX4 SITL 无前缀组合（MAVROS connected=True/armed=False + ego + lease）：LEASE START(RENEW/END 全 ACTIVE/INACTIVE)，task MOVE_TO ACCEPTED，ego IDLE→EXECUTING→COMPLETED→HOLD，setpoint 106，COMBINED PASS。
   - END gate 现在启用 `mavros_state_topic=/mavros/state`（MAVROS 新鲜/connected/stable disarmed 全部满足），lease END 通过。

## 环境清理

- 已清理 roscore/gzserver/px4/MAVROS/executor/ego/relay/bridge 全部进程；无残留。
