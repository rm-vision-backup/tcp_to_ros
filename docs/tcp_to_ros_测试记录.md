# tcp_to_ros 测试记录

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

## 测试方法

### 正向测试（A 组六批次）

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

### 负向测试

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

## 正向测试结果

### A 组六批次任务

| 测试项 | 预期 | 实际结果 | 是否通过 | roslaunch log |
|---|---|---|---|---|
| prepare | `SUCCESS` | `SUCCESS` | 是 | `accepted runtime connection from ('127.0.0.1', 37994)` |
| coverage-segment-1 | `SUCCESS` | `SUCCESS` | 是 | `accepted runtime connection from ('127.0.0.1', 48054)` |
| fault-exit | `SUCCESS` | `SUCCESS` | 是 | `accepted runtime connection from ('127.0.0.1', 38084)` |
| recovery | `SUCCESS` | `SUCCESS` | 是 | `accepted runtime connection from ('127.0.0.1', 38606)` |
| coverage-segment-2 | `SUCCESS` | `SUCCESS` | 是 | `accepted runtime connection from ('127.0.0.1', 54030)` |
| hold-end | `SUCCESS` | `SUCCESS` | 是 | `accepted runtime connection from ('127.0.0.1', 38308)` |

## 负向测试结果

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

## Phase 1-3 集成验证记录（本会话）

### 范围

在上一轮 mock 闭环基线之上，验证 GCS_A 整组分发后端、单机 mock 执行适配层与跨 Master Service 代理：

- `src/tcp_to_ros/scripts/gcs_a_backend_node.py`
- `src/tcp_to_ros/scripts/uav_mock_node.py`
- `src/swarm_topology_bridge/scripts/bridge_node.py`（Topic 限频修正 + Service 代理）

### 测试方法

**单 Master 语义联调**：启动 GCS_A 后端 + UAV1..3 mock：

```bash
source ~/.bashrc
source /home/ub20tg/catkin_swarm6-2/devel/setup.bash
roslaunch tcp_to_ros gcs_a_backend_test.launch
```

另开终端跑集成测试客户端：

```bash
# 正常整组 / 重复幂等（第二次仍 COMPLETED，幂等）
python3 src/tcp_to_ros/test/test_gcs_a_backend_client.py --command-id ver-ok-001 \
  --timeout 15 --expect COMPLETED
python3 src/tcp_to_ros/test/test_gcs_a_backend_client.py --command-id ver-ok-001 \
  --timeout 15 --expect COMPLETED
# 失败即停：改用注入失败的 launch（UAV2 FAILED）
roslaunch tcp_to_ros gcs_a_backend_fail.launch
python3 src/tcp_to_ros/test/test_gcs_a_backend_client.py --command-id ver-fail-001 \
  --timeout 15 --expect FAILED
```

预期：前两次 `RESULT_FINAL=COMPLETED`（completed=A01,A02,A03）；失败场景 `RESULT_FINAL=FAILED`、failed_uavs=A02、error=`MOCK_UAV2_FAIL`。

**双 Master 跨机 Service 代理**：先清理上述 launch，再按 GCS_A / UAV1 两个 Master 启动桥与 mock，最后在 GCS_A Master 上 `rosservice call /UAV1/uav_task` 验证跨 Master 转发（见下）。

### 单 Master 语义联调

环境：`roslaunch tcp_to_ros gcs_a_backend_test.launch`（GCS_A 后端 + UAV1..3 mock）。

| 场景 | 预期 | 实际结果 | 是否通过 |
|---|---|---|---|
| 正常整组 MOVE_TO | `COMPLETED`，completed=A01,A02,A03 | `RESULT_FINAL=COMPLETED`，completed_uavs=A01,A02,A03 | 是 |
| 重复 `command_id` | 返回缓存 `COMPLETED`，不重新执行 | 第二次仍 `COMPLETED`（UAV 幂等缓存 + GCS_A ACCEPTED 前终态缓冲回放）| 是 |
| 单机失败即停 | 整组 `FAILED`，failed_uavs=A02，错误码透传，活动 UAV 被 HOLD | `RESULT_FINAL=FAILED`，failed_uavs=A02，error=`MOCK_UAV2_FAIL`，A01/A03 HOLD 确认成功 | 是 |

### 双 Master 跨机 Service 代理

环境：`topology_group_a_sim.launch`，GCS_A Master=11311，UAV1 Master=11312。

| 场景 | 预期 | 实际结果 | 是否通过 |
|---|---|---|---|
| GCS_A bridge 注册 `/UAV1..3/uav_task`、`/UAV1..3/uav_hold` 代理 | 6 个 Service 代理可用 | `rosservice list` 显示全部 6 个代理 | 是 |
| GCS_A `rosservice call /UAV1/uav_task` | 转发到 UAV1 mock，返回 accepted=true | `accepted: True`，status=`ACCEPTED` | 是 |

### 过程中修复的问题

1. rospy `ServiceProxy` 无原生调用超时且不允许位置+关键字混用：改为工作线程 + Event 软超时。
2. 幂等重复请求的缓存终态可能早于 `accepted_sent`：GCS_A 缓冲 ACCEPTED 前到达的终态并在 ACCEPTED 后回放。
3. `UavHold` 响应无 `status` 字段：mock 改用 `UavHoldResponse` 构造。
4. bridge 启动崩溃：`roslib.services` 不存在，改用 `roslib.message.get_service_class`。
5. bridge Service 载荷反序列化：`genpy.Message.deserialize` 需 bytes，不能用 `BytesIO`。
6. 依赖方向修正：`swarm_topology_bridge` 为通用包，移除其对 `tcp_to_ros` 的依赖；Group A 联调示例移入 `tcp_to_ros` 包；`tcp_to_ros`/`safe_valley_exp` 依赖 `swarm_topology_bridge`。

### 待后续覆盖

- 状态 Topic 跨 Master 转发（UAV 状态经 bridge 到 GCS_A 聚合）完整闭环；
- 完整 4 秒确认边界（慢确认拒绝/重试耗尽）；
- cancel 与 TCP 断线触发 HOLD 的端到端验证；
- 弱网/重连下 bridge Service 代理的超时与迟响应。

## 关联文档

- 测试步骤：`tcp_to_ros_测试步骤.md`
- 工程过程：`tcp_to_ros_工程过程.md`
- 后续开发计划：`tcp_to_ros_后续开发计划.md`
