# tcp_to_ros 四种完整动作命令实现验证记录（2026-08-11 15:00）

> 依据 `plans/implementation_plan_26081023.md` 的 [Implementation Order] 实施
> 四种完整动作命令（MOVE_TO / FOLLOW_ROUTE / FAULT_EXIT / HOVER），并在
> `swarm_uav_executor` + `ego_planner_driver` 机载执行层补齐。
>
> 本记录对应的基线：`docs/test_record_26081021.md` 场景 2+ 表格中的 FAILURE 项
> （FOLLOW_ROUTE follower、FAULT_EXIT、HOVER）已全部转为 PASS。

## 1. 单元测试 / 语法校验

执行 `catkin build` + `catkin run_tests` 后 `catkin_test_results --all build`：

| 项目 | 结果 |
|---|---|
| 语法校验（models/validation/ego_swarm py_compile） | PASS |
| YAML（executor_defaults.yaml）与 launch XML（uav_executor_ego.launch）校验 | PASS |
| `test_validation.py`（follower/层高/layer_z/FAULT_EXIT waypoints/HOVER 占位） | 18 passed |
| `test_ego_swarm_driver.py`（HOVER 冻结、follower PI/超时、垂直先行、FAULT_EXIT 两阶段） | 19 passed |
| `catkin_test_results --all build` 全量 | 103 tests, 0 errors, 0 failures |

## 2. 单机（无 MAVROS）冒烟

`verification/ego_smoke/smoke_commands.py`（隔离驱动层，仅 roscore）：

| 验证项 | 结果 |
|---|---|
| HOVER 冻结当前位置并持续发布（不跟随新自身位姿） | PASS |
| FAULT_EXIT 纯垂直 8m → 水平航线两阶段 | PASS |
| FOLLOW_ROUTE follower：leader odom + 编队偏置 + PI 限幅 setpoint | PASS |
| MOVE_TO 纯垂直 15m 层 → 水平目标两阶段 | PASS |

## 3. 场景 2+ 端到端验收（15 mock executor + TCP Server）

验收环境：`verification/acceptance/gcs_a_backend_15exec.launch`（15 个 mock
executor A01..A15 + 15 个 mock MAVROS state）+ `tcp_server_node.py`（39001），
单 Master 11311。通过 `references/runtime/task_test.py` 发送 6 个批次：

| 批次 | 命令 | 参与机 | 结果（本次） | 基线 test_record_26081021 结果 |
|---|---|---|---|---|
| prepare | MOVE_TO | A01..A12 | `SUCCESS` failed=[] | 是 |
| coverage-segment-1 | FOLLOW_ROUTE | A01..A12 | `SUCCESS` failed=[] | 部分（follower NOT_IMPLEMENTED）→ PASS |
| fault-exit | FAULT_EXIT | A07..A12 | `SUCCESS` failed=[] | 否（mock 未实现落地语义）→ PASS |
| recovery | MOVE_TO | A01..A06 + A13..A15 | `SUCCESS` failed=[] | 是 |
| coverage-segment-2 | FOLLOW_ROUTE | A01..A06 + A13..A15 | `SUCCESS` failed=[] | 部分（follower NOT_IMPLEMENTED）→ PASS |
| hold-end | HOVER | A01..A06 + A13..A15 | `SUCCESS` failed=[] | 否（mock 未实现 HOVER 语义）→ PASS |

## 4. 本轮代码改动摘要

- `swarm_uav_executor/models.py`：`MotionGoal` 增加 `layer_z`、`formation_offset`
  （均带默认值，不破坏既有构造点）；`ExecutorConfig` 增加 follower PI 增益/限幅字段。
- `swarm_uav_executor/validation.py`：
  - 移除 FOLLOW_ROUTE follower 的 NOT_IMPLEMENTED；`leader_id != uav_id` 或
    `formation_follow=True` 走 follower 校验（follower 不要求 waypoints）。
  - FAULT_EXIT 支持单点 target_pose 与撤离航线 waypoints 两种形态（8m 层校验）。
  - HOVER 接受 runtime 的占位 assignment（仅 uav_id，高度由驱动按当前位姿冻结）。
  - `build_goal` 填充 `layer_z`：MOVE_TO@15 / FOLLOW_ROUTE@12 / FAULT_EXIT@8 / HOVER 冻结值。
- `swarm_uav_executor/drivers/ego_swarm.py`：
  - 订阅 leader odom（`/<leader>/mavros/local_position/odom`，自动 /UAVn 映射）。
  - `start_move_to` 按 command 分派：HOVER → `_hover_loop`（30Hz 冻结发布）、
    follower → `_follower_loop`（10Hz PI + 30Hz setpoint 发布、LEADER_LOST 超时）、
    MOVE_TO/FAULT_EXIT/FOLLOW_ROUTE leader → `_plan_vertical_transition` 纯垂直先行
    + `_plan_horizontal`。均复用 C++ exec_state 终态或 `_emit_completed` 回写。
- `executor_defaults.yaml` + `uav_executor_ego.launch`：新增 PI/层高参数与
  `formation_offsets`、`leader_odom_topic_prefix` 转发。
- `tcp_to_ros/config/topology_group_a_sim.yaml`：新增 `mavros/local_position/odom`
  topic 桥接（nav_msgs/Odometry）。
- `swarm_uav_executor` package.xml / CMakeLists.txt：显式声明 `nav_msgs` 依赖；
  `test_ego_swarm_driver.py` 纳入 catkin 测试清单。
- `tcp_to_ros/CONTEXT.md`：补充「误差积分」术语定义。
- `verification/ego_smoke/smoke_commands.py`：新增四种动作命令单机冒烟脚本。

## 5. 备注

- 多 Master（GCS_A + UAV1..UAV3）真实 MAVROS + PX4 SITL 端到端（场景 3）未在本轮执行，
  需在带图形界面/无头 SITL 环境按 `test_record_26081021.md` 场景 3 另行验证。
- mock driver 对四命令返回确定性 COMPLETED；本机 ego 层的 HOVER/follower/分层语义
  已在第 2 节无 MAVROS 冒烟中验证。