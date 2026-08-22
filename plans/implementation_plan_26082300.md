# Implementation Plan（方案 B：机载分层航点 + C++ 滚动局部规划）

[Overview]

针对 2026-08-22 有头模式 prepare 复测暴露的问题（复测数据与结构说明见 `docs/test_record_26082300.md`）：

1. **速度超标**：轨迹峰值 10.6 m/s（远超 1.0 m/s 预期）。根因：C++ `planOnce` 一次规划完整航线（全局规划），
   稀疏关键点 + 固定 knot span ts=0.4 把整条航线压缩进少量 B-spline 控制点；`max_vel` 为软惩罚
   （`lambda_feasibility=1.0`），控制点不足时无法压住，PX4 OFFBOARD 位置环在 `MPC_XY_VEL_MAX=12` 内全速追。
2. **无分层**：MOVE_TO 未按"垂直 15m 层 → 水平 → 垂直目标 z"执行，而是直线三维飞行。
3. **避碰弱**：机间最小距离 0.93 m（<1 m 门禁）。全局规划一次后规划间隔内邻机避碰信息陈旧；
   原版 ego-planner-swarm 采用 `planning_horizon`（默认 5 m）滚动局部规划 + 实时重规划，避碰即时。

**方案 B（对齐原版 ego-planner-swarm 的 receding-horizon 思路）**：

- **上层（`swarm_uav_executor` 的 `EgoSwarmDriver`）**：按 `command` 生成含动作分层的关键点序列
  （垂直上升点、水平航点、垂直下降点），对关键点序列按最大 2 m 间距均匀密化，作为 waypoints 发布给
  C++（`onWaypoints`）。**不在 C++ 中做全局密化**。
- **C++（`ego_planner_driver`）**：引入 `planning_horizon`，每次规划只取当前位置前方 horizon 内的局部
  点（含 waypoints 弧长截取），轨迹执行到 horizon 末端时滚动取下一段重新规划；对齐原版动态局部规划。
- **移除** C++ `planOnce` 内的 `densifyPoints`（全局密化）与 `waypoint_max_spacing` 参数。

分层高度沿用冻结语义：`MOVE_TO` 15 m 层、`FOLLOW_ROUTE` leader 12 m 层、`FAULT_EXIT` 8 m 层。
垂直段与水平段统一参与 2 m 密化（规避 B-spline 平滑切角）。

速度控制：2 m 密化 + horizon 内控制点少 → 优化快；`max_vel=1.0` 惩罚在控制点充足时可将轨迹速度
压到 1 m/s 量级。1 m/s 是安全措施而非硬指标，实测达标后可在 launch 参数放宽。

[Types]

- `ego_planner_driver/include/ego_planner_driver/ego_planner_core.h`：
  - `EgoPlannerConfig` 新增 `double planning_horizon = 5.0;`（horizon 弧长，m；`<=0` 表示退回全局规划）。
  - **删除** `double waypoint_max_spacing;`（密化已移到上层）。
- `swarm_uav_executor/src/swarm_uav_executor/drivers/ego_swarm.py`：
  - 新增驱动层参数：`layer_move_to=15.0`、`layer_follow_route=12.0`、`layer_fault_exit=8.0`、
    `waypoint_densify_spacing=2.0`（关键点密化最大间距）。
- 复用冻结契约：`MotionGoal`（含 `command/layer_z/waypoints`）、`UavTrajectoryIntent` 均不改变。

[Files]

### 修改 `swarm_uav_executor`（分层 + 密化，上层）

- `src/swarm_uav_executor/src/swarm_uav_executor/drivers/ego_swarm.py`：
  - 新增 `_build_layered_keypoints(goal, start)`：按 `goal.command` 生成分层关键点序列
    （MOVE_TO/FAULT_EXIT/FOLLOW_ROUTE leader）；垂直段在关键点序列中体现为 z 变化的关键点。
  - 新增 `_densify_waypoints(points, max_spacing)`：port `verification/plot_ego_bspline_waypoints.py`
    `densify_waypoints`，逐段均匀插值使相邻点距 ≤2 m。
  - `start_move_to` / `_plan_horizontal` 改造：MOVE_TO / FAULT_EXIT / FOLLOW_ROUTE(leader) →
    分层关键点 → 2 m 密化 → `_publish_waypoints`（替代 MOVE_TO 的 `_publish_goal` 单点发布）；
    HOVER 保持现状（冻结位姿 setpoint）。
  - `from_ros_params` 读取新增参数。
- `src/swarm_uav_executor/config/executor_defaults.yaml`：`ego_swarm` 段新增 layer/密化参数。
- `src/swarm_uav_executor/launch/uav_executor_ego.launch`：转发新增参数。
- `test/test_ego_swarm_driver.py`：新增分层关键点生成、2 m 密化间距、MOVE_TO 发布 waypoints 用例。

### 修改 `ego_planner_driver`（horizon 局部规划，C++）

- `src/ego_planner_driver/src/ego_planner_core.cpp`：
  - **删除** `densifyPoints` 与 `planOnce` 内的密化调用（`waypoint_max_spacing`）。
  - `planOnce` 增加 horizon 截取：从 `start` 沿 `sparse_waypoints` 按弧长累计，仅保留前方
    `planning_horizon` 内的点（不足则到终点）；`local_target_pt_` 取截取段末点。
- `src/ego_planner_driver/src/ego_planner_driver_node.cpp`：
  - 构造函数读取 `planning_horizon`；`replanLoop` 滚动：执行到 horizon 末端（`elapsed` 接近
    当前轨迹 `duration`）且仍有后续 waypoints 时，重新取下一段规划（`planning_requested_=true`）。
  - 保留 `onNeighborIntent` 触发重规划（邻机轨迹变化时基于最新 waypoints 重新局部规划）。
- `src/ego_planner_driver/launch/ego_planner_driver.launch`：删 `waypoint_max_spacing_m`，加 `planning_horizon`。
- `test/test_trajectory.cpp`：更新密化相关用例为 horizon 截取/滚动用例。

### 文档

- 新增 `docs/test_record_26082300.md`：方案 B 测试记录。
  - **复用** `docs/test_record_26081402.md` §2.1-2.7（环境/仿真层/机载层/GCS/标定/TCP/起飞）的
    步骤、预期与结果（简写引用，不复制正文）。
  - **详写**本计划单元测试与端到端批次任务测试的步骤/预期/结果（结果在测试执行前留空）。
- `plans/README.md`：实施计划表登记本计划（待实施）。

[Functions]

- 上层（`ego_swarm.py`）：
  - `_build_layered_keypoints(goal, start) -> List[Tuple[float,float,float]]`：按命令生成
    MOVE_TO（start→(sx,sy,15)→(gx,gy,15)→(gx,gy,gz)）、FAULT_EXIT（→8m 层）、FOLLOW_ROUTE leader
    （12m 层 route）的关键点序列；垂直段保留 z 变化关键点。
  - `_densify_waypoints(points, max_spacing) -> List[...]`：逐对点 `segments=max(1,ceil(dist/spacing))` 均匀插值。
  - `_plan_horizontal(goal, cancel, deadline)`：按命令走 `_publish_waypoints`（密化后）。
- C++：
  - `std::vector<WayPoint> truncateToHorizon(start, waypoints, horizon)`：按弧长截取 horizon 内航点。
  - `replanLoop`：滚动取段（horizon 末端触发下一段规划）。

[Classes]

- 修改 `EgoSwarmDriver`（`ego_swarm.py`）：新增分层关键点与密化能力；`MotionDriver` 抽象不变。
- 修改 `EgoPlannerCore`（C++）：新增 horizon 截取；删除全局密化。
- 修改 `EgoPlannerDriverNode`：滚动重规划逻辑。
- 不新增/删除其他类。

[Dependencies]

- 无新增第三方依赖。
- `swarm_uav_executor` 继续依赖 `geometry_msgs`（waypoints PolygonStamped 已有）。
- `ego_planner_driver` 依赖不变（Eigen/roscpp/swarm_uav_interfaces）。

[Testing]

测试记录：`docs/test_record_26082300.md`（单元测试与端到端批次任务测试的详细步骤/预期/结果见该文档，
结果在测试执行前留空）。

### 复用步骤（预期/结果沿用 `docs/test_record_26081402.md`）

- `test_record_26081402.md` §2.1 ROS/PX4 环境、§2.2 仿真层、§2.3 机载层、§2.4 GCS bridge、
  §2.5 backend+标定、§2.6 TCP server、§2.7 逐机 takeoff 的步骤/预期/结果直接复用，本计划不重复。

### 单元测试（详写见 test_record_26082300.md §2）

- `test_ego_swarm_driver.py`（`swarm_uav_executor`）：
  - MOVE_TO 分层关键点生成：垂直升 15m → 水平 → 垂直降目标 z；
  - 2m 密化后相邻点距 ≤2.0m；
  - MOVE_TO 发布 `/waypoints`（PolygonStamped）而非 `/goal`；
  - HOVER 保持冻结 setpoint；
  - FAULT_EXIT 先垂直 8m 层再水平。
- `test_trajectory.cpp`（`ego_planner_driver`）：
  - horizon 截取：waypoints 超出 horizon 时只取前方部分，`local_target_pt_` 为截取段末点；
  - 滚动：到达 horizon 末端后再次规划下一段；
  - 全局退化：`planning_horizon<=0` 时一次规划完整航线（回归兼容）。
- `catkin_test_results --all build` 0 failure。

### 端到端批次任务测试（详写见 test_record_26082300.md §3）

- prepare（A01-A12 MOVE_TO）：12 机编队到达且 `status=SUCCESS`（终点偏差 ≤0.2m 容差）、
  速度 ≤1.5m/s、出现 15m 层、机间最小距离 ≥1m。
- 依次 coverage-segment-1 / fault-exit / recovery / coverage-segment-2 / hold-end。

[Implementation Order]

1. `ego_swarm.py`：分层关键点 + 2 m 密化 + MOVE_TO 改发 waypoints；单测通过。
2. `ego_planner_core.cpp` / `driver_node.cpp`：删除全局密化，加 `planning_horizon` 截取与滚动重规划；
   更新 C++ 单测。
3. launch/config 参数对齐；`catkin build` 6 包全过；全量单测回归。
4. 有头/无头模式端到端验证 prepare（速度/分层/避碰/到达），再依次跑其余批次。
5. 结果回写 `docs/test_record_26082300.md`，各包独立 commit（Conventional Commit），不 push。
