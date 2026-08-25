# Implementation Plan（方案 B：机载分层航点 + C++ 滚动局部规划）

[Overview]

针对 2026-08-22 有头模式 prepare 复测暴露的问题（复测数据与结构说明见 `test_records/test_record_26082300.md`）：

1. **速度超标**：轨迹峰值 10.6 m/s（远超 1.0 m/s 预期）。根因：C++ `planOnce` 一次规划完整航线（全局规划），
   稀疏关键点 + 固定 knot span ts=0.4 把整条航线压缩进少量 B-spline 控制点；`max_vel` 为软惩罚
   （`lambda_feasibility=1.0`），控制点不足时无法压住，PX4 OFFBOARD 位置环在 `MPC_XY_VEL_MAX=12` 内全速追。
2. **无分层**：MOVE_TO 未按"垂直 15m 层 → 水平 → 垂直目标 z"执行，而是直线三维飞行。
3. **避碰弱**：机间最小距离 0.93 m（<1 m 门禁）。全局规划一次后规划间隔内邻机避碰信息陈旧；
   原版 ego-planner-swarm 采用 `planning_horizon`（默认 5 m）滚动局部规划 + 实时重规划，避碰即时。

**方案 B（对齐原版 ego-planner-swarm 的 receding-horizon 思路）**：

- **上层（`swarm_uav_executor` 的 `EgoSwarmDriver`）**：按 `command` 生成含动作分层的关键点序列，
  **关键点序列纳入当前位置 start**（执行前 HOLD 态读取一次），对关键点序列按最大 0.4 m 间距均匀密化
  （对齐原版 ego-planner-swarm 控制点间距 0.4 m），作为 waypoints 发布给 C++（`onWaypoints`）。
  **不在 C++ 中做全局密化**。
- **C++（`ego_planner_driver`）**：引入 `planning_horizon`，每次规划只取当前位置前方 horizon 内的局部
  点（含 waypoints 弧长截取），轨迹执行到 horizon 末端时滚动取下一段重新规划；对齐原版动态局部规划。
- **移除** C++ `planOnce` 内的 `densifyPoints`（全局密化）与 `waypoint_max_spacing` 参数。

分层高度沿用冻结语义：`MOVE_TO` 15 m 层、`FOLLOW_ROUTE` leader 12 m 层、`FAULT_EXIT` 8 m 层。
垂直段与水平段统一参与 0.4 m 密化（start 纳入序列后垂直爬升/下降段也有足够控制点，规避 B-spline
平滑把垂直段拉斜）。

速度控制：0.4 m 密化（原版控制点间距）+ horizon 5 m → 每段 ~12 控制点 → B-spline 有足够控制点
把速度压到 `max_vel` 量级；`max_vel=1.0` 惩罚生效。1 m/s 是安全措施而非硬指标，实测达标后可在
launch 参数放宽。

## 补充修改（2026-08-23 无头复测后追加）

### 补充 1：C++ 到达最终目标后发布 COMPLETED 终态

- 根因：`checkArrival` 到达最终目标后直接 `setState(kStateHold)`，C++ **从不发布 `COMPLETED`**；
  executor `_wait_for_terminal` 只认 `COMPLETED` 为成功、`HOLD` 判 `COMMAND_HELD` 失败 →
  MOVE_TO 即使飞机到位（终点偏差 0.2-0.4m）也从未产生成功终态，prepare 恒 TIMEOUT。
- 修改：到达最终目标后**先 `setState(kStateCompleted)`** 并冻结 `steady_target_`（持续发布目标
  setpoint 悬停），不再切 HOLD；后续 HOLD/新任务由 `onHold`/`onGoal` 正常接管。
- 位置容差 `position_tolerance_m` 0.2 → **0.6 m**（接受 0.3-0.5 m PX4 稳态偏差，对齐原版 1.0 m 量级）。

### 补充 3：滚动段间 vel/acc 连续 + periodic replan（时间触发）

- 根因：滚动段每次从当前位姿 start 重新规划，B-spline 起始导数未约束（全零）→ **段间速度/加速度跳变**
  （PX4 全速追新段，vmax 9.9）；且滚动仅在"到达段末"触发，段内邻居信息不更新，动态避障陈旧。
- 修改：
  - `planOnce` 增加起始 `start_vel`/`start_acc` 参数，传入 B-spline `start_end_derivatives`
    （`[0, start_vel, start_acc, 0]`）；node 滚动时从当前轨迹**数值微分**取当前 vel/acc → 段间连续；
  - 滚动触发改为 **periodic（时间触发）**：新增 `replan_period_s`（默认 1.0，对齐原版
    `fsm/thresh_replan_time=1.0`），EXECUTING 且 `elapsed >= 周期` 且仍有 remaining waypoints →
    置 `planning_requested_`；保留到达段末触发（最终段 `COMPLETED`）。

### 补充 2：移植 swarm rebound 硬推机制（无 A\*，waypoints 替代 A\* 路径）

- 根因：移植仅有 swarm 软惩罚（`calcSwarmCost` dist_err²），实测 `lambda_swarm` 1.0→1.5 无效
  （机间距离 0.81→0.53 m，反而更近）；原版除软惩罚外还有 `check_collision_and_rebound`（A\* 路径
  求安全参考 + 把侵入控制点硬推到安全面，STOP_FOR_REBOUND 后重新优化）。
- 修改：移植 rebound 到**邻机避障**，**用密化 waypoints 替代 A\* 路径作为安全参考**：
  - 新增 `reboundFromNeighbors(neighbors, waypoints, clearance)`：对侵入邻机 clearance 的控制点
    （前 2/3，时间同步采样邻机轨迹），推开方向取"椭球归一方向去除沿 waypoints 切向分量"（即沿
    waypoints 法向避让，不脱离航线方向），把控制点硬推到 clearance 外；
  - `planOnce` 优化后接入（用推开后的控制点重新采样轨迹），保留 `checkClearance` 硬校验兜底；
  - 新参数 `swarm_rebound`（默认 true）；`lambda_swarm` 回 **1.0**（软惩罚保留为基础，rebound 为硬机制）。

[Types]

- `ego_planner_driver/include/ego_planner_driver/ego_planner_core.h`：
  - `EgoPlannerConfig` 新增 `double planning_horizon = 5.0;`（horizon 弧长，m；`<=0` 表示退回全局规划）。
  - **删除** `double waypoint_max_spacing;`（密化已移到上层）。
- `swarm_uav_executor/src/swarm_uav_executor/drivers/ego_swarm.py`：
  - 新增驱动层参数：`layer_move_to=15.0`、`layer_follow_route=12.0`、`layer_fault_exit=8.0`、
    `waypoint_densify_spacing=0.4`（关键点密化最大间距，对齐原版 control_points_distance=0.4）。
- 复用冻结契约：`MotionGoal`（含 `command/layer_z/waypoints`）、`UavTrajectoryIntent` 均不改变。

[Files]

### 修改 `swarm_uav_executor`（分层 + 密化，上层）

- `src/swarm_uav_executor/src/swarm_uav_executor/drivers/ego_swarm.py`：
  - 新增 `_build_layered_keypoints(goal, start)`：按 `goal.command` 生成分层关键点序列
    （MOVE_TO/FAULT_EXIT/FOLLOW_ROUTE leader）；**序列首个点 = 当前位置 start**（执行前 HOLD 态
    读取一次），垂直段在关键点序列中体现为 z 变化的关键点。
  - 新增 `_densify_waypoints(points, max_spacing)`：port `verification/plot_ego_bspline_waypoints.py`
    `densify_waypoints`，逐段均匀插值使相邻点距 ≤0.4 m（含垂直段）。
  - `start_move_to` / `_plan_horizontal` 改造：MOVE_TO / FAULT_EXIT / FOLLOW_ROUTE(leader) →
    分层关键点（含 start）→ 0.4 m 密化 → `_publish_waypoints`（替代 MOVE_TO 的 `_publish_goal`
    单点发布）；HOVER 保持现状（冻结位姿 setpoint）。
  - `from_ros_params` 读取新增参数。
- `src/swarm_uav_executor/config/executor_defaults.yaml`：`ego_swarm` 段新增 layer/密化参数。
- `src/swarm_uav_executor/launch/uav_executor_ego.launch`：转发新增参数。
- `test/test_ego_swarm_driver.py`：新增分层关键点生成（含 start）、0.4 m 密化间距、MOVE_TO 发布 waypoints 用例。

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

- 新增 `test_records/test_record_26082300.md`：方案 B 测试记录。
  - **复用** `test_records/test_record_26081402.md` §2.1-2.7（环境/仿真层/机载层/GCS/标定/TCP/起飞）的
    步骤、预期与结果（简写引用，不复制正文）。
  - **详写**本计划单元测试与端到端批次任务测试的步骤/预期/结果（结果在测试执行前留空）。
- `plans/README.md`：实施计划表登记本计划（待实施）。

[Functions]

- 上层（`ego_swarm.py`）：
  - `_build_layered_keypoints(goal, start) -> List[Tuple[float,float,float]]`：按命令生成
    MOVE_TO（[start]→垂直 15m→水平目标上方→垂直降 gz）、FAULT_EXIT（→8m 层）、FOLLOW_ROUTE leader
    （12m 层 route）的关键点序列；**start 为首个关键点**，垂直段保留 z 变化关键点。
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

测试记录：`test_records/test_record_26082300.md`（单元测试与端到端批次任务测试的详细步骤/预期/结果见该文档，
结果在测试执行前留空）。

### 复用步骤（预期/结果沿用 `test_records/test_record_26081402.md`）

- `test_record_26081402.md` §2.1 ROS/PX4 环境、§2.2 仿真层、§2.3 机载层、§2.4 GCS bridge、
  §2.5 backend+标定、§2.6 TCP server、§2.7 逐机 takeoff 的步骤/预期/结果直接复用，本计划不重复。

### 单元测试（详写见 test_record_26082300.md §2）

- `test_ego_swarm_driver.py`（`swarm_uav_executor`）：
  - MOVE_TO 分层关键点生成：序列含 start（垂直升 15m → 水平 → 垂直降目标 z）；
  - 0.4m 密化后相邻点距 ≤0.4m；
  - MOVE_TO 发布 `/waypoints`（PolygonStamped，首点=start）而非 `/goal`；
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

1. `ego_swarm.py`：分层关键点（含 start）+ 0.4 m 密化 + MOVE_TO 改发 waypoints；单测通过。
2. `ego_planner_core.cpp` / `driver_node.cpp`：删除全局密化，加 `planning_horizon` 截取与滚动重规划；
   更新 C++ 单测。
3. launch/config 参数对齐；`catkin build` 6 包全过；全量单测回归。
4. 有头/无头模式端到端验证 prepare（速度/分层/避碰/到达），再依次跑其余批次。
5. 结果回写 `test_records/test_record_26082300.md`，各包独立 commit（Conventional Commit），不 push。

## 补充 4：修复滚动 replan 簿记 bug（UAV 停在半途，2026-08-23 有头诊断确认）

### 现象

- 2026-08-23 有头模式 15 机 prepare：confirm 通过、任务下发给 driver、12 机从 5m 爬到
  ~6.2m 后全部停滞（EXECUTING 保持但位置不动），150s TIMEOUT。
- 单机诊断（UAV1 注入 waypoints，目标 z=9.4m）：正常爬到 ~8.3m 后停滞，setpoint 30Hz
  持续发布但轨迹终点≈当前位置；`traj_id` 每 1s 递增（periodic replan 在触发）。

### 根因（滚动簿记提前 consume）

`replanOnce` 滚动簿记逻辑：

```cpp
// 1) 截取：从 remaining_waypoints_.front() 开始累积弧长到 horizon
data.waypoints = truncateToHorizon(remaining_waypoints_, horizon, start);
data.planned_waypoint_count = data.waypoints.size();
// 2) 规划成功后：erase 本次规划覆盖的点数
remaining_waypoints_.erase(begin, begin + planned_waypoint_count);
```

问题：第一次规划时 `truncateToHorizon` 取前 N 个点（horizon 段）并 `erase`，
**但 UAV 实际只执行了其中一部分**（执行滞后于簿记，planning_horizon 截取按几何弧长、
执行按轨迹时间推进）。periodic replan（每 1s）时：

- `remaining_waypoints_` 首点已在 UAV 前方数米（前次 erase 提前消耗）；
- `truncateToHorizon` 把「首点距 start 的距离」计入 horizon 预算 → 每段只取 0-2 个点；
- 新轨迹终点≈当前位置 → UAV 原地停滞，periodic replan 反复触发。

### 修改

- `ego_planner_driver/src/ego_planner_driver_node.cpp`：
  - 移除 `remaining_waypoints_` 按 `planned_waypoint_count` erase 的簿记；
  - 改为**按 UAV 实际弧长推进**：新增 `waypoint_consumed_`（size_t 索引），每次 replan 时
    从 `waypoints_` 中取 start 前方（沿折线弧长投影）尚未消费的点，`truncateToHorizon`
    只保留前方 horizon 内点；
  - `truncateToHorizon` 的弧长累积**从 start 出发**（不再从 remaining.front()），并允许
    截取起点前移（先丢弃 start 已越过的点）；
  - `publishSetpoint` periodic 触发条件与 `checkArrival` 的「仍有后续航点」判断改用
    `waypoint_consumed_ < waypoints_.size()`；
  - `onWaypoints`/`onGoal` 重置 `waypoint_consumed_=0`。
- `include/ego_planner_driver/ego_planner_driver_node.h`：删除 `remaining_waypoints_`，
  新增 `waypoint_consumed_`。
- `test/test_trajectory.cpp` / `test/test_ego_state_machine.cpp`：新增/更新滚动簿记用例
  （连续两次 replan，第二次从实际位置取 horizon 段，轨迹终点持续推进）。

### 验证

- 单测：`catkin test ego_planner_driver` 全量 0 failure。
- 无头 prepare：UAV 持续推进（不再停滞半途），150s 内 SUCCESS 或接近目标。
