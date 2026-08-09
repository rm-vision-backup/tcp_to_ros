# Implementation Plan

[Overview]
以“原版 ego-planner-swarm 内核按不改动方式移植为机载轨迹后端，驱动 Group A 的 MOVE_TO / FAULT_EXIT / 单机 FOLLOW_ROUTE / HOVER”为本轮（P1）核心；编队跟随（FOLLOW_ROUTE 整组）留待 P2，其 follower 只跟随、不规划、不参与 swarm 轨迹/位姿交换，从而与 ego-swarm 原版互不干扰。本计划承接 2026-08-09 handoff 与已完成的 MAVROS 租约断链/END 验证，在既有 `swarm_uav_executor + MavrosPositionDriver + SafetyLeaseWatchdog + swarm_topology_bridge` 之上增加“带时平滑轨迹与机间避碰”执行能力。

已冻结决策（前序调研与拷问结论，写入本计划并在实施时回填 CONTEXT.md 领域词汇）：
- A 组固定 15 架 A01–A15（TCP 需求以 `references/docs` 为准）；A13–A15 为恢复/接替集；RETURN 是 B 组命令，A 组不实现。
- 坐标系：ROS ENU；向南 `dy=-5m`、向东 `dx=+5m`。
- 分层高度：MOVE_TO 上爬集结 15 m；FOLLOW_ROUTE 巡航 12 m；FAULT_EXIT 下降至 8 m。
- 编队矩阵（P2 生效，GCS_A 静态配置）：coverage-1 = 3 行×4 列（A01,A04,A07,A10 / A02,A05,A08,A11 / A03,A06,A09,A12）；coverage-2 = 3 行×3 列（A01,A04,A13 / A02,A05,A14 / A03,A06,A15）。
- 机间链路：`swarm_topology_bridge` services 分支已支持 UAV↔UAV 多对多（`topology_sim_swarm.yaml` 已示范），复用其 Topic 直连；参考坐标系为全局 ENU。
- ego-swarm 语义核实（依据 `references/ego-planner-swarm/docs/3.Zhou-等-2021-EGO-Swarm...md` 与源码）：priority 仅用于启动 chain 顺序（§V-B-2）；运行时避碰为 reciprocal 对等惩罚（式 5）；planning range 外轨迹被忽略（§V-B-1，即“阈值外不避让”）；A01 属项目场景而非论文概念。
- 本计划 P1 按原版移植，不做任何避碰语义改造；单向优先级（A01 最高、低位单向让行）不在 P1；除非出现明确安全需求，否则默认不进入 P2（届时单独立 ADR）。

P1（本轮）范围：
1. 抽取 `references/ego-planner-swarm` 的 bspline 优化 + 轨迹生成内核，封装为新增 catkin 包 `ego_planner_driver`，编译进本工作空间；**不移植**其感知外挂（drone_detect、VIO 漂移补偿、深度图像处理）——本场景无环境障碍，仅保留机间轨迹/位姿交换与 reciprocal 避碰。
2. 以 `MotionDriver` 兼容实现 `EgoSwarmDriver`：输入稀疏关键点（runtime/任务层）→ 内部生成带时平滑轨迹 → 周期发布 setpoint（兼容 PX4 OFFBOARD，默认不 arm、不切 OFFBOARD）；同时把本机轨迹广播给邻机，并把邻机轨迹作为动态障碍参与优化（原版 reciprocal）。
3. 消息/总线适配：机间直连新增轨迹广播 topic 与邻机位姿 topic；Group A 配置对需要互相避让的 UAV 增加 UAV↔UAV topology 条目；GCS_A 不进入控制环。
4. `UavTaskExecutor` 接线：MOVE_TO、FAULT_EXIT、HOVER 直接路由到 `EgoSwarmDriver`；FOLLOW_ROUTE 在 P1 退化为“仅 leader(A01) 单机执行 route”，follower 参与时返回明确的 `NOT_IMPLEMENTED`（P2 补齐整组编队）。
5. 维持既有安全语义：SafetyLeaseWatchdog（断链/过期→本机 HOLD）、leader 参考超时 1 s → HOLD/LEADER_LOST、END 门禁、SUSPECT/LOST/RECOVERED 全链路不回退。
6. 测试与验证：自动化单测（轨迹数值/时基/取消/重启/邻居缺失）+ mock 故障注入；SITL 冒烟按“单机→双机交叉→三机错位/换位”逐级，全部机载分布式运行；每次实验按测试记录规范同时记录启动流程与结果。

P2（紧随的独立阶段，不在本轮实现）：
- FOLLOW_ROUTE 整组编队：GCS_A 静态模板实例化 + leader 带时轨迹；follower 仅订阅 leader 位姿 + 固定偏移（≤1 s 缓冲），**不发布自身轨迹、不消费其他 UAV 位姿**，从而对 ego-swarm 网络不可见，与原版算法兼容。
- 编队跟随判据 a+b：实时偏移跟踪（1 s 内相位滞后）+ 可配位置闭环；编队误差全程容差 + 终点稳定作为 follower COMPLETED。
- Phase 6 实机不上桨门禁（namespace/坐标/优先级/急停/场地签字）。

[Types]

### `swarm_uav_interfaces` 新增（机间轨迹共享，P1 落地）

1. 新增 `UavTrajectoryIntent.msg`
   - `string protocol_version`：固定 `"1.0"`。
   - `string uav_id`：任务层身份（如 `A01`）。
   - `string exec_target`：执行身份（如 `UAV1`）。
   - `int32 traj_id`：本机轨迹单调递增标识，供乱序去重。
   - `float64 stamp`：发布时刻 ROS time 秒值。
   - `float64[] t`：轨迹采样时间序列（相对发布时刻，单位 s）。
   - `geometry_msgs/Pose[] sampled_traj`：与 `t` 等长的全局 ENU 位姿采样。
   - `float64 clearance`：本机所需机间安全间距（≤0 表示不参与避碰计算）。
   - 若实施时发现原版 `quadrotor_msgs/SwarmTraj` 更契合内核，可在桥接层把 `UavTrajectoryIntent` 映射到该类型，两者取固定其一并与 driver 约定一致。

### `ego_planner_driver` 配置与内部结构

- `EgoSwarmDriverConfig`：关键点时间分配、最大速度/加速度、规划时域 `planning_horizon`、忽略范围 `planning_range`、重规划周期 `replan_period`、机间间距 `clearance`、分层高度 `layer_heights{move:15, follow:12, fault:8}`、setpoint 发布频率（≥30 Hz）。
- 机内结构：`TrajectoryIntentBuffer`（按 `uav_id + traj_id` 乱序去重）、`NeighborIntentSet`（planning_range 过滤后的邻机轨迹集）、`EgoOptimizerHandle`（包装抽取的 bspline 优化内核）。
- 错误码新增：`LEADER_LOST`、`NOT_IMPLEMENTED`、`EGO_PLAN_FAILED`、`EGO_EXEC_TIMEOUT`。

### 复用冻结契约（不破坏）

- `UavTask.srv`（command 仍为 `MOVE_TO/FOLLOW_ROUTE/FAULT_EXIT/HOVER`；`HOVER` 名与 TCP 接口文档一致）、`UavTaskState.msg`、`UavHold.srv`、`UavSafetyLease.srv`、`UavLinkHealth.msg`、`TaskAssignment.msg` 均不改变字段与语义。

[Files]

### 新增文件

- `src/ego_planner_driver/`（独立 catkin 包，C++）
  - `CMakeLists.txt`、`package.xml`：编译核心抽取代码；依赖 `roscpp`、`Eigen`、`quadrotor_msgs`（或 `swarm_uav_interfaces` 意图消息）、原版 bspline/优化源码（以源码拷贝或 submodule 引用方式，实施时按许可证/仓库边界固定）。
  - `include/ego_planner_driver/`：把 ego-planner-swarm 的 bspline 优化 + 轨迹生成内核按原样抽取为头文件/实现（仅裁剪感知/建图/检测相关文件）。
  - `src/ego_planner_driver_node.cpp`：作为可执行节点，提供 topic 订阅（目标、邻机轨迹/位姿）与发布（setpoint、本机意图）。
  - `launch/ego_planner_driver.launch`：参数化 `uav_id/exec_target/namespace`、分层高度、clearance、planning_range。
  - `test/test_trajectory.cpp`：轨迹数值/时基/可行性/邻居缺失单测。
- `src/swarm_uav_interfaces/msg/UavTrajectoryIntent.msg`：按 `[Types]` 定义。
- 文档：`src/swarm_uav_executor/README.md`、`src/tcp_to_ros/README.md`、`docs/tcp_to_ros_测试记录.md`、`docs/tcp_to_ros_后续开发计划.md`、`docs/tcp_to_ros_工程过程.md`、`src/tcp_to_ros/CONTEXT.md`（术语回填）、`docs/tcp_to_ros_提问记录.md`（已确认决策历史）。

### 修改文件

- `src/swarm_uav_executor/`
  - `drivers/ego_swarm.py`（Python 包装节点与 `MotionDriver` 适配，调用 `ego_planner_driver_node` 或直连其 topic/service）。
  - `drivers/base.py`：确认 `MotionDriver` 抽象足够（start_move_to/hold/health/shutdown）；必要时新增 `trajectory_intent` topic 发布能力。
  - `executor.py`：命令路由扩展（MOVE_TO/FAULT_EXIT/HOVER → ego；FOLLOW_ROUTE 单 leader 执行、follower 返回 `NOT_IMPLEMENTED`）。
  - `scripts/uav_executor_node.py`：装配 ego driver（由 `driver` 参数选择 `mavros_position|mock|ego_swarm`）。
  - `config/executor_defaults.yaml`：新增 ego driver 配置段。
  - `launch/uav_executor_ego.launch`：机载 ego 执行入口。
  - `test/test_ego_swarm_driver.py`、`test/test_executor_route_follow.py`。
- `src/swarm_uav_interfaces/CMakeLists.txt`、`README.md`：注册 `UavTrajectoryIntent` 并记录语义。
- `src/tcp_to_ros/config/topology_group_a_sim.yaml`：为需要互相避让的 UAV 增加 UAV↔UAV topology 条目（如交叉/换位场景中 A01↔A02↔A03）；新增轨迹意图 topic。
- `src/swarm_topology_bridge/`：仅配置/ README 示例更新；不改通用 bridge 代码（除非冒烟证明通用缺陷）。

### 删除或移动

- 不删除现有 ROS 消息/Service；不修改冻结计划 `implementation_plan_26080804.md`、`implementation_plan_26080900.md`。
- `references/ego-planner-swarm` 保持原样（可记 commit；不 push，用户无 push 权限，不配置 remote）。
- 不把独立仓库登记为工作空间根仓库的 submodule；不配置 remote、不 push。

[Functions]

### 新函数

- `ego_planner_driver` 内：
  - `bool planOnce(const GoalPoint& start, const vector<WayPoint>& sparse_waypoints, TimedTrajectory& out)`：原版内核包装，输入稀疏关键点输出带时轨迹。
  - `void onNeighborIntent(const UavTrajectoryIntent::ConstPtr&)`：乱序去重 + planning_range 过滤后进入缓冲。
  - `void onNeighborOdometry(const Odom::ConstPtr&)`：邻机实时位姿更新。
  - `void publishSetpoint(const TimedTrajectory&)`：≥30 Hz setpoint 发布。
  - `void publishTrajectoryIntent()`：本机轨迹广播。
  - `bool checkClearance(const TimedTrajectory&, double clearance)`：对外暴露原版避碰罚项完成度检查（供测试/门禁用）。
- `swarm_uav_executor/executor.py`：
  - `_dispatch_command(record, goal)`：按 command 路由（MOVE_TO/FAULT_EXIT/HOVER → ego；FOLLOW_ROUTE leader → ego；follower → NOT_IMPLEMENTED）。
- `swarm_uav_executor/drivers/ego_swarm.py`：
  - `EgoSwarmDriver.start_move_to(goal, cancel_event, deadline)`、`hold(...)`、`health()`、`shutdown()`：实现 `MotionDriver` 抽象；hold 时取消当前轨迹并发布当前位姿 setpoint。

### 修改函数

- `uav_executor_node.py::build_driver`：增加 `ego_swarm` 分支。
- `gcs_a_backend_node.py`：P1 仅把 FOLLOW_ROUTE 作为单机 leader 命令透传（与现有 `MOVE_TO` 分发一致）；编队矩阵实例化归 P2。

### 移除函数

- 无。P1 不做优先级单向改造，不删原版避碰罚项。

[Classes]

### 新类

- `EgoPlannerCore`（C++，`ego_planner_driver`）：封装 bspline 优化内核，纯数值无 ROS 依赖（便于单测）。
- `TrajectoryIntentRelay`：本机意图发布 + 邻机意图接收/去重。
- `EgoSwarmDriver`（`swarm_uav_executor/drivers/ego_swarm.py`）：`MotionDriver` 实现，桥接 `ego_planner_driver`。
- `NeighborIntentSet`：planning_range 过滤后的邻机轨迹集合，供内核避碰。

### 修改类

- `UavTaskExecutor`：命令路由扩展；维持单活动任务与 HOLD 抢占。
- `ExecutorConfig`：新增 ego driver 参数分组（与 `SafetyLeaseConfig` 一样按组装配，避免增长长参数列表）。

### 移除类

- 无。

[Dependencies]

- 新增 `ego_planner_driver` 对 ego-planner-swarm 内核源码的编译依赖（C++、Eigen、OSQP 或原版内联求解器）；不新增 pip 全局包。
- `swarm_uav_interfaces` 增加 `geometry_msgs` 依赖（`UavTrajectoryIntent` 含 `geometry_msgs/Pose`）。
- 复用 `swarm_topology_bridge` services 分支；不改通用桥代码，不新增其业务依赖。
- 不 push、不配置 remote（用户无 push 权限）。

[Testing]

1. 单元（C++/Python）：
   - `EgoPlannerCore`：稀疏→带时轨迹数值正确、约束可行性（速度/加速度）、取消/重启、邻居缺失（无邻机轨迹时退化为单机规划）。
   - `TrajectoryIntentRelay`：乱序去重、planning_range 过滤。
   - 回归 `catkin run_tests swarm_uav_executor` 与 `catkin run_tests tcp_to_ros` 原有全部用例。
2. 桥接/多 Master：
   - 双 Master 机间直连：轨迹意图 + 位姿 topic 互通；reciprocal 避碰注入（交叉穿越两机成功错开）。
   - 三 Master：15 m 集结 → 12 m 单机 route（leader）→ 8 m fault-exit；leader 参考 1 s 超时 → HOLD/LEADER_LOST。
3. SITL 分级：
   - 单机轨迹闭环 → 双机交叉/换位 → 三机错位/换位；全程记录启动流程与结果（测试记录规范）。
4. 全链路安全回归：
   - 断链/SUSPECT/LOST / 租约过期 HOLD / RECOVERED；END 门禁与旧 epoch 拒。
5. 门禁：P1 完成仅声明“ego 内核驱动单机/多机 SITL 闭环 + 启动流程记录规范”；实机上桨仍属 Phase 6。

[Implementation Order]

1. 保存基线：确认四个仓库与 `references/ego-planner-swarm` commit 并记录；工作树无无关改动。
2. 新增 `ego_planner_driver` C++ 包（只抽取 bspline 内核 + 轨迹生成，排除感知外挂），单包编译 + 最小单测。
3. 新增/固定机间轨迹意图消息与 `EgoSwarmDriver`；`UavTaskExecutor` 接入（MOVE_TO/FAULT_EXIT/HOVER → ego；FOLLOW_ROUTE 单 leader）。
4. 更新 `topology_group_a_sim.yaml` 增加 UAV↔UAV 直连；双 Master 机间轨迹/位姿冒烟。
5. SITL 逐级：单机轨迹闭环→双机 reciprocal（交叉/换位）→三机错位/换位；按测试记录规范记录。
6. 全链路安全回归：断链/SUSPECT/LOST/租约过期 HOLD/leader 1 s HOLD/END/旧 epoch 拒。
7. 文档收口 + 各独立仓库按 Conventional Commit 提交（feat/deps/docs，含中文说明行）；不配置 remote、不 push。