# min-snap 统一单段重构：轨迹平滑 + 双机不安全 HOLD 暂停实施计划

日期：2026-08-24 15:52

状态：**待实施**

## 1. 目标与边界

### 1.1 成功判据

- **单机飞行平滑**：速度无剧烈震荡——滚动段切换前后速度/加速度连续，不存在与 `replan_period_s` 同步的周期性/阶跃性速度波动。
- **双机不安全时 HOLD 暂停**：执行层运行时距离门禁触发本机 HOLD，任务以 `FAILED/COMMAND_HELD` 收口。

### 1.2 本次不实施（边界）

- **避碰行为不实现**：不实施机间主动规避（reciprocal 软惩罚之外的规避、rebound 硬推）。未完成任务即 HOLD 作为避碰失败兜底，后续再考虑。
- **不实施**：rebound 硬推、reparam（时间重参数化）、原版 2.1 完整安全机制（safety timer / 在线碰撞预测 / 紧急停止状态机）。
- `replan_period_s` 保持 1.0（本计划不修改）。

## 2. 背景与根因

当前基线（方案 B，`implementation_plan_26082300.md` + `implementation_plan_26082322.md`）：

- 上层 `EgoSwarmDriver` 按命令生成分层关键点（含 start）并 **0.4m 均匀密化** 发布 `/waypoints`；
- C++ `planOnce` 直接把窗口内 waypoints 作为 point_set，`parameterizeToBspline` 初始化 B-spline 后 L-BFGS 优化，随后 `setPhysicalLimits/checkFeasibility/lengthenTime`（时间重参数化）与 `reboundFromNeighbors`（硬推）兜底；
- 滚动窗口 `advanceConsumed`（实际位置投影）+ `truncateToHorizon`（5m）已稳定；PositionTarget P/V/A + `setpoint_relay` 唯一出口已冻结。

遗留问题：

1. 0.4m 密化点直接进 B-spline：窗口内控制点密集，初始 B-spline 贴近折线，jerk 平滑代价与可行性软惩罚互相拉扯，滚动段切换时速度仍可能出现震荡；
2. 时间重参数化 `lengthenTime` 在中长段上改变时间轴，与 1s 滚动配合时速度谱不稳定；
3. 避碰依赖优化层 swarm 软惩罚 + rebound 硬推，实测机间最小距离不稳定（prepare 曾 0.64m < 1.0m 门禁）；执行层运行时距离门禁目前只覆盖 follower PI，不覆盖 EGO 巡航。

本计划采用原版 ego-planner-swarm 的 **min-snap 单段多项式初始轨迹**（`one_segment_traj_gen`）重构初始 point_set：五次多项式闭式解一次性连接 [起点状态] → [窗口末点]，再按 **0.4m 弧长等距** 采样生成 B-spline 参数化输入。多项式初始轨迹保证 P/V/A 边界精确插值，初始 B-spline 速度剖面平滑；滚动段间通过"起点 = 旧轨迹 P/V/A"保持连续性。机间安全改由**执行层运行时距离门禁**兜底。

## 3. 数据链改动（冻结）

```text
上层（EgoSwarmDriver）
  原始 keypoints（start + 层点 + 弓字转折 + 目标）
    → 最大 2m 密化（代替当前 0.4m）
    → 发布 /waypoints（PolygonStamped）

C++（ego_planner_driver）
  [当前状态] + 窗口内 2m 密化点（truncateToHorizon 复用）
    → min-snap 单段 one_segment_traj_gen
        · 起点 = 旧轨迹 P/V/A（滚动连续性）
        · end_vel = 末点切向 × min(v_cruise, max_vel)；end_acc = 0
        · 仅最终段 end_vel = 0（静止到达）
        · 梯形时间 T（适配起点非零速度：v0 投影，加速-匀速-减速）
    → 0.4m 弧长等距采样 point_set
    → parameterizeToBspline
    → L-BFGS（jerk + feasibility 软惩罚）
    → 发布 PositionTarget P/V/A
```

- 2m 密化点只用于**窗口推进（advanceConsumed）/ 末点参考**，**不直接进 B-spline**；进 B-spline 的是 min-snap 多项式按 0.4m 弧长等距采样的 point_set。
- PositionTarget 契约（EGO mask 2048 / follower 2496 / HOLD 2552）与 `setpoint_relay` 唯一出口保持 `implementation_plan_26082322.md` 冻结不变。

## 4. 关键决策（问题 1-11 已确认）

适用范围：当前单段方案同时覆盖**现测试的 MOVE_TO**与**后续将测试的 FOLLOW_ROUTE**——`planOnce` 不区分命令类型，一律按"窗口内起点状态 → 窗口末点"生成单段轨迹，仅最终段 `end_vel=0`；分层/弓字关键点由上层 `_build_layered_keypoints` 保证，单段多项式只负责连接段内状态。

| 项 | 决策 |
|---|---|
| min-snap 形态 | **统一单段** `one_segment_traj_gen`（覆盖 MOVE_TO 与 FOLLOW_ROUTE）；多段 `minSnapTraj` 仅作**备用方案记录**（单段无法处理时的修改方案，本计划不实施） |
| keypoints 密化 | 上层最大 **2m**（替代当前 0.4m），作窗口推进/末点参考，不进 B-spline |
| 窗口 | 现有 `advanceConsumed`（实际位置投影）+ `truncateToHorizon`（5m）直接复用 |
| 采样 | min-snap 多项式 → **0.4m 弧长等距** point_set |
| 边界导数 | `start = 旧轨迹 P/V/A`；`end_vel = 末点切向 × min(v_cruise, max_vel)`，`end_acc = 0`；仅最终段 `end_vel = 0` |
| 梯形时间 T | 适配起点非零速度（v0 投影，加速-匀速-减速） |
| 停用 | rebound 硬推、reparam（时间重参数化）：加参数开关，默认关 |
| 安全 | **执行层运行时距离门禁**：复用 `neighbor_odom_topics`，水平 <1.0m 且垂直 <2.0m → 本机 HOLD 暂停 |
| 周期 | `replan_period_s` 保持 **1.0**（本计划不改） |

### 边界导数与梯形 T 约定（细化）

- `start`：当前滚动段切换时刻，从旧轨迹同一 `t_switch` 采样位置/速度/加速度（与 26082322 PositionTarget 契约一致，杜绝段间 P/V/A 归零跳变）。
- `end_vel`：窗口末点切线方向（窗口内 2m 密化折线方向），幅值 `min(v_cruise, max_vel)`；最终段（无剩余航点）`end_vel = 0`。
- `end_acc = 0`（巡航/静止段无持续加速度）。
- 梯形时间：给定 v0（起点速度在段方向上的投影）、v_cruise、加速度限制 a，按加速-匀速-减速三段构造时间 T；T 必须正定有限，作为 `one_segment_traj_gen` 的段时长。

### 备用方案：多段 minSnapTraj（**不实施**，仅记录）

> **定位**：多段方案**不是**为"弓字 keypoints 场景"预留的后续功能，而是**当前单段方案无法处理时的修改方案**。
> 当前单段方案适用于现在测试的 **MOVE_TO** 与后续将测试的 **FOLLOW_ROUTE**；仅当单段出现下述触发条件、验收不通过时，
> 才按本子节修改 `planOnce` 切换为多段，且切换后需重新走完整验收。

**触发条件**（满足任一即评估切换）：

- 窗口内 2m 密化点含方向突变关键点（弓字转折、分层切换点），单段五次多项式绕行/偏移超阈值；
- 单段端到端位移过大，梯形 T 无法把峰值速度/加速度压到 `max_vel`/`max_acc` 内；
- 单段可行性检查或 L-BFGS 后速度/加速度仍超限、平滑度不达标。

**实现内容**（移植原版 `references/ego-planner-swarm/src/planner/traj_utils/src/polynomial_traj.cpp` 的 `minSnapTraj`）：

- 签名：`PolynomialTraj::minSnapTraj(Pos, start_vel, end_vel, start_acc, end_acc, Time)`；
  - `Pos`：`2×(seg_num+1)` 关键点矩阵（每段起止位置，中间点共享）；
  - `Time`：`seg_num` 维每段时长向量（按上面"梯形时间 T"约定逐段计算，含 v0 投影）；
  - 每段 6 阶多项式，`poly_coeff` 为 `seg_num×18`（Px/Py/Pz 各 6 系数）；
- 固定导数：首段起点 pos/vel/acc、末段终点 pos/vel/acc；中间段位置连续，中间段起点/终点导数作为自由变量（`2*seg_num-2` 个）；
- 闭式解：每段构造映射矩阵 A（6×6，`Factorial × Time^k`）与选择矩阵 C'；按 Rff/Rfp/Rpf/Rpp 分块，`Dp = -(Rpp^{-1} · Rfp^T) · Df`，`P = A^{-1} · C · D1`；
- 分段点：窗口内 2m 密化点中的关键转折点（弓字转折、层点、目标）作为 `Pos` 列；
- 采样与优化与单段一致：各段按 0.4m 弧长等距采样合并 `point_set` → `parameterizeToBspline` → L-BFGS（jerk + feasibility 软惩罚）；
- 段间导数连续由 min-snap 闭式解保证，与单段"起点=旧轨迹 P/V/A"的滚动连续性语义一致。

**不实施声明**：本计划（min-snap 统一单段重构）**不实现、不启用**多段 `minSnapTraj`；`planOnce` 不包含多段分派逻辑，`one_segment_traj_gen` 是唯一初始轨迹生成路径。本子节仅作记录，供单段验收未通过时参考修改。

## 5. 分阶段实施

### 阶段 1：上层 keypoints 2m 密化（替换 0.4m）

修改 `src/swarm_uav_executor`：

- `src/swarm_uav_executor/src/swarm_uav_executor/drivers/ego_swarm.py`：
  - `waypoint_densify_spacing` 默认 0.4 → **2.0**；
  - `_densify_waypoints` 语义不变（逐段均匀插值使相邻点距 ≤ max_spacing），仅间距阈值改为 2m；
  - `_build_layered_keypoints` 保留（start + 层点 + 弓字转折 + 目标按命令生成）。
- `src/swarm_uav_executor/config/executor_defaults.yaml`：`waypoint_densify_spacing: 2.0`，注释同步更新。
- `src/swarm_uav_executor/launch/uav_executor_ego.launch`：`ego_swarm/waypoint_densify_spacing` 0.4 → 2.0。
- `src/swarm_uav_executor/test/test_ego_swarm_driver.py`：密化断言更新为 ≤2m；新增 2m 密化点内容断言（首点=start、含层点/弓字转折/目标）。

### 阶段 2：原版 polynomial_traj 移植（one_segment_traj_gen 单段）

- 来源：`references/ego-planner-swarm/src/planner/traj_utils/src/polynomial_traj.cpp` 的 `one_segment_traj_gen`（6×6 五次多项式闭式解，min-snap 单段）与 `polynomial_traj.h` 的 `addSegment`/求值接口。
- 移植到 `ego_planner_driver`（新建 `polynomial_traj.h/cpp`）：
  - `PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, end_pt, end_vel, end_acc, t)`；
  - `evaluate(t)` / 一阶导数（切线/速度）求值；
  - **不移植 `minSnapTraj`（多段）**——作为备用方案记录于第 4 节，本计划不实施。
- 单测：边界 P/V/A 精确插值；端点速度方向正确；T 正定有限。

### 阶段 3：C++ planOnce：min-snap 初始 point_set + 边界导数 + 梯形 T

修改 `src/ego_planner_driver`：

- `include/ego_planner_driver/ego_planner_core.h`（`EgoPlannerConfig` 新增）：
  - `bool enable_rebound = false;`（控制 `reboundFromNeighbors`，默认关）
  - `bool enable_time_reparam = false;`（控制 `setPhysicalLimits/checkFeasibility/lengthenTime` 的时间轴修改，默认关）
  - `double cruise_velocity = 1.0;`（v_cruise，`end_vel` 幅值上限）
  - `double acceleration_limit = 2.0;`（梯形 T 用；实施时以现状核对）
  - `planOnce` 签名保留（start/start_vel/start_acc 已存在）。
- `src/ego_planner_driver/src/ego_planner_core.cpp` `planOnce`：
  - 移除"start + sparse_waypoints → 细分 while"的旧 point_set 构建；
  - 改为：`end_pt = point_set.back()`（窗口末点 = `truncateToHorizon` 段的末点，含最终目标）；按第 4 节计算 `end_vel`/`end_acc` 与梯形 T；调用 `one_segment_traj_gen`；
  - 对多项式按 **0.4m 弧长等距**采样生成 `point_set`（`evaluate` + 一阶导数做弧长累计）→ `parameterizeToBspline`；
  - 起始导数数组 `[start_vel, end_vel, start_acc, end_acc]` 传入 `parameterizeToBspline`；保留 P0..P2 精确锚定；
  - L-BFGS 保留 jerk + feasibility 软惩罚（`lambda_smooth`/`lambda_feasibility`）；`lambda_swarm`/`lambda_terminal` 按开关/参数保留；
  - `enable_rebound=false` 时跳过 `reboundFromNeighbors`；`enable_time_reparam=false` 时跳过 `lengthenTime`（`setPhysicalLimits` 的 max 检查与 `checkFeasibility` 仅作诊断/上报，不改变时间轴）；
  - 采样输出与 `checkClearance` 硬校验兜底保留。
- `src/ego_planner_driver/src/ego_planner_driver_node.cpp`：
  - 构造函数读取 `enable_rebound`（默认 false）、`enable_time_reparam`（默认 false）、`cruise_velocity`；
  - 滚动段切换时 `start_vel/start_acc` 由旧轨迹在切换时刻求导采样（现状已具备，确认逻辑不变）；
  - "最终段"判断复用 `waypoint_consumed_`/剩余航点（现有 `near_final` 逻辑）；`truncateToHorizon`/`advanceConsumed`/`waypoint_consumed_` 与 `replan_period_s=1.0` 不变。
- `src/ego_planner_driver/launch/ego_planner_driver.launch`：
  - 新增 `enable_rebound`（默认 false）、`enable_time_reparam`（默认 false）、`cruise_velocity`；`swarm_rebound` 废弃或改由 `enable_rebound` 替代；
  - `planning_horizon=5.0`、`replan_period_s=1.0` 保持不变。
- `src/ego_planner_driver/test/test_trajectory.cpp` / `test_ego_state_machine.cpp`：
  - 更新：初始 point_set 来自 min-snap 多项式 0.4m 弧长采样；段间 P/V/A 连续；
  - 新增：one_segment_traj_gen 边界插值、梯形 T、最终段 `end_vel=0`、`enable_rebound=false`/`enable_time_reparam=false` 分支（无硬推/无时间轴拉伸）、滚动两段速度无阶跃。

### 阶段 4：执行层 EGO 巡航距离门禁

修改 `src/swarm_uav_executor/src/swarm_uav_executor/drivers/ego_swarm.py`：

- 复用 `_distance_safe`（现仅 follower loop 使用）扩展至 EGO 巡航监控：
  - EGO 执行期（`EXECUTING`）按 `_MONITOR_HZ` 周期检查：订阅源复用 `neighbor_odom_topics`（默认 `/UAVn/mavros/local_position/odom` 全集）；
  - 水平中心距 < `min_horizontal_distance_m`（1.0m）**且** 垂直中心距 < `min_vertical_distance_m`（2.0m）→ 触发本机 HOLD；
  - HOLD 动作：调 `_issue_hold`（向 C++ 节点发 hold / 置本机 HOLD 目标）→ 本机暂停 → executor 将任务终态报告为 `FAILED/COMMAND_HELD`（避碰失败兜底，不自动恢复）；
  - 邻机 odom 新鲜度校验（复用 `_FOLLOW_POSE_TIMEOUT_S` 语义或独立参数）。
- `config/executor_defaults.yaml` / `launch/uav_executor_ego.launch`：`min_horizontal_distance_m`/`min_vertical_distance_m`/`neighbor_odom_topics` 已暴露，保持默认 1.0/2.0。
- `test/test_ego_swarm_driver.py`：新增 EGO 巡航距离门禁用例（注入邻机 odom 过近 → 断言 HOLD 触发、终态 `COMMAND_HELD`）。

## 6. 测试计划

### 6.1 单元/构建

- `catkin build ego_planner_driver swarm_uav_executor tcp_to_ros` 全过；
- `catkin test ego_planner_driver swarm_uav_executor` + `catkin_test_results --all build` 0 failure；
- C++：one_segment_traj_gen 边界插值、梯形 T、0.4m 弧长采样间距 ≤0.4、`enable_rebound=false`/`enable_time_reparam=false` 分支、滚动段 P/V/A 连续无阶跃、最终段静止；
- Python：2m 密化 ≤2m、EGO 巡航距离门禁 HOLD、follower 门禁回归。

### 6.2 单机无头 prepare（平滑验证）

- 无头 15 机环境，单机或 A01-A12 prepare；
- 判定：速度谱无剧烈震荡（无周期性/阶跃性尖峰，速度峰值 ≤1.5m/s 量级）、分层保持（15m→水平→12m）、终点偏差 ≤0.6m 容差、`COMPLETED` 终态正常。

### 6.3 双机交叉（HOLD 验证）

- 双机交叉/换位场景，注入邻机 odom 过近（水平 <1.0m 且垂直 <2.0m）；
- 判定：触发本机 HOLD 暂停，任务终态 `FAILED/COMMAND_HELD`，无自动恢复/无旧目标接管；HOLD 后 relay 输出固定位置/yaw。

### 6.4 回归

- 26082322 的 PositionTarget 契约（EGO mask 2048 / follower 2496 / HOLD 2552）与 relay 唯一出口不回归；
- `replan_period_s` 仍为 1.0；`planning_horizon` 仍为 5.0。

## 7. 实施顺序与提交边界

1. 上层：keypoints 2m 密化（替换 0.4m）→ Python 单测；
2. 原版移植：`polynomial_traj`（one_segment_traj_gen 单段，去 minSnapTraj 依赖）→ C++ 单测；
3. C++ `planOnce`：min-snap 初始 point_set 生成 + 边界导数 + 梯形 T；加 `enable_rebound`/`enable_time_reparam` 参数默认 false；
4. 执行层：EGO 巡航距离门禁（复用 `_distance_safe` 逻辑扩展）；
5. 单测回归 + 单机无头 prepare（平滑验证）+ 双机交叉（HOLD 验证）；
6. 文档落盘：`plans/README.md` 状态更新、新 `docs/test_record_2608xxxx.md`（按测试归档命名规范，日期取实施当日）；
7. 按仓库提交（`ego_planner_driver`、`swarm_uav_executor`；Conventional Commit，不 push）。

## 8. 回滚方案

- 基线：min-snap 集成前的快照 commit（对应 `chore: snapshot before minsnap integration`，见工作区 `.tmp/commit_msgs/commit_snapshot_minsnap.txt`）；
- 回滚需同时恢复上层密化（0.4m）、C++ planOnce 初始 point_set 构建、launch 参数与测试，禁止只回滚部分；
- 若 min-snap 单段在弓字/长水平段表现不佳，需在切回 `enable_rebound=true`/`enable_time_reparam=true` 后重新验收，不作为默认配置；
- SITL 坐标/字段验证失败时停止 OFFBOARD 测试并整体回滚，不以业务层临时换轴绕过。

## 9. 完成定义

- 单机飞行平滑：速度无剧烈震荡（与 replan 无关的周期性/阶跃波动消失），段间 P/V/A 连续；
- 双机不安全：运行时距离门禁触发本机 HOLD 暂停（`FAILED/COMMAND_HELD`），无自动恢复；
- `one_segment_traj_gen` 单段初始轨迹落地，覆盖 MOVE_TO 与 FOLLOW_ROUTE 测试；多段 `minSnapTraj` 仅作备用方案记录（单段无法处理时的修改方案），本计划不实施；
- 上层 2m 密化、0.4m 弧长采样、梯形 T、边界导数约束全部落地且单测覆盖；
- `enable_rebound`/`enable_time_reparam` 参数默认 false；
- `replan_period_s=1.0`、PositionTarget 契约、relay 唯一出口不回归；
- 单测回归 0 failure；单机无头 prepare 平滑验证 + 双机交叉 HOLD 验证通过并写入测试记录。



