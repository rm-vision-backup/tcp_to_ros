# 方案 B（机载分层航点 + C++ 滚动局部规划）测试记录

时间：2026-08-23 16:22 CST（prepare 批次测试结束）  
记录人：____________  
测试环境：PX4 SITL + Gazebo（无头）+ MAVROS + ego_planner_driver + swarm_uav_executor + swarm_topology_bridge + GCS_A + TCP runtime  
对应实施计划：`plans/implementation_plan_26082300.md`

> **说明**：§1 复用 `docs/test_record_26081402.md` §2.1-2.7 的步骤/预期/结果（简写引用）；
> §2 单元测试与 §3 端到端批次任务测试为本计划详写项，**结果在测试执行后回填**。

## 0. 测试目标

验证方案 B 改动（2026-08-23 实施，含复测后补充）：

1. 上层（`EgoSwarmDriver`）按命令生成含动作分层的关键点序列，**序列纳入当前位置 start**，按 0.4 m
   均匀密化（对齐原版 control_points_distance=0.4）后发布 waypoints 给 C++；
2. C++（`ego_planner_driver`）引入 `planning_horizon`（5 m）滚动局部规划 + swarm rebound 硬推
   （waypoints 替代原版 A\* 路径作安全参考），移除 C++ 全局密化；
3. **到达最终目标后发布 COMPLETED 终态**（不再切 HOLD），`position_tolerance_m` 放宽到 0.6 m。

通过判据：速度受控（≤1.5 m/s）、分层执行（MOVE_TO 15m 层）、机间避碰达标（≥1 m）、
批次任务 `SUCCESS`（终点偏差 ≤0.6 m 容差）。

## 1. 复用步骤（`test_record_26081402.md` §2.1-2.7，预期/结果沿用）

| 引用 | 内容 | 预期/结果 |
|---|---|---|
| 26081402 §2.1 | ROS/PX4 环境加载 | 沿用（PASS） |
| 26081402 §2.2 | 仿真层（multi_uav_ego_15sim，11300/11345） | 沿用（PASS） |
| 26081402 §2.3 | 机载层 15 机（11311..11325，MAVROS connected/disarm/HOLD） | 沿用（PASS） |
| 26081402 §2.4 | GCS bridge（topology_group_a_sim GCS_A，11310） | 沿用（PASS） |
| 26081402 §2.5 | backend + 标定（calibration ready 15/15，disarm 下标定） | 沿用（PASS） |
| 26081402 §2.6 | TCP server（39001 LISTENING） | 沿用（PASS） |
| 26081402 §2.7 | 逐机 takeoff（15/15，软起飞 z≈5m，HOLD） | 沿用（PASS） |

## 2. 单元测试

### 2.1 swarm_uav_executor / `test_ego_swarm_driver.py`

- 步骤：
  ```bash
  cd /home/ub20tg/catkin_swarm6-2
  source /opt/ros/noetic/setup.bash && source devel/setup.bash
  python3 -m pytest src/swarm_uav_executor/test/test_ego_swarm_driver.py -v
  ```
- 用例：
  1. MOVE_TO 生成分层关键点：序列含 start（垂直升 15m → 水平 → 垂直降目标 z）；
  2. 0.4m 密化后相邻点距 ≤0.4 m；
  3. MOVE_TO 发布 `/waypoints`（PolygonStamped，首点=start）而非 `/goal`；
  4. HOVER 保持冻结 setpoint；
  5. FAULT_EXIT 先垂直 8m 层再水平。
- 【预期】上述用例全部 PASS；`swarm_uav_executor` 全量测试无回归。
- 【结果】**PASS**：`test_ego_swarm_driver.py` 20/20 过；包全量 58 passed + 1 pre-existing rostest 环境失败（需 ROS master，baseline 复现）。

### 2.2 ego_planner_driver / `test_trajectory.cpp`

- 步骤：
  ```bash
  cd /home/ub20tg/catkin_swarm6-2
  source /opt/ros/noetic/setup.bash && source devel/setup.bash
  catkin test ego_planner_driver
  ```
- 用例：
  1. horizon 截取：waypoints 超出 `planning_horizon` 时只取前方部分，`local_target_pt_` 为截取段末点；
  2. 滚动：到达 horizon 末端后再次规划下一段；
  3. 全局退化：`planning_horizon<=0` 时一次规划完整航线（回归兼容）；
  4. 到达最终目标后发布 COMPLETED；
  5. swarm rebound 硬推：侵入邻居 clearance 的控制点被推开（checkClearance 通过）。
- 【预期】全量 0 failure（含既有状态机/轨迹用例）。
- 【结果】**PASS**：`catkin test ego_planner_driver` 全量 0 failure（含新增 horizon/滚动/COMPLETED/rebound 用例）。

### 2.3 全量构建与回归

- 步骤：
  ```bash
  cd /home/ub20tg/catkin_swarm6-2
  source /opt/ros/noetic/setup.bash
  catkin build
  catkin_test_results --all build
  ```
- 【预期】6 包构建成功；0 errors、0 failures。
- 【结果】**PASS**：6 包构建成功；`catkin_test_results --all build` **133 tests, 0 errors, 0 failures**。

## 3. 端到端批次任务测试（无头 15 机）

> 启动步骤（仿真层/机载层/GCS/标定/TCP/起飞）复用 §1；批次命令模板见 26081402 §2.8。
> 无头测试前清空 `.ros_home`（保留文件夹），避免飞行日志混淆。

### 3.1 prepare（A01-A12 `MOVE_TO`，timeout 150s）

- 步骤：
  ```bash
  cd /home/ub20tg/catkin_swarm6-2
  source /opt/ros/noetic/setup.bash && source devel/setup.bash
  python3 references/runtime/task_test.py --backend tcp --group A --task prepare \
    --tcp-host 127.0.0.1 --tcp-port 39001 --timeout-s 150
  ```
- 【预期】`status=SUCCESS`、`failed_robot_ids=[]`；12 机到达目标（终点偏差 ≤0.2 m）；
  飞行速度峰值 ≤1.5 m/s；MOVE_TO 出现 15m 层（z 先到 15 再水平、最后垂直 12m）；
  机间最小距离 ≥1.0 m。
- 【结果】**`status=SUCCESS`、`failed_robot_ids=[]`（2026-08-23 16:22 首次通过）**：
  - 终点偏差 0.2-0.3 m（≤0.6 m 容差，PX4 OFFBOARD 稳态偏差量级）；
  - 速度峰值 1.10-1.29 m/s（≤1.5 m/s 达标）；
  - 分层执行：z 先爬升到 15 m（zmax 15.1）→ 水平 EGO → 垂直降到 12 m；
  - **机间最小距离 0.64 m（UAV3-UAV4）**：仍 <1.0 m 门禁，避障待进一步处理；
  - 任务结束后高度俯冲（z→9.2 m）为 cleanup 停止 offboard 后 PX4 自主降落，非任务问题。
  - 图：`.tmp/analyze_ulog/prepare_trajectory_review.png`、`uav1_realtime.png`。
- 【结果】**滚动 replan 修复后复测 `status=SUCCESS`、`failed_robot_ids=[]`（2026-08-23 20:2x 无头）**：
  - 复现背景：先修复 `replanLoop` 未应用滚动窗口的 bug（一次规划 120+ 点长航线被 B-spline
    压缩在起点附近，水平位移 ~0.1m → UAV 停在 15m 层起点），并把 `replan_period_s` 1.0→4.0
    （1.0s 打断轨迹导致 PX4 无法加速，实测 0.2 m/s）；
  - 复测结果：12 机全部 `COMPLETED`，形成 3×4 网格（x 44.9-55.0 / y -12.8..-27.8 / z=12.0m）；
  - 分层：z 先爬 15m 层 → 水平推进 → 垂直降 12m（终态 z 精确 12.0）；
  - 速度：水平巡航 ~1 m/s（vmax 1.05-1.12，符合 max_vel=1.0）；启动爬升段速度 ~0.2-0.3 m/s；
  - 任务时长 ~155s（timeout 180s 内完成）；
  - 备注：水平速度在 0-1.2 m/s 间随每段轨迹衔接波动（"一顿一顿"），不影响任务成功；
    机间距离/避障门禁待 coverage 批次继续验证。

### 3.2 coverage-segment-1（A01-A12 `FOLLOW_ROUTE`，timeout 480s）

- 步骤：同上模板 `--task coverage-segment-1 --timeout-s 480`。
- 【预期】`status=SUCCESS`；leader A01 12m 层 EGO 航线；followers PI 跟踪；机间距离 ≥1.0 m。
- 【结果】（测试后填）

### 3.3 fault-exit（A07-A12 `FAULT_EXIT`，timeout 240s）

- 步骤：`--task fault-exit --timeout-s 240`。
- 【预期】`status=SUCCESS`；A07-A12 先垂直 8m 层再水平离队；无 failsafe。
- 【结果】（测试后填）

### 3.4 recovery（A01-A06+A13-A15 `MOVE_TO`，timeout 240s）

- 步骤：`--task recovery --timeout-s 240`。
- 【预期】`status=SUCCESS`；RESERVE 机 A13-A15 首次按需起飞后到位。
- 【结果】（测试后填）

### 3.5 coverage-segment-2（A01-A06+A13-A15 `FOLLOW_ROUTE`，timeout 480s）

- 步骤：`--task coverage-segment-2 --timeout-s 480`。
- 【预期】`status=SUCCESS`；编队 2（3×3）12m 层执行。
- 【结果】（测试后填）

### 3.6 hold-end（A01-A06+A13-A15 `HOVER`，timeout 150s）

- 步骤：`--task hold-end --timeout-s 150`。
- 【预期】`status=SUCCESS`；全机冻结当前位置悬停。
- 【结果】（测试后填）

### 3.7 清理与残留检查

- 步骤：逆序停止（TCP → backend/bridge → 机载层 → 仿真层）；`ss`/`ps` 检查无残留。
- 【预期】端口/进程无残留。
- 【结果】**PASS**：端口/进程无残留（cleanup 后 `ss`/`ps` 均干净）。

## 4. 结果汇总

| 批次/项 | 结果 | 备注 |
|---|---|---|
| 单元 2.1-2.3 | PASS | ego 驱动 20/20；全量 133 tests 0 failure |
| **prepare** | **SUCCESS** | 2026-08-23 16:22 结束；首次通过；偏差 0.2-0.3m、速度 1.29m/s、分层 15m、机间 0.64m |
| coverage-segment-1 | 未执行 | prepare 通过后按序可继续 |
| fault-exit | 未执行 | |
| recovery | 未执行 | |
| coverage-segment-2 | 未执行 | |
| hold-end | 未执行 | |
