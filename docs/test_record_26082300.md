# 方案 B（机载分层航点 + C++ 滚动局部规划）测试记录

时间：2026-08-23 00:xx CST  
记录人：____________  
测试环境：PX4 SITL + Gazebo + MAVROS + ego_planner_driver + swarm_uav_executor + swarm_topology_bridge + GCS_A + TCP runtime  
对应实施计划：`plans/implementation_plan_26082300.md`

> **说明**：§1 复用 `docs/test_record_26081402.md` §2.1-2.7 的步骤/预期/结果（简写引用）；
> §2 单元测试与 §3 端到端批次任务测试为本计划详写项，**结果在测试执行后回填**。

## 0. 测试目标

验证方案 B 两项核心改动：

1. 上层（`EgoSwarmDriver`）按命令生成含动作分层的关键点序列并 2m 均匀密化，作为 waypoints 发布给 C++；
2. C++（`ego_planner_driver`）引入 `planning_horizon` 滚动局部规划（对齐原版 ego-planner-swarm
   receding-horizon），移除 C++ 全局密化。

通过判据：速度受控（≤1.5 m/s）、分层执行（MOVE_TO 15m 层）、机间避碰达标（≥1 m）、
批次任务 `COMPLETED`（终点偏差 ≤0.2 m 容差）。

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
  1. MOVE_TO 生成分层关键点：垂直升 15m → 水平 → 垂直降目标 z；
  2. 2m 密化后相邻点距 ≤2.0 m；
  3. MOVE_TO 发布 `/waypoints`（PolygonStamped）而非 `/goal`；
  4. HOVER 保持冻结 setpoint；
  5. FAULT_EXIT 先垂直 8m 层再水平。
- 【预期】上述用例全部 PASS；`swarm_uav_executor` 全量测试无回归。
- 【结果】（测试后填）

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
  3. 全局退化：`planning_horizon<=0` 时一次规划完整航线（回归兼容）。
- 【预期】全量 0 failure（含既有状态机/轨迹用例）。
- 【结果】（测试后填）

### 2.3 全量构建与回归

- 步骤：
  ```bash
  cd /home/ub20tg/catkin_swarm6-2
  source /opt/ros/noetic/setup.bash
  catkin build
  catkin_test_results --all build
  ```
- 【预期】6 包构建成功；0 errors、0 failures。
- 【结果】（测试后填）

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
- 【结果】（测试后填）

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
- 【结果】（测试后填）

## 4. 结果汇总（测试后填）

| 批次/项 | 结果 | 备注 |
|---|---|---|
| 单元 2.1-2.3 | | |
| prepare | | |
| coverage-segment-1 | | |
| fault-exit | | |
| recovery | | |
| coverage-segment-2 | | |
| hold-end | | |
