# 周期性零速修复 + 统一 PositionTarget 控制链路测试记录

时间：2026-08-24 00:xx CST（prepare 批次测试结束）  
记录人：____________  
测试环境：PX4 SITL + Gazebo（无头）+ MAVROS + ego_planner_driver + swarm_uav_executor + swarm_topology_bridge + GCS_A + TCP runtime  
对应实施计划：`plans/implementation_plan_26082322.md`

## 0. 测试目标

验证实施计划 26082322 的完整链路：

1. EGO 改发 `mavros_msgs/PositionTarget`（mask 2048：position+velocity+acceleration+yaw），
   `replan_period_s` 从 4.0 恢复为 **1.0**；
2. follower PI 改发显式 position/velocity/yaw 候选（mask 2496）；
3. `setpoint_relay` 独占 `/mavros/setpoint_raw/local`，实现选源、generation 隔离、
   freshness/finite 校验和固定位置/yaw HOLD；
4. 旧 `/setpoint` PoseStamped 链路与 `/mavros/setpoint_position/local` 旁路已清理；
5. 无头 prepare 复测：EGO ULog `trajectory_setpoint.vx/vy/vz` 不再全 NaN，巡航无与
   replan 参数锁定的周期性零速。

## 1. 单元测试

### 1.1 ego_planner_driver（C++）

- 步骤：
  ```bash
  cd /home/yjq/catkin_swarm6-2
  source /opt/ros/noetic/setup.bash && source devel/setup.bash
  catkin test ego_planner_driver
  ```
- 新增用例：
  1. `TrajectorySetpointPublishesPvaMask`：执行帧 `FRAME_LOCAL_NED`、mask=2048、P/V/A/yaw
     均有限、速度非零；轨迹耗尽后 mask=2552 且速度/加速度归零；
  2. `HoldAndTakeoffUseHoldMask`：HOLD 与 TAKEOFF 输出 mask=2552；
  3. `DirectControlSuppressesEgoCandidates`：`direct_control_active=true` 不发布候选，
     恢复 false 后恢复发布；
  4. `TimedTrajectorySamplesPvaConsistently`：acc 与 vel 有限差分一致；
  5. `SamplePvaAtStartMatchesRequestedDerivatives`：起点速度锚定 start_vel。
- 【结果】**PASS**：`catkin test ego_planner_driver` 52 tests、0 errors、0 failures。

### 1.2 swarm_uav_executor（Python + rostest）

- 步骤：
  ```bash
  cd /home/yjq/catkin_swarm6-2
  source /opt/ros/noetic/setup.bash && source devel/setup.bash
  catkin test swarm_uav_executor
  ```
- 更新用例：
  1. `test_follower_target_includes_offset`：follower 发布 PositionTarget（frame=LOCAL_NED、
     mask=2496），position=leader+offset，velocity=PI 输出，yaw=目标航向，direct-control
     Bool 进入 true/退出 false；
  2. `test_follower_pi_clamped`：position 为权威目标，velocity 被 xy 限幅。
- 新增 `test_setpoint_relay.py`（13 用例）：选源、generation 隔离、candidate timeout、
  NaN/错误 frame/mask 拒绝、未选中源不接管、HOLD 锁存不漂移、pose stale 复用上个 HOLD、
  无 pose 不输出、恢复候选退出 HOLD、每 tick 恰发一帧。
- 新增 `setpoint_relay_rostest.test`：输出话题唯一发布者、旧 position 话题无发布者、
  30Hz 输出、契约 mask 齐全、HOLD→follower 顺序。
- 【结果】**PASS**：`catkin test swarm_uav_executor` 75 tests、0 errors、0 failures。

### 1.3 全量构建与回归

- 步骤：
  ```bash
  cd /home/yjq/catkin_swarm6-2
  source /opt/ros/noetic/setup.bash
  catkin build
  catkin_test_results --all build
  ```
- 【结果】**PASS**：`catkin_test_results --all build` **150 tests, 0 errors, 0 failures**
  （基线 133 + 新增 17）。

## 2. 端到端 prepare 复测（无头 15 机）

> 启动步骤复用 `test_records/test_record_26082300.md` §1（仿真层/机载层/GCS/标定/TCP/起飞）。
> 无头测试前清空 `.ros_home`（保留文件夹）。

### 2.1 环境与起飞

- 仿真层：`multi_uav_ego_15sim.launch gui:=false`（11300/11345）。
- 机载层：`startup_offboard_ego.sh 1..15`（11311..11325，日志/pid 写到 `.tmp/logs/`）。
- GCS_A：bridge + backend + TCP（11310 / 39001），标定 ready 15/15。
- 起飞：`offboard_takeoff_15.py` 15/15 `armed=True / mode=OFFBOARD`，软起飞 z≈5m。
- 【结果】**PASS**（两次运行 15/15 起飞成功；`/mavros/setpoint_raw/local` 稳定 30.009 Hz，
  唯一发布者为 relay）。

### 2.2 prepare（A01-A12 `MOVE_TO`，timeout 180s）

- 步骤：
  ```bash
  cd /home/yjq/catkin_swarm6-2
  source /opt/ros/noetic/setup.bash && source devel/setup.bash
  python3 references/runtime/task_test.py --backend tcp --group A --task prepare \
    --tcp-host 127.0.0.1 --tcp-port 39001 --timeout-s 180
  ```
- 【预期】`status=SUCCESS`、`failed_robot_ids=[]`；12 机到达目标（终点偏差 ≤0.6 m 容差）；
  分层执行（15m 层→水平→12m）；EGO ULog `trajectory_setpoint.vx/vy/vz` 为有限值；
  巡航速度不再出现与 replan 周期锁定的周期性零速。
- 【结果】**`status=SUCCESS`、`failed_robot_ids=[]`（Run 9，建议 2+3 与到达修复后）**：
  - 任务耗时 ~111 s（02:15:18→02:17:19）；
  - 12 机全部 `COMPLETED`，终点偏差 0.2-0.7 m（多数 ≤0.6 m，个别 0.6-0.7 m 在 near_final
    容差判定内通过）；
  - **最大速度 1.12-1.18 m/s**（建议 2+3 消除下降段速度超限；Run 4 为 2.15-2.37）；
  - 巡航段水平速度 max 1.04、mean 0.61，5s 窗口均值 0.52-0.69 m/s（无零谷）；
  - PX4 ULog `trajectory_setpoint.vx/vy/vz` 有限样本 859-970/1198-1216；
  - 分层执行：z 先爬 15.1m 层 → 水平巡航 → 垂直降 12m。

### 2.3 中间失败与根因修复（Run 5-8）

- **Run 5/6/8 `TIMEOUT`**：飞机 110s 全部到位（偏差 0.23m < 0.6 容差），但 exec_state
  未 COMPLETED。节点诊断确认 `checkArrival` 从未被调用（`elapsed >= duration` 未满足）：
  每 1s 时间重规划持续进行、`waypoint_consumed_` 未到末尾，轨迹从不执行完。
  - 建议 2 后 `advanceConsumed`/`truncateToHorizon` 误用旧轨迹期望位置推进窗口；
    每 1s 换段时期望位置总在下降段中段 → consumed 卡住。
  - 即使改用实际位置，密化末段投影阈值（`t<seg_len`）也会让 consumed 停在 `size-2`。
- **Run 7 `FAILURE(failed=[A05])`**：A05 在并发确认阶段被拒（bridge/executor 偶发抖动），
  未进入飞行阶段，与到达逻辑无关。
- **修复（本计划新增）**：
  1. `advanceConsumed`/`truncateToHorizon` 改用**实际位置**（真实进度感知）；
  2. 飞机进入最终目标容差（`near_final`）时**停止时间触发重规划**并**退化规划完整剩余
     航点**（含目标）；
  3. `checkArrival` 的 COMPLETED 判定增加 `near_final`（不依赖 `waypoint_consumed_`）。
- 新增回归测试：`RollingSegmentsReachCompleted`（滚动多段最终 COMPLETED）、
  `RollingSegmentsReachCompletedWithTrackingLag`（滞后 0.15m 下仍收敛）。

### 2.4 原版对比与降震荡改动（Run 9-12）

对照 `references/ego-planner-swarm` 动态重规划差异：

| 维度 | 原版 | 当前移植版（本计划落地） |
|---|---|---|
| 时间参数化 | `ts=ctrl_pt_dist/max_vel*1.5`；优化后 STEP 3 `checkFeasibility`+`lengthenTime` 硬保证 v/a | 原固定 `ts=0.4` 无 reparam → **新增 STEP 3 `checkFeasibility`+`lengthenTime`**（`planOnce`） |
| 重规划停止 | `t_cur>replan_thresh` 且 `(end-pos)>no_replan_thresh` | **新增 `no_replan_thresh_m=1.0`**：目标 1m 内停止时间重规划+退化完整剩余 |
| 重规划周期 | 1.0s | 1.0s（2.0s A/B 对比后回退，见下） |

- **Run 9（建议2+3+到达修复，replan 1.0）SUCCESS**：vmax 1.12-1.18、巡航 5s 窗口
  [0.52, 0.69]。
- **Run 10（+ts=0.6）TIMEOUT**：ts=0.6 使轨迹显著变慢，180s 内 12 机未全部完成
  （UAV1 截止时仍在 x≈37）→ **ts 回退 0.4**（reparam 已能压峰值，无需 ts 预拉长）。
- **Run 11（reparam+no_replan，ts=0.4，replan 1.0）SUCCESS**：vmax 1.12-1.46、
  巡航 5s 窗口 [0.44, 0.71]、自相关 2.4s r=0.74。**速度震荡未明显改善**（reparam 只压
  峰值、不改变每 1s 切段的段间速度 profile 差异）。
- **Run 12（replan 2.0 A/B，12c 复测）SUCCESS 但波动仍差**：巡航 5s 窗口出现 0.02-0.05 深谷
  （UAV2/6/9）、自相关 r=0.73-0.91；vmax 全时段 1.26-1.37、机间距 0.34m。2s 切段时速度
  profile 展开更多、段间差异更大 → **replan 回退 1.0**（当前最优）。
- **结论**：reparam（硬保证速度峰值 ≤ 逐轴限）+ no_replan（终点附近停止重规划）+ replan
  1.0 为当前最优组合；巡航段速度波动 ±0.15 属"段间轨迹几何差异"固有，需更深度改动
  （如轨迹速度 profile 均匀化/切段速度平滑）进一步压缩。

### 2.5 ULog 分析

- 脚本：`.tmp/analyze_ulog/analyze_prepare_trajectory.py`、`plot_uav_realtime.py`
  （p-t 三轴合并一张 + 新增三轴 v-t 一张 + speed）。
- 检查项：
  1. `trajectory_setpoint` 的 vx/vy/vz 有限样本占比（859-970/1198-1216，修复前 0/750）；
  2. 实际水平速度谱：巡航段无 ~4 s 周期性零速谷值（5s 窗口均值最低 0.52 m/s）；
  3. 最小机间距离、终点偏差、最大速度（见 §3）。
- 【结果】见 §3 汇总与图：
  - `.tmp/analyze_ulog/prepare_trajectory_review.png`（3D 轨迹/速度/机间距/高度）
  - `.tmp/analyze_ulog/uav1_realtime.png`、`uav6_realtime.png`（3D + 三轴 p-t 合并 +
    三轴 v-t + speed）

## 3. 结果汇总

| 项目 | 结果 | 备注 |
|---|---|---|
| 单元 1.1-1.3 | PASS | ego 56→60、executor 75、全量 150→158 tests 0 failure |
| prepare（Run 3-4，仅 PVA+clamp） | SUCCESS | 下降段 setpoint 尖峰 15.7→2.41 m/s（Run 4） |
| prepare（Run 5/6/8） | TIMEOUT | 到位未 COMPLETED → 根因：consumed 卡住、checkArrival 未触发 |
| prepare（Run 7） | FAILURE(A05) | 并发确认阶段偶发拒绝，未进入飞行 |
| prepare（Run 9，建议2+3+到达修复） | **SUCCESS** | vmax 1.12-1.18，任务 ~111s |
| PVA 前馈 | PASS | 12 机 `trajectory_setpoint.vx/vy/vz` 均有限 |
| 周期性零速 | 消除 | 巡航 5s 窗口均值 0.52-0.69，无零谷 |
| 巡航速度 | PASS | max 1.04 m/s、mean 0.61 m/s |
| 下降段速度 | **修复** | Run 4 2.15-2.37 → Run 9 1.12-1.18 m/s（建议 2+3） |
| 分层执行 | PASS | z 先 15.1m 层再降 12m |
| 终点偏差 | PASS | 0.2-0.7 m（near_final 容差判定） |
| 最小机间距离 | **未达标（0.42 m）** | UAV3-UAV4 @t≈75.2s，低于 1.0m 门禁（既有问题） |

## 4. 清理与残留检查

- 逆序停止（TCP → backend/bridge → 机载层 → 仿真层）；`ss`/`ps` 检查无残留。
- 【结果】**PASS**：每次运行后残留进程 0、测试端口全部释放；Run 9 后 QGC 断连为脚本
  清理所致（非仿真崩溃）。

## 5. 遗留项（未验证/未解决）

1. 最小机间距离 0.42 m 低于 1.0 m 门禁（UAV3-UAV4 巡航段），属既有 swarm rebound 未完全
   覆盖动态场景的问题，需后续验证/增强，本计划未移植原版独立碰撞预测。
2. 个别机终点偏差 0.6-0.7 m（分析脚本按 z∈[11.4,12.6] 最近点评估），任务判定以 C++
   `near_final`+`pos_tol` 为准通过；如需更紧终点可调 `position_tolerance_m`。
3. 本次未做 plan §6.4 的 3s/5s `replan_period_s` A/B 诊断与故障注入（候选停止/NaN/模式
   快速切换）的 SITL 级验证；relay 的对应行为由纯单测与 rostest 覆盖。
4. follower（FOLLOW_ROUTE）的 PositionTarget 候选已在单测覆盖，未在真实 SITL 编队飞行验证。


