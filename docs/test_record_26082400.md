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
  cd /home/ub20tg/catkin_swarm6-2
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
  cd /home/ub20tg/catkin_swarm6-2
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
  cd /home/ub20tg/catkin_swarm6-2
  source /opt/ros/noetic/setup.bash
  catkin build
  catkin_test_results --all build
  ```
- 【结果】**PASS**：`catkin_test_results --all build` **150 tests, 0 errors, 0 failures**
  （基线 133 + 新增 17）。

## 2. 端到端 prepare 复测（无头 15 机）

> 启动步骤复用 `docs/test_record_26082300.md` §1（仿真层/机载层/GCS/标定/TCP/起飞）。
> 无头测试前清空 `.ros_home`（保留文件夹）。

### 2.1 环境与起飞

- 仿真层：`multi_uav_ego_15sim.launch gui:=false`（11300/11345）。
- 机载层：`startup_offboard_ego.sh 1..15`（11311..11325，日志改到 `.tmp/`）。
- GCS_A：bridge + backend + TCP（11310 / 39001），标定 ready 15/15。
- 起飞：`offboard_takeoff_15.py` 15/15 `armed=True / mode=OFFBOARD`，软起飞 z≈5m。
- 【结果】**PASS**（两次运行 15/15 起飞成功；`/mavros/setpoint_raw/local` 稳定 30.009 Hz，
  唯一发布者为 relay）。

### 2.2 prepare（A01-A12 `MOVE_TO`，timeout 180s）

- 步骤：
  ```bash
  cd /home/ub20tg/catkin_swarm6-2
  source /opt/ros/noetic/setup.bash && source devel/setup.bash
  python3 references/runtime/task_test.py --backend tcp --group A --task prepare \
    --tcp-host 127.0.0.1 --tcp-port 39001 --timeout-s 180
  ```
- 【预期】`status=SUCCESS`、`failed_robot_ids=[]`；12 机到达目标（终点偏差 ≤0.6 m 容差）；
  分层执行（15m 层→水平→12m）；EGO ULog `trajectory_setpoint.vx/vy/vz` 为有限值；
  巡航速度不再出现与 replan 周期锁定的周期性零速。
- 【结果】**`status=SUCCESS`、`failed_robot_ids=[]`，两次运行均通过**（Run 3 无 clamp、
  Run 4 带 start 导数 clamp）：
  - 任务耗时 ~133 s（00:35:51→00:38:04 / 00:46:28→00:48:41）；
  - 12 机全部 `COMPLETED`，终点偏差 0.1-0.6 m（≤0.6 m 容差）；
  - 分层执行：z 先爬 15.1m 层 → 水平巡航 → 垂直降 12m；
  - 巡航段水平速度 max 1.05 m/s、mean 0.59 m/s，5s 窗口均值 0.48-0.70 m/s，
    **不再出现修复前的周期性降零**（修复前 ~0.2 m/s 谷值）；
  - PX4 ULog `trajectory_setpoint.vx/vy/vz` 有限样本 859-970/1198-1216（修复前 0/750）；
  - Run 3 曾发现下降段 setpoint 速度尖峰 15.7 m/s、z 过冲至 +21 m（数值微分导数病态），
    已加 start_vel/start_acc 1.5× 界限 clamp（`clampStartDerivatives`），
    Run 4 尖峰收敛至 setpoint vmax ≤2.41 m/s、acc_z_max ≤8.35 m/s²。

### 2.3 ULog 分析

- 脚本：`.tmp/analyze_ulog/analyze_prepare_trajectory.py`、`plot_uav_realtime.py`
  （p-t 三轴合并一张 + 新增三轴 v-t 一张 + speed）。
- 检查项：
  1. `trajectory_setpoint` 的 vx/vy/vz 有限样本占比（Run 4：859-970/1198-1216，修复前 0/750）；
  2. 实际水平速度谱：巡航段无 ~4 s 周期性零速谷值（5s 窗口均值最低 0.48 m/s）；
  3. 最小机间距离、终点偏差、最大速度（见 §3）。
- 【结果】见 §3 汇总与图：
  - `.tmp/analyze_ulog/prepare_trajectory_review.png`（3D 轨迹/速度/机间距/高度）
  - `.tmp/analyze_ulog/uav1_realtime.png`、`uav6_realtime.png`（3D + 三轴 p-t 合并 +
    三轴 v-t + speed）

## 3. 结果汇总

| 项目 | 结果 | 备注 |
|---|---|---|
| 单元 1.1-1.3 | PASS | ego 52、executor 75、全量 150 tests 0 failure |
| prepare（Run 3，无 clamp） | SUCCESS | setpoint 速度尖峰 15.7 m/s → 定位为导数病态 |
| prepare（Run 4，带 clamp） | SUCCESS | 尖峰收敛至 setpoint vmax ≤2.41、acc ≤8.35 |
| PVA 前馈 | PASS | 12 机 `trajectory_setpoint.vx/vy/vz` 均有限 |
| 周期性零速 | 消除 | 巡航 5s 窗口均值 0.48-0.70，无零谷 |
| 巡航速度 | PASS | max 1.05 m/s、mean 0.59 m/s |
| 分层执行 | PASS | z 先 15.1m 层再降 12m |
| 终点偏差 | PASS | 最大 0.6 m（≤容差） |
| 实际速度峰值 | **超限（2.15-2.37 m/s）** | 出现在 15→12m 下降/拐角过渡，setpoint ≤2.41；需后续调平 |
| 最小机间距离 | **未达标（0.35 m）** | UAV2-UAV4 @t≈47.8s，低于 1.0m 门禁（既有问题，Run 3 为 0.49m） |

## 4. 清理与残留检查

- 逆序停止（TCP → backend/bridge → 机载层 → 仿真层）；`ss`/`ps` 检查无残留。
- 【结果】**PASS**：`prepare 退出码=0` 后清理，残留进程 0、测试端口全部释放。

## 5. 遗留项（未验证/未解决）

1. 下降/拐角过渡段实际速度峰值 2.2-2.4 m/s 仍超 `max_vel=1.0`（setpoint 已 clamp 到 ≤2.41，
   不再出现 15 m/s 级病态）；根因是水平→垂直过渡 B-spline 的加速度/速度超限，需后续在
   计划层约束或 PX4 侧速度限制。
2. 最小机间距离 0.35 m 低于 1.0 m 门禁（UAV2-UAV4 爬升阶段），属既有 swarm rebound 未完全
   覆盖动态场景的问题，需后续验证/增强，本计划未移植原版独立碰撞预测。
3. 本次未做 plan §6.4 的 3s/5s `replan_period_s` A/B 诊断与故障注入（候选停止/NaN/模式
   快速切换）的 SITL 级验证；relay 的对应行为由纯单测与 rostest 覆盖。
4. follower（FOLLOW_ROUTE）的 PositionTarget 候选已在单测覆盖，未在真实 SITL 编队飞行验证。

