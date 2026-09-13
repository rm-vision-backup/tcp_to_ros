# 实施计划：`ego_planner_driver` 基于起始位姿反馈的 5m 闭环软起飞

> **状态：暂不执行（2026-09-13 评估）** —— 2026-09-11 UAV6 实机起飞 setpoint/thrust 剧烈震荡的真因
> 已定位并解决：开机自启动程序 `offb_hover.py`（机上 `~/catkin_ws/src/formation_follow_control/scripts/`）
> 与我方 `setpoint_relay` **同时发布 `/mavros/setpoint_raw/local`**，PX4 被两路互不相容的位置目标
> 拉扯；清除外部发布者后同一条软起飞指令链路实测正常（planner 报
> `TAKEOFF started from z=3.08, target z=8.08`，PX4 `log_143` 实际 3.07 → 8.71 m 并在 8.08 m 悬停
> 约 40 s）。本文指出的"TAKEOFF 完成判据为时间/指令开环"问题成立，但**不是**该次异常的主因，
> 故暂不实施；§2.1 的无桨证据作为复活依据保留。**状态与证据的唯一权威记录见
> `README.md` §状态-26091316 与 `../../../.tmp/notes/flight_260911_log137_setpoint_fight_260913.md`；
> 下文为可复查的备选方案，未实施，勿据本文改动源码。**

## 1. 目标

修正 `ego_planner_driver` 当前 TAKEOFF 按时间开环推进、按指令高度判完成的问题，使软起飞满足：

1. 以进入 TAKEOFF 后首次可用的本机 `local_pose` 为起点；
2. 锁存起始 `x/y/z/yaw`，目标高度固定为 `start_z + takeoff_height_m`；
3. 保留限速、连续的上升位置参考，由 PX4 位置控制器完成内环跟踪；
4. 只依据无人机实测位置判断是否实际完成上升；
5. 到达目标并连续稳定后切换 HOLD，悬停目标保持起始 `x/y/yaw` 和目标 z。

本计划不在 planner 内新增 PID。PX4/MAVROS 继续负责位置闭环；planner 只生成平滑位置 setpoint、读取反馈并判定收敛。

## 2. 当前实现事实与问题

### 2.1 当前 TAKEOFF 是时间开环

**文件：**

- `src/ego_planner_driver/src/ego_planner_driver_node.cpp`
- `src/ego_planner_driver/include/ego_planner_driver/ego_planner_driver_node.h`

当前 `publishSetpoint()` 的 TAKEOFF 分支使用：

```text
current_z = takeoff_start_z + min(elapsed * takeoff_ascend_speed,
                                  takeoff_height)
```

并以：

```text
current_z >= target_z - 0.1
```

宣布 TAKEOFF 完成。这里的 `current_z` 是 planner 生成的指令高度，不是 `/local_pose` 的实测高度。

结果：只要墙钟时间达到理论爬升时间，即使实机未起飞、跟踪滞后或实际高度未到 5m，状态也会切换到 HOLD。

上桨前的不上桨测试已复现该问题（2026-09-13，UAV6）：

- ROS 侧 `uav_offboard_ego_real.launch` 运行（run `429fc230-…`）：`TAKEOFF started from z=3.91` → `4.900 s` 后 `TAKEOFF complete at z=8.91`，第二次同样 `4.934 s` 后 `complete at z=8.86`；
- PX4 侧 4 条 ulg（日志 id 138–141，14:15:02/10/28/56）：`vehicle_local_position.z` 全程只在 **4.4–12.9 cm** 内抖动，`vz` 绝对值 ≤ `0.072 m/s`，期间 `nav_state` 为 2(POSCTL)/14(OFFBOARD)；
- 即实测从未离地，而 planner 已按 `4.9 s × 1.0 m/s ≥ 5.0 − 0.1` 判为完成。

（证据文件：`~/Downloads/flight_logs/UAV6_2026-09-12_1407/` 与 UAV6 `~/catkin_swarm6-2/.ros_home/log/429fc230-…/`。）

### 2.2 横向目标未锁存

当前 TAKEOFF 每个 setpoint 周期重新从 `local_pose` 读取 x/y/yaw。无人机若横向漂移，目标会跟随漂移，不能约束其回到起始水平位置，也不能保证完成后在起飞点上方悬停。

### 2.3 现有测试未覆盖反馈闭环

`test/test_ego_state_machine.cpp` 现有用例只覆盖：

- `ArmOffboardEdgeStartsTakeoff`：arm+OFFBOARD 上升沿进入 TAKEOFF、下降沿回 HOLD；
- `TakeoffPublishesHoldSetpoint`：起飞首帧 setpoint 的 z 等于起始 z；
- `RepeatedArmEdgesRemainHold`：重复 arm 边沿不会复用旧起飞。

尚未验证“理论时间已到但实测未到不得完成”“实测稳定到达后才进入 HOLD”，以及起飞期间的横/纵目标锁存。

## 3. 修改方案

### 3.1 起飞初始化时锁存起始位姿

首次执行 TAKEOFF setpoint 分支且 `takeoff_active_ == false` 时：

1. 使用已有 `local_pose_` 作为起飞反馈来源；没有本机 pose 时不初始化起飞，也不使用 `(0,0,0)` 作为替代起点；
2. 一次性锁存：

   ```text
   takeoff_start_x
   takeoff_start_y
   takeoff_start_z
   takeoff_start_yaw
   takeoff_target_z = takeoff_start_z + takeoff_height_m
   ```

3. 初始化 `takeoff_command_z = takeoff_start_z`；
4. 记录本次 setpoint 更新时间；
5. 清空起飞到达稳定窗口。

起飞激活后，不再随实测漂移更新 start x/y/yaw 或目标高度。

### 3.2 按控制周期生成平滑上升参考

保留现有匀速软起飞行为，但从“总 elapsed 直接计算”改为每个发布周期增量推进：

```text
dt = now - last_setpoint_time
command_z = min(command_z + takeoff_ascend_speed * dt,
                takeoff_target_z)
```

为避免实机跟踪落后时指令持续远离反馈，增加最大指令超前量：

```text
command_z = min(command_z,
                actual_z + takeoff_max_tracking_lead_m)
```

每周期发送：

```text
x   = takeoff_start_x
y   = takeoff_start_y
z   = command_z
yaw = takeoff_start_yaw
mask = kMaskHold
```

保持现有 `PositionTarget`、ENU 字段、`FRAME_LOCAL_NED` 与 `/setpoint/ego` 输出契约。

### 3.3 使用实测位置判断起飞完成

TAKEOFF 完成条件改为基于当前 `local_pose_`：

```text
vertical_error = abs(actual_z - takeoff_target_z)
horizontal_error = hypot(actual_x - takeoff_start_x,
                         actual_y - takeoff_start_y)
```

进入到达区要求：

```text
vertical_error <= takeoff_vertical_tolerance_m
horizontal_error <= takeoff_horizontal_tolerance_m
```

首次进入到达区时记录 `takeoff_arrival_since`；连续满足 `takeoff_stable_s` 后才完成。中途任一误差超限，清空稳定窗口并重新计时。

不得继续使用 command z、elapsed 或“实测 z 单边超过阈值”作为完成条件，避免飞机未跟上指令或瞬时超调时误报完成。

### 3.4 完成后锁存固定 HOLD 目标

反馈稳定到达后：

```text
steady_target = (
  takeoff_start_x,
  takeoff_start_y,
  takeoff_target_z,
  takeoff_start_yaw
)
```

随后：

1. `takeoff_active_ = false`；
2. 清空起飞稳定窗口；
3. 状态切换为 HOLD；
4. 后续继续通过现有 `steady_target_` 分支持续发布同一位置目标。

不在完成瞬间重新捕获实测 x/y 作为 HOLD 点，避免把起飞期间横向漂移固化为新悬停位置。

### 3.5 参数配置

**文件：**

- `src/ego_planner_driver/config/planner_defaults.yaml`
- `src/ego_planner_driver/src/ego_planner_driver_node.cpp`
- `src/ego_planner_driver/include/ego_planner_driver/ego_planner_driver_node.h`

保留：

```yaml
takeoff_height_m: 5.0
takeoff_ascend_speed: 1.0
```

新增建议默认值：

```yaml
takeoff_vertical_tolerance_m: 0.3
takeoff_horizontal_tolerance_m: 0.5
takeoff_stable_s: 1.0
takeoff_max_tracking_lead_m: 0.8
```

参数约束：

- `takeoff_height_m >= 0`；
- `takeoff_ascend_speed > 0`；
- 两个 tolerance 均 `> 0`；
- `takeoff_stable_s > 0`；
- `takeoff_max_tracking_lead_m > 0`；
- 所有 double 必须 finite，非法值启动时 fail-fast。

`takeoff_ascend_speed` 保留 legacy 参数名，避免无必要的配置迁移。

## 4. 状态重置边界

在以下既有状态转换点同步清理本轮起飞上下文，避免再次 arm/OFFBOARD 时复用旧起点或稳定窗口：

- arm+OFFBOARD 新上升沿进入 TAKEOFF；
- TAKEOFF 完成进入 HOLD；
- arm/OFFBOARD 下降沿退出 TAKEOFF；
- direct control 接管；
- 节点现有 run reset 路径。

只重置本计划新增的 takeoff 字段。

## 5. 测试计划

**主要文件：**

- `src/ego_planner_driver/test/test_ego_state_machine.cpp`（状态机与起飞用例）
- `src/swarm_uav_executor/test/test_ego_launch_config.py`（canonical 参数静态回归，见 §5.4）

### 5.1 起点和参考生成

1. 起始 pose 为 `(x=3, y=-2, z=2, yaw=0.4)` 时，目标为 `(3,-2,7,0.4)`，不是绝对 z=5；
2. 首个 setpoint z 等于起始 z；
3. 时钟按 30Hz 前进时，command z 单调、不超过 `start_z + 5m`；
4. 实测位置停滞时，command z 不超过 `actual_z + takeoff_max_tracking_lead_m`；
5. 起飞期间注入 x/y/yaw 漂移，输出仍保持锁存的起始 x/y/yaw。

### 5.2 反馈闭环完成判定

1. 时间推进超过理论 5 秒，但实测 z 仍为起始高度：状态必须保持 TAKEOFF；
2. command z 已到目标、实测 z 未进入容差：不得进入 HOLD；
3. 实测垂直误差满足但水平误差超限：不得进入 HOLD；
4. 实测进入垂直和水平容差、但稳定时间不足：保持 TAKEOFF；
5. 连续满足 `takeoff_stable_s`：进入 HOLD；
6. 稳定窗口中途越界：窗口清零，重新连续满足后才完成；
7. 瞬时越过目标高度但未稳定：不得误判完成。

### 5.3 HOLD 连续性和重复起飞

1. TAKEOFF 完成后的 `steady_target_` 等于锁存起始 x/y/yaw 与 `start_z + 5m`；
2. 切到 HOLD 前后 setpoint 无非预期位置跳变；
3. HOLD 阶段继续按现有频率输出 `kMaskHold`；
4. 完成或中断后再次出现 arm+OFFBOARD 上升沿，使用新的起始 pose，不复用旧起飞上下文。

### 5.4 参数测试

1. canonical YAML 包含全部新增参数；
2. 默认值与源码默认一致；
3. `swarm_uav_executor/test/test_ego_launch_config.py` 静态回归不回归（canonical YAML ↔ launch `rosparam load` 关系、planner 参数白名单），并按仓库惯例为新增 takeoff 参数补充 YAML 断言；
4. NaN、无穷、零或负 tolerance/stable/lead 参数按约束 fail-fast；
5. 原有 `takeoff_height_m`、`takeoff_ascend_speed` 接线无回归。

## 6. 验收标准

### 6.1 自动测试

- `ego_planner_driver` 起飞状态机新增测试全部通过；
- `swarm_uav_executor/test/test_ego_launch_config.py` 参数静态回归通过（含新增 takeoff 参数断言）；
- 工作空间现有测试无新增 failure。

### 6.2 SITL 行为

至少验证：

1. 非零起始 z 时实际上升量为 5m；
2. 将 `takeoff_ascend_speed` 配为明显高于 PX4 实际爬升能力（例如 5.0 m/s，理论时间约 1 s）后，planner 仍等待实测进入容差，不提前进入 HOLD；
3. 实测位置进入容差并稳定后才进入 HOLD；
4. 起飞期间 x/y 目标固定，完成后在起点上方目标高度悬停；
5. `/setpoint/ego` 与 relay 最终输出保持现有 30Hz、`kMaskHold` 和坐标契约。

### 6.3 实机（仅不上桨）

1. 不上桨 arm + OFFBOARD：确认目标 z 为 `start_z + takeoff_height_m`，状态只在实测进入垂直/水平容差并稳定后才切 HOLD；实测高度不动时必须**保持 TAKEOFF**（2026-09-13 UAV6 的不上桨测试即该场景）。
2. 上桨、限域试飞等现场门禁按现场安全评审执行，不属于本计划的实施与验收内容。

## 7. 不在本计划范围

- planner 内新增 PID、速度控制器或推力控制器；
- 修改 PX4/MAVROS 内环参数；
- TAKEOFF 位姿失联/超时的新安全策略；
- 将相对 local ENU 上升量改为 AGL、GPS 海拔或地形高度；
- 修改 `setpoint_relay` 仲裁架构或唯一输出者契约；
- 修改 arm、OFFBOARD、外部起飞脚本所有权；
- 修改普通任务规划、机间避碰、BRAKE_HOLD、executor、GCS/backend 或 ROS msg/srv。

## 8. 修改文件汇总

```text
src/ego_planner_driver/include/ego_planner_driver/ego_planner_driver_node.h
src/ego_planner_driver/src/ego_planner_driver_node.cpp
src/ego_planner_driver/config/planner_defaults.yaml
src/ego_planner_driver/test/test_ego_state_machine.cpp
src/ego_planner_driver/README.md                        # 状态机/TAKEOFF 语义的单一权威，判据变更必须同步
src/swarm_uav_executor/test/test_ego_launch_config.py    # canonical 参数静态回归，按惯例补新参数断言
src/tcp_to_ros/plans/README.md                           # 实施后把该计划状态由“未实施”更新为已实施
src/tcp_to_ros/test_records/test_record_yymmddhh.md      # 实施后的测试记录（仓库惯例）
```

**不需要修改**：`src/ego_planner_driver/launch/ego_planner_driver.launch` 与
`src/swarm_uav_executor/launch/uav_executor_ego.launch` —— 两者均以
`rosparam command="load"` 整文件加载 `config/planner_defaults.yaml`，从不逐条枚举 planner 参数
（该约定由 `swarm_uav_executor/test/test_ego_launch_config.py` 静态回归守护）。

实施应保持外科式修改：优先复用 `local_pose_`、`steady_target_`、`kMaskHold`、现有参数读取和状态发布机制，不重构相邻模块。
