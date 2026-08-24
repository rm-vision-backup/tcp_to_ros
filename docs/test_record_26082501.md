# 预测式避碰增强实施与测试记录

时间：2026-08-25 01:xx CST（持续更新）  
对应计划：`plans/implementation_plan_26082500.md`  
工作空间：`/home/ub20tg/catkin_swarm6-2`

## 0. 记录原则

- 本文件只记录本计划实施后实际执行的构建、单测、rostest、闭环和 SITL 证据；不回填旧记录。
- 临时脚本/输出仅放工作区 `.tmp/`；ROS 运行数据放 `.ros_home/`；诊断日志放
  `runtime_logs/ego_planner/`。
- 无头/SITL 前按需清理 `.ros_home`，但不删除历史证据目录；测试结束 HOLD 测试机并检查残留。
- 阶段 A 仅 shadow。任何 warning/emergency 预测均不得改变 setpoint、intent、轨迹或状态。

## 1. 实施前基线

| 仓库 | commit |
|---|---|
| `ego_planner_driver` | `d248525` |
| `swarm_uav_executor` | `63c6278` |
| `safe_valley_exp` | `d3307db` |
| `swarm_uav_interfaces` | `c5b3aca` |
| `tcp_to_ros`（计划提交后） | `69530d7` |

实施前 `catkin_test_results --all build`：**186 tests，0 errors，0 failures**；其中
`ego_planner_driver` 78 tests。

## 2. 阶段 A：预测、身份、episode 与异步日志

### 2.1 实施内容

- 新增 `SafetyPredictor`：共同绝对时间窗采样、closing speed、CPA/TTC、数据年龄、制动余量和
  warning/emergency 动态门槛；共同预测窗不足输出 `INSUFFICIENT_DATA`。
- 新增严格末尾数字 ID 解析；低 ID=`STAND_ON`、高 ID=`YIELD`、非法/重复=`UNKNOWN`。
- 新增 `ConflictEpisode`：冲突 peer/role 锁存，持续安全达到迟滞时间后释放。
- 新增 `SafetyLogger`：truncate 覆盖、合法文件名、有界队列、单写线程、退出 drain；timer 不同步写盘。
- 节点新增 25Hz safety timer。生产 launch 默认 `shadow`；库内默认 `off`。shadow 只更新诊断快照和
  日志；误配 `active` 显式告警并降级 shadow。
- `replanLoop()` 已只调用 `replanOnce()`，候选生成/提交不存在第二套业务路径，未新增无意义事务类。

### 2.2 构建与测试

执行：

```bash
cd /home/ub20tg/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin build ego_planner_driver swarm_uav_executor safe_valley_exp
catkin test ego_planner_driver
catkin_test_results build/ego_planner_driver
python3 src/swarm_uav_executor/test/test_ego_launch_config.py -v
```

结果：

- 构建：5 个依赖/目标包成功，0 warning，0 failure；
- `ego_planner_driver`：**96 tests，0 errors，0 failures**（原 78 + 新 18）；
- launch 静态回归：3/3 PASS；三个相关 launch 均通过 XML 检查和 `roslaunch --files` 解析；
- 新覆盖：相向/正交/背离、不同激活 epoch、CPA/TTC、年龄门槛单调性、ID role、episode 释放、
  logger 覆盖/drain/队列满，以及 shadow tick 前后 trajectory/activation/state/setpoint 不变。

日志：`.tmp/logs/stage_a_build.log`、`.tmp/logs/stage_a_test_ego.log`、
`.tmp/logs/stage_a_test_launch_config.log`。

随后执行计划指定全量回归：6 包构建成功、0 warning；`catkin test ego_planner_driver
swarm_uav_executor tcp_to_ros` 全部成功；`catkin_test_results --all build` 为 **205 tests，0 errors，
0 failures**（ego 96、executor 82、tcp 27）。测试阶段的 catkin warnings 为既有测试 stderr 汇总。

流程偏差：该次 rostest 未在命令行重复导出 `ROS_HOME`，rostest 在工作区外
`/home/ub20tg/.ros/log/` 生成日志。未擅自删除工作区外文件；后续测试命令强制导出工作区
`ROS_HOME`/`ROS_LOG_DIR`。

### 2.3 阶段结论

**自动测试 PASS；阶段 A 完成。** 尚未执行单机运行时 timer deadline 测量、双节点动力学、PX4 SITL
或 12 机完整链路，因此不声明飞行安全目标达成。阶段 B–E 继续保持待实施；旧 1m 执行层门禁保留。

## 3. 后续阶段记录

| 阶段 | 状态 | 证据 |
|---|---|---|
| B：受约束紧急制动 | 自动测试完成，SITL 待验收 | ego 104 tests；executor 专项 24 passed |
| C：tentative intent 屏障 | 待实施 | — |
| D：确定性四候选让路 | 待实施 | — |
| 双节点动力学闭环 | 待执行 | — |
| PX4 SITL 单/双/三机 | 待执行 | — |
| 12机 prepare 完整链路 | 待执行 | — |
| 旧 1m 门禁关闭 A/B | 禁止执行 | 前置证据未满足 |

## 4. ULog 图片

阶段 A 未运行 PX4 SITL，故未生成新 ULog，不覆盖 `.tmp/analyze_ulog` 现有图片。首次 SITL 后使用
工作区 `.tmp/analyze_ulog/` 脚本按公共 PX4 时间轴重绘，并在本节登记图片、ULog 和场景参数。

## 5. 阶段 B 自动测试

- `EmergencyBrakeGenerator` 覆盖 0/0.2/1.0/1.5m/s、不同方向、非零起始加速度、NaN 和超速拒绝；
- 生成轨迹严格检查 P/V/A 起终点、速度、加速度、jerk、finite 和时间单调；
- active 状态机回归覆盖 `EMERGENCY_BRAKE → BRAKE_HOLD`、终点锁存与 HOLD mask；
- executor 覆盖 braking 中间态不终止、`BRAKE_HOLD` 失败收口；
- `ego_planner_driver`：**104 tests，0 failure**；`test_ego_swarm_driver.py`：**24 passed**。
- 阶段 B 相关包全量：executor **83 tests**；`catkin_test_results --all build`：
  **214 tests，0 errors，0 failures**。

限制：未完成带 200–800ms intent 延迟、控制滞后和滑行的闭环；生产 launch 保持 shadow；旧 1m
执行层门禁保留。生成器异常 fallback HOLD 仅作为“不继续 nominal”底线，不算受约束制动成功证据。