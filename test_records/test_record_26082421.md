# EGO 轨迹 intent 时间契约与主动避碰审计记录

时间：2026-08-24 21:xx–23:25 CST
范围：`swarm_uav_interfaces`、`ego_planner_driver`、`swarm_uav_executor`、
`safe_valley_exp`、`tcp_to_ros`

## 1. 审计结论

- `UavTrajectoryIntent.stamp` 定义为轨迹实际激活 epoch；`t` 是相对时间，严格递增、`t[0]=0`，并以精确 duration 结束。
- intent 新增 `frame_id`，当前共享 ENU frame 固定为 `map`；生产者已设置，接收端校验协议、身份、frame、epoch、序列、数组长度、单调性与有限值。
- 规划快照后的优化耗时不能计入轨迹相对样本。候选仍在优化完成后激活，但邻机绝对 epoch 会按候选激活时刻重新基准；最多规划 3 次，并在提交锁内使用最新 intent 做最终 hard-clearance。
- rebound 硬推可生成 1m clearance 的理想正交交叉路径，但原实现峰值速度 6.04m/s、峰值加速度 22.41m/s²。修复后 rebound 输出强制时间重参数化，并按新时间轴重新检查 clearance；回归满足速度 ≤1.5m/s、加速度 ≤`max_acc*1.05`。
- 真实 peer min-snap nominal intent 的单边让路仍可能规划失败，多轮互惠收敛未完成。因此 `enable_rebound` 保持默认 false，执行层运行时距离门禁 HOLD 继续作为生产兜底。

## 2. 自动化验证

```bash
cd /home/yjq/catkin_swarm6-2
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin build
catkin test ego_planner_driver swarm_uav_executor tcp_to_ros
catkin_test_results --all build
```

结果：

- 6 个包构建全部成功；
- `ego_planner_driver`：78 tests，0 failure；
- `swarm_uav_executor`：81 tests，0 failure，含现有 2 个 rostest；生产 EGO launch 的
  `enable_rebound=false` 已经 roslaunch 展开并通过 runtime 参数断言；
- `tcp_to_ros`：27 tests，0 failure，含 TCP listener 早绑定及 Linux 端口策略回归；
- 全工作区：186 tests，0 error，0 failure；catkin 仅报告既有 warnings 汇总，无失败。

## 3. 新增/强化回归

- 同 epoch、同高度正交交叉：到达终点、全重叠时域 1m clearance、速度/加速度门禁；
- FakeClock：规划快照与 0.75s 优化耗时只移动激活 epoch，首帧仍为轨迹起点；
- intent wire/time contract：frame、epoch、版本重置、乱序、重复、非法时间、NaN、长度不匹配；
- 精确 duration 末样本，防止 intent 比实际执行时间提前结束；
- 真实 peer nominal intent 保护测试：明确阻止在互惠收敛未通过时恢复 rebound 默认值。
- launch 双层回归：静态 XML 测试检查参数透传；rostest 检查生产 EGO launch 展开后的 runtime
  参数仍为 `enable_rebound=false`。

## 4. 未完成验证

- 双节点、多轮、同步目标下的确定性互惠让路与无振荡收敛；
- 真实双机 PX4 SITL 交叉；
- 4 机与 15 机闭环六批次；
- Phase 6 实机不上桨/上桨门禁。

## 5. 15 机 PX4 SITL 无头 prepare（23:16–23:25 CST）

### 5.1 目的与证据

- 清空 `.ros_home` 后启动真实 15 机 PX4 SITL、15 套 onboard、GCS topology bridge/backend，
  验证业务 TCP `39001` 不会被 ROS XML-RPC 随机端口抢占，并执行一次 `prepare`。
- 证据目录：
  `/home/yjq/catkin_swarm6-2/.ros_home/log/headless_test/prepare_20260824_231608`。
- 关键证据：`result_summary.json`、`07_task_states.bag`、
  `10_UAV5_uav_task_state.csv`、`10_group_a_group_task_state.csv`、
  `10_a05_executor_full.txt`、`10_rosout_matching.txt`、
  `10_tcp_after_task.txt`、`12_cleanup_final_gate.txt`、`manifest.txt`。

### 5.2 启动与发送前门禁

- simulation：PX4 15/15、Iris model 15/15、simulator TCP 15/15；
- onboard：15/15 connected，所需节点齐全；
- 起飞：15/15 armed + OFFBOARD；
- 任务状态 rosbag 已启动并生成索引；
- GCS bridge/backend 启动后，TCP `39001` 始终由同一 `tcp_server_node`
  （PID `317095`）监听；仿真、onboard、GCS 与任务前后 owner 均未变化。

### 5.3 prepare 结果与失败原因

- 单次发送 A01–A12 `prepare`；command ID：
  `tcp-68981ad0ecf74c2cbc682b8bd63ba17c`。
- 组任务终态：`FAILURE`；失败机：A05（执行节点 UAV5）。
- ROS 任务状态定量证据：A05 报告 `detail_stage=HOLD`、
  `error_code=EGO_PLAN_FAILED`、`message=egoswarm planning/execution failure`；
  组状态随后以 A05 失败收敛。
- **QGC 操作员观察**：失败前 UAV5 与 UAV3 距离过近，画面上甚至疑似发生碰撞，随后触发
  HOLD。结合执行层“水平距离 <1.0m 且垂直距离 <2.0m”门禁，判定本次任务失败直接原因是
  A05–A03 近距触发 HOLD，而非 TCP 端口冲突。
- 证据边界：本轮 rosbag 未录 UAV3/UAV5 同步位姿或 UAV5 独立 ROS Master 的 planner `/rosout`，
  因此无法从已存证据计算最小中心距，也不能仅凭 QGC 画面确认物理碰撞或继续细分 planner 内部原因。

### 5.4 任务后状态与清理

- 任务后 15/15 仍 connected、armed、OFFBOARD；无掉线或坠机证据。
- TCP/bridge/backend 共存验证 **PASS**；prepare 业务任务 **FAILURE**，两者结论必须分开。
- 按精确 PID/PGID 清理本轮进程；相关进程和关键端口残留均为 0，
  `cleanup_final_gate=OK`。
- 本轮运行未修改源文件；`.ros_home` 内容均为本轮新生成日志。

### 5.5 后续根因采集

- 重跑时单独录制 UAV5 ROS Master 的 `/rosout`、EGO planner 状态、轨迹 intent、同步 UAV3/UAV5
  odometry 与执行层距离门禁日志；
- 对比本轮 `result_summary.json`、A05 executor 日志和 task-state rosbag，检查失败是否仍固定于
  A05–A03，或随调度/时序随机迁移；
- 主动互惠让路收敛完成前，`enable_rebound` 继续默认 false；生产安全仍由执行层 HOLD 兜底。