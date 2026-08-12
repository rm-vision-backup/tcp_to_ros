# 冻结坐标、按需起飞与分层轨迹自动化验证记录

时间：2026-08-13 00:36 CST

## 验证范围

- A01-A12 全球位置中位数、圆周平均航向与 30° 门禁的纯函数测试。
- 两个固定编队的精确参与机集合与槽位测试。
- GCS_A 整批 PREPARE/START/ABORT、共享 deadline、聚合失败与 HOLD 测试。
- executor 任务登记、幂等、显式 START/ABORT、HOLD 和安全租约测试。
- EGO driver 的分层调度、followers PI、直接控制仲裁、规划结果 clearance 测试。
- ROS Service/Topic 集成 rostest。

## 执行结果

```text
catkin build --no-status
  6/6 packages succeeded

catkin test --no-status
  tcp_to_ros:          28 tests passed
  swarm_uav_executor:  75 tests passed
  ego_planner_driver:  30 tests passed

catkin_test_results --all build
  133 tests, 0 errors, 0 failures, 0 skipped
```

ROS 集成测试日志按 rostest 默认行为写入 `~/.ros/log`；用户已明确授权该目录读写。

## 尚未覆盖

自动化测试不等价于 15 机 PX4/Gazebo 动力学闭环或实机验证。交付后仍需按以下顺序验收：

1. 启动 A01-A15 机载 MAVROS、EGO、executor 和 topology bridge，保持 disarm。
2. 人工用 QGC 确认 PX4 正常及 RTK 已收敛。
3. 启动 GCS_A，确认 A01-A12 采样 2s、航向门禁通过，且 A01-A15 原点回读全部确认。
4. 再启动 TCP 上游，依次验证未 arm MOVE_TO 的 5m 检查点、15m 分层动作、两个固定编队、
   FAULT_EXIT 8m 离队和 timeout/HOLD 故障路径。
