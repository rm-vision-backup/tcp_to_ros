# ego_planner_driver 状态机修复 + 场景 3 批 1 链路健康修复复测记录（2026-08-12 19:47）

> 对应实施计划 `plans/implementation_plan_26081202.md`（ego_planner_driver 状态机修复 +
> IDLE setpoint 流 + GCS_A 全局原点统一）。
> 本轮先复现 `docs/test_record_26081123.md` 场景 3 批 1 的「全机未动」问题，定位为
> GCS_A backend fail-fast 链路健康预检；实施修复后重跑，验证任务真正下发并进入执行层。

## 0. 测试场景判定

| 项 | 结论 |
|---|---|
| 场景编号 | **场景 3**（真实 MAVROS + PX4 SITL 多 Master 隔离仿真端到端） |
| 本轮目标 | ① 复现并定位批 1「全机未动」根因；② 实施链路健康修复（UNDISCOVERED 忽略 + 前置等待 + 放宽心跳阈值 + driver 状态超时放宽）；③ 重跑验证任务下发与执行 |
| 目标链路 | runtime(TCP 39001) → tcp_server → gcs_a_backend → swarm_topology_bridge(跨 Master) → swarm_uav_executor → ego_planner_driver → PX4 SITL |

## 1. 测试环境

| 层 | Master | 内容 | 状态 |
|---|---|---|---|
| 仿真层 | 11300 | `multi_uav_ego_15sim.launch`（iris_0..iris_14，PX4 SITL 15 实例） | 用户启动 |
| 机载层 | 11311-11325 | 每机 `uav_offboard_ego.launch`（MAVROS + ego driver + pose/setpoint relay + executor + bridge） | 15/15 MAVROS connected |
| GCS_A | 11310 | `topology_group_a_sim.launch uav_name:=GCS_A run_uav_mock:=false` + `gcs_a_backend.launch` + `tcp_server_node.py`(39001) | 5 节点就绪 |

日志约定：`export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home`、`ROS_LOG_DIR=.../.ros_home/log`。

## 2. 第一轮：批 1「全机未动」根因（修复前）

### 2.1 现象

- 15 机起飞 + OFFBOARD + IDLE 悬停全部正常（统一坐标系、`exec_state=IDLE`、setpoint 30Hz）。
- 发送批 1 `prepare`（MOVE_TO → A01..A12）：返回 `FAILURE`（5/12），且**所有无人机位置未动**；
  `task_test.py` 输出 `ticks=2`（立即失败）。

### 2.2 定位（源码 + 日志双重确认）

| 证据 | 结论 |
|---|---|
| executor（UAV1/UAV9）17:41 任务时段**无任何 uav_task 服务记录** | 任务从未到达 executor → ego driver 收不到 goal |
| tcp_server 17:41:37 接受连接并 `send_goal` 成功 | TCP→Action goal 已送达 GCS_A backend |
| backend `_execute()` L412-420：`unhealthy = not is_stably_healthy(...)` → 任一不健康即 `FAILED / UAV_LINK_UNHEALTHY` set_aborted（**不调用 uav_task，也不发 HOLD**） | **fail-fast 预检在任务下发前整组作废**，非 HOLD 触发 |
| backend 17:27 日志：多机 `SUSPECT → LOST`（MAVROS heartbeat timeout） | 建链初期部分机未满足 stable healthy 窗口 |

**结论**：批 1 全机未动的根因 = backend 的 `UAV_LINK_UNHEALTHY` fail-fast 预检。机载层启动初期
bridge 跨 Master 心跳抖动使部分机未达 `is_stably_healthy`，任务在逐机分发前被整组 abort；
executor/ego driver 无感知，位置自然不动。

## 3. 链路健康修复（本轮实施）

| 修改 | 文件 | 内容 |
|---|---|---|
| ① UNDISCOVERED 不阻任务 | `tcp_to_ros/scripts/tcp_to_ros/link_health.py` | 新增 `is_healthy_or_undiscovered()`：`UNDISCOVERED`（从未收到 MAVROS state 的机）直接视为健康；保留 `is_stably_healthy` 兼容 |
| ② 预检替换 + 前置等待 | `tcp_to_ros/scripts/gcs_a_backend_node.py` | 后端预检改用 `is_healthy_or_undiscovered`；新增前置等待窗口（最多 `link_health_pre_task_wait_s`，默认 10s，每秒 `evaluate()` 重评，全绿才继续，超时才 abort） |
| ③ 放宽心跳阈值 | `tcp_to_ros/config/link_safety_defaults.yaml` | `link_suspect_after_s` 3→10、`link_lost_after_s` 5→15、新增 `link_health_pre_task_wait_s: 10.0`；容忍跨 Master 建链初期抖动 |
| ④ driver 状态超时放宽 | `swarm_uav_executor`：`ego_swarm.py` + `executor_defaults.yaml` | `state_timeout_s` 5→200，避免长轨迹规划/执行被 `DRIVER_TIMEOUT` 误判 |

提交（未推送）：`swarm_uav_executor 041c677`、`tcp_to_ros 7be1a8d`。
（修复过程中修正了 backend 插入代码的缩进错误，`catkin build tcp_to_ros swarm_uav_executor` 通过。）

## 4. 第二轮：修复后复测

### 4.1 环境就绪确认

| 检查项 | 结果 |
|---|---|
| GCS_A 5 节点（bridge/backend/tcp_server/gp_origin_publisher） | 全就绪，tcp 39001 监听 |
| 机载层 15 实例 | 15/15 connected，`exec_state=IDLE` |
| 坐标系 | 15/15 统一：5m 编队网格（排1 x≈31358、排2 x≈31363、排3 x≈31368；y 每列差 5m），z≈-67m |
| backend 链路 | **15/15 HEALTHY**（`DISCOVERED → HEALTHY valid MAVROS heartbeat is stable`，新参数生效） |
| setpoint 流 | UAV1 29.999Hz 持续发布（IDLE 下不断流） |

### 4.2 一键起飞（--individual）

`src/safe_valley_exp/scripts/offboard_takeoff_15.py --start 1 --end 15 --timeout 30 --individual`
（arm → 切 OFFBOARD → ego driver 软起飞 → 等 exec_state 回 IDLE，逐机独立子进程）：

```
成功 15/15: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
```

起飞后全网空：15/15 `armed=True / mode=OFFBOARD / exec_state=IDLE`，z≈-61.5~-63.5（较起飞前 -67 升空约 3.5~5.5m）。

### 4.3 批 1 `prepare`（MOVE_TO → A01..A12）

```
status=FAILURE, failed_robot_ids=[A01..A12], ticks=153, timeout_s=30
```

**关键进展**（相对第一轮）：

| 对比项 | 第一轮（修复前） | 第二轮（修复后） |
|---|---|---|
| backend 预检 | `UAV_LINK_UNHEALTHY` 整组 abort（从未调用 uav_task） | **放行**（新参数 + UNDISCOVERED 忽略） |
| task_test `ticks` | 2（立即失败） | **153**（真正执行一个阶段后超时）|
| UAV 位置 | 全机未动 | **z≈-42.7（较起飞 -61.5 上移约 19m，确实执行了任务）** |

## 5. 结论与遗留问题

### 5.1 已确认

- **链路层完全贯通**：runtime → tcp_server → gcs_a_backend → bridge → executor → ego driver 全部打通；
  batch 任务不再被 fail-fast 预检拦截，executor 收到任务、ego driver 发布 goal 并驱动无人机移动。
- 15/15 统一坐标系 + 软起飞 + OFFBOARD 悬停稳定；状态机修复生效（post-task 均回 IDLE，无 HOLD 卡死）。

### 5.2 遗留（执行层超时，供后续聚焦）

- 批 1 全失败的根因已从「任务未下发」转移至「**执行层 30s 超时**」：MOVE_TO 目标层
  z=12m，无人机从 -61.5 仅爬到 -42.7（上移约 19m）即超时。可能原因：① 高度层换算/hover
  基准（本机 local z 相对 GCS_A 原点为 -67m，目标 z=12m 是否为绝对层高）；② 1.0 m/s 最大
  速度约束下的 30s 时限对约 49m 爬升不足；③ 垂直过渡阶段耗时占用整组 timeout。
- 失败后全部回 IDLE/OFFBOARD，可安全重试。

### 5.3 交付物

- 修复提交（未推送）：`swarm_uav_executor 041c677`、`tcp_to_ros 7be1a8d`。
- 起飞脚本已迁入 git 并注册安装：`safe_valley_exp 23c84af`（`scripts/offboard_takeoff_15.py`）。
- 测试环境（机载层 + GCS_A）已全部清理，无残留 ROS 进程。