# min-snap 统一单段重构：集群 prepare 与 UAV1 单机长轨迹测试记录

时间：2026-08-24 20:xx CST（测试结束后）  
记录人：____________  
测试环境：PX4 SITL + Gazebo（无头）+ MAVROS + ego_planner_driver + swarm_uav_executor + swarm_topology_bridge + GCS_A + TCP runtime  
对应实施计划：`plans/implementation_plan_26082415.md`

## 0. 测试目标

验证 min-snap 统一单段重构（单段五次多项式、滚动窗口推进、执行层距离门禁）在
以下两种场景下的行为：

1. **集群 prepare（A01-A12 MOVE_TO）**：多机同时从 5m 软起飞高度出发爬升至 15m 层
   再降落至 12m 目标，机间距离门禁（水平 <1.0m 且垂直 <2.0m → 本地 HOLD）作为
   编队碰撞兜底是否按预期触发并整组收口；
2. **UAV1 单机长轨迹（FOLLOW_ROUTE）**：合并 coverage-segment-1/2 leader 航线的
   21 航点蛇形扫描（总长约 776m）能否稳定滚动推进、多次转弯、多航点消耗并保持
   平滑巡航。

## 1. 上次 UAV1-12 集群测试（无头 prepare，26082415）

### 1.1 环境与启动

- 编排脚本：`.tmp/scripts/run_headless_prepare_26082415.sh`（逐机独立启动 + 就绪补充轮）。
- 环境启动顺序：
  1. 清空 `.ros_home`（保留文件夹）；
  2. 仿真层：`roslaunch safe_valley_exp multi_uav_ego_15sim.launch gui:=false`
     （ROS master 11300 / Gazebo 11345，PX4 SITL 15 实例）；
  3. 机载层：`startup_offboard_ego.sh 1..15`（每机独立 master 11311..11325，
     MAVROS + ego_planner_driver + executor + swarm_bridge）；
  4. GCS_A：`topology_group_a_sim.launch` + `gcs_a_backend.launch` + `tcp_server_node.py`
     （master 11310 / TCP 39001），等待标定 ready（15/15 origin 冻结）。
- 起飞：`offboard_takeoff_15.py 1..15` → **15/15** `armed=True / mode=OFFBOARD`，
  软起飞至 z≈5m，等待 30s 稳定。
- 【结果】**PASS**：两次运行 15/15 起飞成功。

### 1.2 prepare（A01-A12 MOVE_TO，timeout 180s）

- 步骤：
  ```bash
  cd /home/yjq/catkin_swarm6-2
  source /opt/ros/noetic/setup.bash && source devel/setup.bash
  python3 references/runtime/task_test.py --backend tcp --group A --task prepare \
    --tcp-host 127.0.0.1 --tcp-port 39001 --timeout-s 180
  ```
- 任务内容：A01-A12 各自 `MOVE_TO`（3×4 点阵，x∈[12.5,27.5]、y∈[45,55]、z=12）。
- 时间线（第二次运行）：`17:52:52` 发送 → `17:53:25` 返回 **FAILURE**（约 33s）。

### 1.3 结果与分析

- 输出：
  ```json
  {"status": "FAILURE", "task": "prepare", "failed_robot_ids": ["A03", "A05"], ...}
  ```
  退出码 `1`。两次运行结果一致。
- 根因（**FAILURE by design**，非回归）：
  - 编队起飞/爬升阶段机间水平间距不足 **1.0 m** 门禁：UAV3-UAV5 最小 3D 间距
    **0.85 m**（水平 <1.0m 且垂直 <2.0m 同时满足）；
  - 执行层距离门禁（`_wait_for_terminal` 内 `_distance_safe`）触发 → 对应机本地
    `HOLD` → 任务以 `MIN_DISTANCE_BREACH` 收口 → `completion_policy=ALL` 整组失败，
    其余活动机安全 HOLD。
- ULog 分析（`.tmp/analyze_ulog/`，150s 窗口 + drift-based hover trim）：
  - `prepare_precrash_review.png` / `prepare_trajectory_review.png`：集群 3D 轨迹/速度/机间距；
  - `uav{1,3,6}_realtime.png`：分层飞行（15m 层）与速度 profile；
  - `uav6_neighbor_dist.png`：UAV6 与最近邻机距离时序，验证门禁判定。
- **结论**：执行层距离门禁按预期在真实近距编队场景下触发并整组安全收口；编队起飞/
  爬升段机间距设计（<1m）是触发根因，后续需放宽编队初始间距或分层错峰。

## 2. 本次 UAV1 单独测试（26082420）

### 2.1 环境说明（UAV1 机载层重启）

- 集群测试结束后环境保持运行；本次仅操作 UAV1（master 11311）。
- **环境改动**（长轨迹需要，默认行为不变）：
  - `src/swarm_uav_executor/launch/uav_executor_ego.launch`：新增 `ego_state_timeout_s`
    arg（默认 200.0）+ `ego_swarm/state_timeout_s` 透传；
  - `src/safe_valley_exp/launch/uav_offboard_ego.launch`：新增 `ego_state_timeout_s` arg 并透传；
  - UAV1 机载层以 `ego_state_timeout_s:=1200` 重启（其余 14 机参数不变）。
- 原因：executor `_wait_for_terminal` 在命令开始后 `state_timeout_s`（默认 200s）内
  无 terminal exec_state 即报 `DRIVER_TIMEOUT`；776m 航线巡航约 13-15 分钟，必须延长。

### 2.2 MOVE_TO 单机（early-HOLD 问题，未解决）

- 脚本：`.tmp/scripts/single_move_uav1.py`（目标：任务坐标 `(17.5, 50, 12)`，prepare 的 A01 目标）。
- 结果：`PREPARE accepted` → `START accepted` → `EXECUTING` → 爬升至 **15m 层** →
  **提前进入 HOLD**，未到达最终 12m 目标。
- 分析：
  - 非距离门禁触发（单机，垂直间距充足）；
  - 疑似状态机/末段 bookkeeping：中间 15m 层 waypoint 被误判为 final，或
    `waypoint_consumed_`/`near_final` 提前消耗视野末端；
  - 候选排查方向：`_build_layered_keypoints` 输出（start→15m→12m 目标）的末点判定、
    `replanLoop` final-segment 标志设置时机。
- 状态：**待修复**（本次长 FOLLOW_ROUTE 未受影响）。

### 2.3 FOLLOW_ROUTE 小矩形（COMPLETED）

- 脚本：`.tmp/scripts/single_follow_uav1.py`（12m 层折线）。
- 航线：`(17.5,50,12)→(22.5,50,12)→(22.5,45,12)→(17.5,45,12)`。
- 结果：**COMPLETED**，终点 `(17.49, 44.98, 11.96)`（偏差 ≤0.02m）。
- 说明：单段 min-snap + 滚动窗口在小范围折线上可正常完成。

### 2.4 FOLLOW_ROUTE 长轨迹（21 航点 ~776m，稳定执行）

- 脚本：`.tmp/scripts/single_follow_uav1_long.py`。
- 航线：合并 `references/runtime/examples/joint_mission/group_a/plans/plans.json` 中
  coverage-segment-1（y=12..75，11 航点）与 coverage-segment-2（y=75..138，11 航点）
  的 leader A01 航线，交界重复点合并 → **21 航点 S 形蛇形**（x 在 15/80 间摆动，z=12），
  总长约 **776m**，`timeout_s=1200`，携带原始 yaw。
- 结果：
  - `PREPARE accepted` → `START accepted` → `EXECUTING`；
  - 从当前位置爬升至 12m 层后开始蛇形扫描，稳定滚动推进至 **y=124 扫描线**
    （约 **17/21** 航点），耗时约 **5.5 分钟**；
  - 巡航速度稳定 ~1.0 m/s（`max_vel=1.0`）；转弯处轻微 overshoot
    （如 x 曾到 11.5，航线边界 15）后自动回到航线；
  - **无距离门禁触发**（单机，航线远离其余机）；
  - 用户叫停：`/UAV1/uav_hold` → `exec_state=HOLD`，稳定悬停于 `(34.5, 123.9, 11.98)`。
- 监控脚本 `.tmp/scripts/monitor_uav1_flight.py`（纯订阅，不干扰任务）全程记录
  exec_state 与 15s 间隔位置。
- **结论**：min-snap 单段重构在 776m 级长航线、多航点滚动推进、直角转弯场景下
  执行稳定，验证通过。

## 3. 结果汇总

| 项目 | 结果 | 备注 |
|---|---|---|
| 集群 prepare（A01-A12 MOVE_TO，26082415 无头） | **FAILURE（预期）** | 距离门禁触发：UAV3-UAV5 min 0.85m → `MIN_DISTANCE_BREACH` → 整组 ALL 失败，failed=[A03,A05] |
| 起飞（15/15，两次） | PASS | 15/15 arm+OFFBOARD，软起飞 z≈5m |
| UAV1 MOVE_TO 单机 | **HOLD early（问题）** | 爬至 15m 层后提前 HOLD，未达 12m 目标；状态机/末段 bookkeeping 待修 |
| UAV1 FOLLOW_ROUTE 小矩形 | COMPLETED | 终点偏差 ≤0.02m |
| UAV1 FOLLOW_ROUTE 长轨迹（21 点 776m） | **PASS（用户叫停）** | 稳定推进至 17/21 航点，巡航 ~1m/s，无距离门禁；HOLD 收尾悬停稳定 |
| 长轨迹状态超时 | 已解除 | `state_timeout_s` 默认 200s；UAV1 以 `ego_state_timeout_s:=1200` 重启 |

## 4. 清理与残留检查

- 逆序停止：TCP（tcp_server_node.py）→ GCS（gcs_a_backend/topology_group_a_sim）
  → 机载层（`startup_offboard_ego.sh stop 1..15`）→ 仿真层（multi_uav_ego_15sim）
  → PX4/gzserver/rosmaster；残留 swarm_bridge 进程单独清理。
- 【结果】**PASS**：全部测试进程 0；测试端口（39001、11300、11310..11325）全部释放。
- `.ros_home` 与 `.tmp/logs`、`.tmp/analyze_ulog` 保留（测试日志/ulog/图证据）。

## 5. 遗留项（未验证/未解决）

1. **UAV1 MOVE_TO early-HOLD 未解决**：需排查状态机/final-segment bookkeeping
   （中间 15m 层 waypoint 误判 final、`waypoint_consumed_`/`near_final` 提前消耗）。
2. **集群编队起飞/爬升机间距不足**：UAV3-UAV5 最小 0.85m 触发距离门禁属预期兜底，
   但编队初始间距/分层错峰需设计优化，避免正常任务被门禁打断。
3. **UAV1 机载层重启后 local z 偏移约 5.4m**：TAKEOFF 起点读取时序导致软起飞
   高度低于预期；不影响单机任务层（按 local 坐标执行），但需确认 EKF origin
   一致性，避免多机协同坐标失真。
4. **长轨迹仅单机验证**：编队跟随（FOLLOW_ROUTE `formation_follow`）与多机同时
   执行长航线的距离门禁联动未在 SITL 验证。

