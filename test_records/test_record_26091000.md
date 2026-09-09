# 测试记录 26091000：direct BRAKE_HOLD 实施后干净 stack 5 轮 prepare 复测（4/5 SUCCESS）

时间：2026-09-10 00:09–00:33 CST（5 轮矩阵 00:09–00:29；R4 端口强清补测 00:30–00:33）
记录人：cline-deepseek
对应基线：`implementation_plan_26090900.md`（direct BRAKE_HOLD，已实施；首验见 `test_record_26090919.md`）
工作空间：`/home/ub20tg/catkin_swarm6-2`

## 0. 摘要

- **背景**：上一会话（26090919）在 direct BRAKE_HOLD 实施后 4 轮 SITL 0 SUCCESS（3 次真实
  MIN_DISTANCE_BREACH → 组级 fan-out/10 机直锁已实测，但无完整 SUCCESS 轮）。本次删除旧留档
  `.ros_home_round1..4`（约 1.36GB）、清空默认 `.ros_home`，用 v2 runner 5 轮**干净 stack 重跑**，
  验证同参数组合下能否回到历史 SUCCESS 基线。
- **结果**：**matrix 4/5 SUCCESS**（R1/R2/R3/R5 SUCCESS，failed=[]，ticks 1608–1649 ≈ 80–82s，
  正常区间、无排程滑移；R4 第 1 次 infra 失败）。与历史基线（26090722 / 26090816 / 26090902 均
  4/5）持平。
- **R4 第 1 次失败 = infra，非被测行为**：已知 `tcp_server(39001)` 残留 bug——新 server bind 失败
  （当时 `05_tcp.log` 报 `Address already in use`），listening 就绪误判由**旧 server** 满足 → 任务
  连旧 server/已死 backend，整组 A01..A12 失败、ticks=2。非代码回归。
- **R4 补测（端口强清后）失败 = 真实物理事件**：A01 `MIN_DISTANCE_BREACH`（UAV1-UAV2 同帧 ULog
  实测 3D **0.786m**/水平 0.777m @goal+48.9s，<1m 持续 2.45s）→ 后端组级 hold 触发
  （`targets=A03..A12`，排 A01 触发者/A02 已 terminal）→ **10 机 `direct BRAKE_HOLD locked`
  端到端再次实测正常**（同 26090919 验证路径）；任务层 FAILURE，属既有 MIN_DISTANCE 问题族。

## 1. 环境与配置

| 项 | 值 |
|---|---|
| 仿真 | PX4 SITL 15 机无头（`gui:=false`，master 11300 / gazebo 11345） |
| 驱动 | `src/swarm_simulation/bashs/round_prepare_15sitl.sh --round N --task prepare --timeout-s 150 --keep-ros-home-on-fail 1`（每轮 clean→sim→onboard 15/15→gcs bridge/backend→标定→tcp→takeoff 15/15→prepare→cleanup） |
| 编排 | v2 runner `.tmp/scripts/run_5_prepare_matrix_260909b.sh`（每轮前 `pkill tcp_server_node.py` + 残留/端口断言 + 30s 静默；单轮日志 `.tmp/logs/roundN_prepare.log`，汇总 `.tmp/logs/matrix5_impl_260909.log`） |
| 配置 | `horizontal_speed_mps=0.7`（`a2d0544` 恢复）、`schedule_geometric_distance_m=1.3`（`e96127f`）、`schedule_safety_margin_m=1.0`；Safety active |
| 参与 | A01–A12 参与，A13–A15 reserve |
| 代码基线 | ego_planner_driver `c772e96`（direct BRAKE_HOLD，含 `ae4654d` SLOW 1.5 档）；swarm_uav_executor `a5e4a02`；tcp_to_ros `ce16160`（含 `29d779b`/`9894052`） |

> runner 头部注释残留“e96127f(1.3m/0.6mps)”为 260909b 旧文案；本次实际配置为
> `a2d0544` 改回后的 **v=0.7 m/s**（`.tmp/plots/r4_260910/latest_flight_metrics.json`
> `model_horizontal_speed_mps=0.7` 佐证），与 26090919 基线同参数。

## 2. 结果汇总

| 轮 | 时间 | 结果 | failed | ticks(≈s) | 详情 |
|---|---|---|---|---|---|
| R1 | 00:09–00:13 | SUCCESS | [] | 1649(≈82s) | 无事件，12/12 COMPLETED |
| R2 | 00:13–00:17 | SUCCESS | [] | 1638 | 无事件 |
| R3 | 00:18–00:21 | SUCCESS | [] | 1637 | 无事件 |
| R4(1) | 00:22–00:24 | FAILURE（infra） | A01..A12 | 2(0.1s) | tcp_server 39001 残留：旧 server 仍占监听、新 server bind 失败；任务未起飞即整组失败 |
| R5 | 00:25–00:29 | SUCCESS | [] | 1608 | 无事件 |
| **matrix 汇总** | 00:09–00:29 | **4/5 SUCCESS** | — | 1608–1649 | 成功轮 ticks 正常区间，无排程滑移 |
| R4 补测 | 00:30–00:33 | FAILURE（真实事件） | A01,A02 | 1080(≈54s) | A01 `MIN_DISTANCE_BREACH` → 组级 fan-out A03..A12 → 10 机 direct BRAKE_HOLD locked |

证据（本轮日志）：
- `.tmp/logs/matrix5_impl_260909.log`：5 轮 start/end + exit code + 汇总（round1..3/5 `ROUND RESULT: SUCCESS`、round4 `FAILURE/UNKNOWN`）。
- `.tmp/logs/round{1..5}_prepare.log`：每轮 7 阶段日志与 `status=... failed=... ticks=...`。
- `.tmp/logs/round4_retry_260910.log`：R4 补测全程。

## 3. R4 失败分析

### 3.1 第 1 次：tcp_server 39001 残留（infra，非回归）

- round4_prepare.log 阶段 [5]：`TCP launched pid=1375102 (39001)` 后首个 3s 轮询即
  `tcp listening=1`（00:23:56），而 R1/R2/R3/R5 同阶段均为首轮 `listening=0`、次轮才 `1`——
  即 R4 开始时 39001 已有**旧监听者**存在，就绪探针被旧 server 满足；新 server bind 失败
  （当时 `05_tcp.log`：`Address already in use`，现该轮 `.ros_home_round4` 已被补测覆盖，原
  05_tcp.log 未留存）。
- 结果：`status=FAILURE failed=[A01..A12] ticks=2`（任务连旧 server/已死 backend，未起飞即
  失败）；随后补测前强杀残留并断言端口 clean。
- 判定：**非被测行为**。根因是 cleanup.sh/round runner 对 `tcp_server_node.py` 的残留清理缺口
  （既有开放项，见 §5）。

### 3.2 补测：A01 MIN_DISTANCE_BREACH → 组级直锁（真实物理事件）

- 后端（`.ros_home_round4/log/<run>/gcs_a_backend_node-1.log`）：
  `00:32:15,270 GCS_A goal joint-mission-002/... uavs=A01..A12` →
  `00:33:07,828 GCS_A group safety hold triggered by A01 (MIN_DISTANCE_BREACH); targets=A03,A04,...,A12`
  （goal 后 **+52.6s**，恰一行、fan-out 一次性）。
- 机载 C++（`.ros_home_round4/log/<uuid>/rosout.log`）：UAV3..UAV12 共 **10 机**
  `[UAVn] direct BRAKE_HOLD locked at (x,y,z)`（epoch 1788971587.837–1788971588.852，
  fan-out 后约 1s 内全部锁存；位置为本机当前位置）。UAV1/UAV2（触发对/已 terminal）不在集合；
  reserve A13–A15 无任何 /brake_hold/uav_hold。
- 物理量（ULog 同帧实测，`.tmp/plots/r4_260910/latest_flight_metrics.json`）：
  UAV1-UAV2 最小 3D 距 **0.7855m**、水平 **0.7773m** @1788971584.135（goal+48.9s）；
  <1.0m 持续 2.45s、<1.1m 2.55s、<1.35m 3.05s；`actual_horizontal_runtime_1m_breached=true`。
  UAV2 机载在 +45.5s/+51.7s 两度 `yield committed (peer=UAV1)`（SLOW 让路），仍进入 UAV1-UAV2
  走廊汇合破距——同 26090919/26090902 R2 事件族（走廊汇合、物理收敛 <1m）。
- 判定：executor 实时 3D 兜底门禁合理触发；组级 fan-out + 机载 direct BRAKE_HOLD 在干净 stack
  上**再次端到端正常**。任务层 SUCCESS 未达成。

## 4. 收尾状态

- 残留进程 0；测试端口 39001/11300/11310/11345 clean（复检通过）。
- 默认 `.ros_home` 已清空（8KB，仅留空 log 目录）。
- 失败轮留档：`.ros_home_round4`（**384M**，含 PX4 ulog/机载 rosout/GCS 日志/07_task JSON；
  来自 **R4 补测**，覆盖了 matrix R4 第 1 次失败留档 239M）。
- 绘图证据：`.tmp/plots/r4_260910/`（`latest_flight_overview.png`、`uav1_uav2_focus.png`、
  `latest_flight_metrics.json`；goal-epoch 1788971535.270872，ticks 1080，heading 缺省 -0.006）。
- 全程日志：`.tmp/logs/matrix5_impl_260909.log`、`.tmp/logs/round{1..5}_prepare.log`、
  `.tmp/logs/round4_retry_260910.log`。

## 5. 结论与建议

- **结论**：direct BRAKE_HOLD 实施后干净 stack 5 轮复测 **4/5 SUCCESS**，与历史基线
  （26090722/26090816/26090902 均 4/5）持平。成功轮无排程滑移、无提前锁存；R4 第 1 次失败为
  已知 infra 残留 bug（非代码回归），补测失败为真实 MIN_DISTANCE 事件（同既有问题族）。
  任务级 4/5 不能视为 100% 通过；剩余 FAIL 不为 direct BRAKE_HOLD 实现引入。
- **开放项（已知）**：`src/swarm_simulation/bashs/cleanup.sh` / round runner 对
  `tcp_server_node.py`(39001) 的残留清理存在缺口（本轮 R4 第 1 次失败暴露）——建议补上按
  cmdline 匹配 tcp_server 的兜底 kill 与启动前端口占用断言。
- **后续建议**：若需 5/5，先修上述残留清理项，再单独重跑 R4 若干轮；MIN_DISTANCE 走廊汇合族
  事件（UAV1/2、UAV6-9 族）建议按 geo/margin 组合继续多轮对照。

## 6. 关联

- 历史对照：`test_record_26090722.md`（margin=1.0 4/5）、`test_record_26090814.md`（1/5）、
  `test_record_26090816.md`（4/5）、`test_record_26090902.md`（v0.6/geo1.3 4/5）、
  `test_record_26090919.md`（direct BRAKE_HOLD 实施，4 轮 0 SUCCESS）。
- 代码基线：ego `c772e96`、executor `a5e4a02`、tcp `ce16160`；config `a2d0544`(v=0.7) +
  `e96127f`(geo1.3) + margin=1.0。

