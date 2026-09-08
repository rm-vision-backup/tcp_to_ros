# 测试记录 26090902：prepare 五次干净重启复测（d=1.3m / v=0.6m/s / SLOW 1.5 档）

> 配置：无 runtime HOLD 注入。被测改动为三条排程/让路参数：
> `schedule_geometric_distance_m: 1.3`、`horizontal_speed_mps: 0.6`（commit `e96127f`，2026-09-08）
> 与 yield SLOW 档位新增 1.5 加速档（commit `ae4654d`）。本记录为**该配置下**的
> 五次干净重启 prepare 复测（对照：26090816 回退基线 4/5）。
> 驱动：`src/swarm_simulation/bashs/round_prepare_15sitl.sh` ＋
> `.tmp/scripts/run_5_prepare_matrix_260909b.sh`（v2：每轮前强杀 tcp_server 39001 残留，
> 轮间 30s 静默 + 残留进程清 0）。

## 1. 执行过程与编排缺陷修复（v1 污染轮已弃）

- 首轮（v1，`run_5_prepare_matrix_260909.sh`）出现 R2/R4 伪失败：**tcp_server(39001) 残留**——
  cleanup.sh 按 pid 文件杀 `rosrun` 独立进程会漏杀（bridge/backend 是 roslaunch 进程组故有效），
  旧 server 存活占 39001 → 下轮新 server `bind: Address already in use`（05_tcp.log traceback），
  listening 检测误通过（旧 server 在监听）→ task 连旧 server（其 backend 已死）→ 全组立即 FAILED
  （R2 ticks=2 / R4 ticks=101）。`SO_REUSEADDR=1` 已确认，非 TIME_WAIT。
- **修复**：v2 runner 每轮前/后 `pkill -TERM/-KILL -f tcp_server_node.py`；R2–R5 tcp bind 全部成功。
  v1 结果与留档（`.ros_home_round2/round4`）**已作废并删除**，以下统计仅含 v2 干净轮。

## 2. 结果汇总（v2，2026-09-09 01:32–01:52）

| 轮 | 起止 | 退出码 | 结果 | 详情 |
|---|---|---|---|---|
| 1 | 01:32:43–01:35:33 | 0 | **SUCCESS** | `failed=[]` ticks=1689（≈84s）12/12 COMPLETED |
| 2 | 01:37–01:39:08 | 1 | **FAIL** | `failed=[A08,A09]` ticks=613（≈31s，机载执行中途） |
| 3 | 01:40–01:43:28 | 0 | **SUCCESS** | `failed=[]` ticks=1617（≈81s）12/12 |
| 4 | 01:44–01:47:49 | 0 | **SUCCESS** | `failed=[]` ticks=1687（≈84s）12/12 |
| 5 | 01:48–01:52:09 | 0 | **SUCCESS** | `failed=[]` ticks=1670（≈84s）12/12 |

- 每轮任务前就绪一致（sim 15 px4、onboard 15/15、`calibration_ready=true`、takeoff 15/15、
  tcp 39001 监听正常）；结果差异全部出现在任务发出后的执行阶段。
- 失败轮自动留档：`.ros_home_round2`（342M，含 PX4 `sitl_iris_*` ulog/GCS 层日志/07_task JSON）；
  成功轮不留档。全程日志：`.tmp/logs/round{1..5}_prepare.log`。

## 3. R2（A08/A09）失败 ULog 分析（`.tmp/plots/r2_260909/`）

goal-epoch=1788889113.687843（backend `GCS_A goal`），heading 缺省 -0.006（round 日志未记录
标定航向，目标变换字段不可信；**pair 距离为同帧 ULog 实测，不受 heading 影响**）。

| 机对 | 实测最小 3D | 时刻（goal+） | <1.0m 累计 | <1.1m 累计 |
|---|---|---|---|---|
| UAV6-UAV8 | 0.966m | +29.2s | 0.55s | 1.05s |
| UAV6-UAV9 | **0.778m** | +30.3s | 1.10s | 1.25s |
| UAV8-UAV9 | 0.812m | +25.6s | 3.20s | 5.85s |
| UAV1-UAV2 | 2.552m | — | 0 | 0 |

- 失败时刻 ticks=613 ≈ goal+30.7s，恰在 6/8/9 相互 <1.0m 破距窗口内 → 机载距离门禁/监督触发
  A08/A09 FAILED → 整组 ALL 收口。**实际破距为根因，非排程预测不足**：backend 排程 pair
  A08/A09 predicted_dmin≈12.6m（直线水平模型），EGO 实际轨迹在末端走廊汇合处收敛 <1m。
- 局限：planner 结构化监督日志（runtime_logs/ego_planner）每轮 cleanup 删除，无法复核预测级
  EMERGENCY 等级；console 事件与 structured 均为空（metrics `structured_any_emergency: False`
  仅表示无该日志，不代表未触发）。判定基于 ULog 实测 3D 距离。

## 4. 结论

**结果：4/5 通过（R1/R3/R4/R5 SUCCESS，R2 FAIL）。**

- 与本配置参数相关的成功轮 ticks 1617–1690（81–85s），与 26090722/26090816 正常区间一致，
  无排程过宽/任务变慢迹象；5 轮无排程层失败。
- R2（A08/A09 + A06，约 31s，实际 3D 0.778–0.966m 破 1.0m 门禁）属 15 机 SITL 既有
  "末端走廊多机汇合 / 机载 intent·监督偶发"问题族（同 26090816 R5 A05/A06、26090722 R5
  A08/A09 模式），非本次三项参数直接引入。
- **对比基线**：26090802 实施期（实时判据）1/5 → 回退后 4/5 → 本次（1.3m/0.6mps/SLOW1.5）
  4/5。结论：三项参数在该轮次无退化、效果中性偏稳。
- 编排层修复（tcp_server 39001 残留 → v2 强杀）为必要前置，否则统计被污染；cleanup.sh 本身
  未改（测试脚本仓库 clean）。

## 5. 关联

- 参数提交：`e96127f`（tcp_to_ros，1.3m/0.6mps）、`ae4654d`（ego_planner_driver，SLOW 1.5 档）
- 历史对照：`test_record_26090816.md`（回退后 4/5）、`test_record_26090814.md`（回退前 1/5）、
  `test_record_26090722.md`（4/5，v=0.7 基线）
- R2 分析产物：`.tmp/plots/r2_260909/{latest_flight_overview.png, uav8_uav9_focus.png,
  latest_flight_metrics.json, latest_flight_events.csv}`
- 注：本记录书写时 `horizontal_speed_mps` 已按后续指示改回 0.7（见 plans/README 状态与
  `test_records` 索引），本记录内容为 **0.6 m/s 配置下**的实际结果。
