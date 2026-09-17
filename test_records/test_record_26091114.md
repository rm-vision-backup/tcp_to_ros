# 测试记录 26091114：26091101 实施后 5 轮 prepare SITL 复测（4/5）+ 新留档规则验证

时间：2026-09-11 13:54–14:16 CST
记录人：cline-deepseek
对应计划：`plans/implementation_plan_26091101.md`（已实施，见 `test_record_26091103.md`）
工作空间：`/home/yjq/catkin_swarm6-2`

## 0. 摘要

- 目的：实机部署适配（26091101）实施后的干净 stack 5 轮 prepare 回归，并验证
  `test_records/README.md` §8 新留档规则（成功轮也留档 `.ros_home_round<n>_<s|f>`；
  启动阶段/端口原因失败不计轮）。
- 结果：**4/5 SUCCESS**（R1/R3/R4/R5），R2 FAILURE（真实 MIN_DISTANCE 事件 → 组级 fan-out
  直锁 10 机）。与历史基线（4/5）一致；无实施回归迹象。
- 驱动：`.tmp/scripts/run_5_prepare_matrix_260911b.sh`（v3：每轮前强杀 tcp_server 39001 残留；
  结果按 `_s/_f` 归档；`ticks < 100` 视为未真正执行 prepare → 不计轮、重跑同一编号）。

## 1. 结果

| 轮 | 时间 | 结果 | failed | ticks(≈s) | 留档 |
|---|---|---|---|---|---|
| R1 | 13:54–13:58 | SUCCESS | [] | 1658(≈83s) | `.ros_home_round1_s`（491M） |
| R2 | 13:59–14:02 | FAILURE | A01,A07 | 1169(≈58s) | `.ros_home_round2_f`（423M） |
| R3 | 14:03–14:06 | SUCCESS | [] | 1669 | `.ros_home_round3_s`（486M） |
| R4 | 14:07–14:11 | SUCCESS | [] | 1651 | `.ros_home_round4_s`（498M） |
| R5 | 14:12–14:15 | SUCCESS | [] | 1631 | `.ros_home_round5_s`（472M） |

- matrix **4/5 SUCCESS**；成功轮 ticks 1631–1669（正常区间，无排程滑移）。
- 无“启动阶段/端口失败”轮次（5 轮均真正执行 prepare，ATTEMPT=1，无重跑）。

## 2. R2 失败证据

- task JSON（`.ros_home_round2_f/log/headless_test/07_task_prepare.log`）：
  `status=FAILURE failed_robot_ids=[A01, A07] ticks=1169`（≈58s，中段）。
- backend（`gcs_a_backend_node-1.log`）：
  `14:02:08,298 GCS_A group safety hold triggered by A07 (MIN_DISTANCE_BREACH);
  targets=A02,A03,A04,A05,A06,A08,A09,A10,A11,A12`（一次性 fan-out，排除触发者 A07 与已 terminal 的 A01）。
- 机载：10 个 UAV `rosout.log` 含 `direct BRAKE_HOLD locked`（组级直锁端到端正常）。
- 判定：真实物理事件（同既有走廊 MIN_DISTANCE 问题族），非本计划实施的回归。

## 3. 收尾

- 测试后进程 0、端口（39001/11300/11310/11345/1131x）0；默认 `.ros_home` 已清空（仅留空 log 目录）。
- 编排日志：`.tmp/logs/matrix5_260911b.log`、`.tmp/logs/round{1..5}_prepare.log`。
- 留档规则验证通过：SUCCESS/FAILURE 均按 `.ros_home_round<n>_<s|f>` 留档。

## 4. 关联

- 计划与实施证据：`plans/implementation_plan_26091101.md`、`test_record_26091103.md`。
- 留档规则：`test_records/README.md` §8。
- 历史对照：`test_record_26091000.md`（实施前 4/5）。
