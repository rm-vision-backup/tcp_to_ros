# 测试记录 26090814：swarm_simulation/bashs 15-UAV prepare 五次干净重启复测

> 配置：无 runtime HOLD 注入（被测为正常全链路；`schedule_safety_margin_m: 1.0`）。
> 驱动：正式入库脚本 `src/swarm_simulation/bashs/round_prepare_15sitl.sh`（首版整套单轮，
> 替代历史 `.tmp/headless_test` 编排）＋ `.tmp/scripts/run_5_prepare_matrix.sh`（5 轮串联）。
> 方法同 implementation_plan_26090106 §8.4 / test_record_26090722：每轮从干净状态启动
> sim→onboard→GCS→标定→takeoff→`task prepare 150`→cleanup；失败即记、不重试覆盖；
> 每轮独立隔离 `.ros_home`，失败轮复制为 `.ros_home_roundN`。
> 本记录同时承接 `plans/README.md` 状态-26090802 的"15-UAV prepare 无头回归未验收"项。

## 1. 环境与配置

- workspace：/home/ub20tg/catkin_swarm6-2（PX4 SITL 15 机，无头 `gui:=false`）
- 参与集：A01–A12（reserve A13–A15 在场不参与 prepare）
- 编排：`src/swarm_simulation/bashs/round_prepare_15sitl.sh --round N --task prepare
  --timeout-s 150 --keep-ros-home-on-fail 1`（cleanup 内建于 EXIT trap，轮末进程 0 / 端口 clean）
- `schedule_safety_margin_m: 1.0`（backend_defaults.yaml；排程预测 d_ij≈2.0m，口径同 26090722）
- 机载安全：`safety_supervisor_mode: active`；planner `swarm_clearance=0.5`（×2 = 1m hard-clearance）；
  26090802 变更在环（`replan_retry_period_s=0.6`、同输入重试 5 收口、EMERGENCY 直接锁存
  `BRAKE_HOLD`、executor 按 `reason=GROUP_SAFETY` 安全锁存、backend 白名单组级广播默认
  `all_enabled` 含 reserve）
- runtime accept_timeout：5s（与既有五次验收一致）
- git：当前工作区未处于 git 版本控制下，本次无法标注提交 hash；bashs 正式版见
  `src/swarm_simulation/bashs/`（README 同目录）

## 2. 结果汇总

| 轮 | 起止 | 退出码 | 结果 | 详情 |
|---|---|---|---|---|
| 1 | 13:24:59–13:28:10 | 1 | **FAIL** | `failed=[A02]` ticks=1061（≈53s，机载执行中途失败） |
| 2 | 13:28:10–13:30:27 | 1 | **FAIL** | `failed=[A01..A12]` ticks=101（≈5s，runtime 5s ACCEPTED 窗口内未整组确认 → 组级取消） |
| 3 | 13:30:27–13:33:08 | 1 | **FAIL** | `failed=[A09]` ticks=551（≈27.5s，机载执行中途失败） |
| 4 | 13:33:08–13:36:43 | 0 | **SUCCESS** | `failed=[]` ticks=1628（≈81s）12/12 COMPLETED |
| 5 | 13:36:43–13:39:33 | 1 | **FAIL** | `failed=[A05]` ticks=769（≈38s，机载执行中途失败） |

- ticks→时间按 runtime 20Hz（0.05s/tick）；总时长约 14.6 min。
- 失败轮均已留档：`.ros_home_round1`（399M）、`.ros_home_round2`（251M）、
  `.ros_home_round3`（317M）、`.ros_home_round5`（343M）；成功轮（R4）不留档。
  默认 `.ros_home` 在全部结束后已清空（不占用）。

## 3. 每轮就绪证据（任务发出前，各轮一致）

- onboard：15/15 `ready`（11311..11325，单次启动，无补启）；
- GCS：bridge→backend 顺序启动，`calibration_ready=true`；
- takeoff：`ok=1`（15/15，HOLD→arm→OFFBOARD→TAKEOFF）。
- 即：**5 轮在任务发出前链路就绪状态相同**，结果差异全部出现在任务发出之后的
  确认/执行阶段。

## 4. 失败归类（诚实记录）

- **R1/R3/R5（A02/A09/A05，ticks 1061/551/769）**：单机在 prepare 执行中途失败，属 15 机
  SITL 既有"机载 intent/监督偶发"同族（与 26090722 round5 A08 EMERGENCY/A09
  INSUFFICIENT_DATA、26090106 margin=0.5 时 0/5 中的失败模式一致），非编排/GCS 层错误。
- **R2（ticks 101，整组）**：runtime 在 5s ACCEPTED 窗口内未收到全参与者确认 → 组级取消
  （所有 A-id 判 FAILED）。本系列中属新模式；该轮 backend stdout 日志为空（无头重定向缓冲），
  留档中无直接可归因文本，需从 `.ros_home_round2` 的 PX4 ulog / GCS 层日志二次分析。
- 机载文本日志（planner/executor stdout，`.tmp/logs/UAVn_offboard_ego.log`）按轮 cleanup 惯例
  删除，未随留档保留；留档内容为 `.ros_home_roundN`（PX4 `sitl_iris_*` ulog ＋ GCS 层日志 ＋
  07_task JSON），单机失败现场需基于 ulog 二次离线分析，本批未逐条还原。

## 5. 结论

**结果：1/5 通过（4 FAIL）。**

- R4 为完整 SUCCESS（排程/START/执行/12 机收口，无 failed）。
- 相较 26090722（margin=1.0 时 4/5），本次 1/5 明显退化；但 5 轮任务前就绪完全一致，
  差异集中于任务发出后，**不足以判定为脚本或配置引入的确定性回归**，也不满足"可重复高通过率"。
- 剩余 FAIL = 3× 机载 intent/监督偶发（既有问题族）＋ 1× 组级 5s 确认超时（新模式，待二次分析）。
- 结论：**本次复测不能视为 100% 通过**；bashs 正式编排的启动/就绪/标定/takeoff/收口链路
  5/5 轮稳定工作，执行稳定性缺口在机载侧（既有）与一次组级确认超时（待查）。

## 6. 附注

- matrix 驱动脚本 `.tmp/scripts/run_5_prepare_matrix.sh` 的 summary 语义已修正：汇总改为打印
  `exit_code`（脚本真实 $?），不再用 `grep -c SUCCESS` 计数（此前 `code=0` 易误读为失败）。
- 关联文档：`src/swarm_simulation/bashs/README.md`、`src/tcp_to_ros/test_records/test_record_26090722.md`
  与 `test_record_26090106.md`；本文件已登记进 `plans/README.md`（状态-26090802 未验收项更新）。
