# 测试记录 26090816：26090802 源码回退后 prepare 五次干净重启复测

> 配置：无 runtime HOLD 注入；`schedule_safety_margin_m: 1.0`。
> 背景：implementation_plan_26090802（A–F）2026-09-08 实施后当日回退源码至实施前（新增提交、不删历史：
> `ego_planner_driver` dd27824 / `swarm_uav_executor` 395a9b4 / `tcp_to_ros` 89a6ff6；说明文档
> a5c77f9/1b77aaf）。本记录为**回退后源码**下由 `src/swarm_simulation/bashs/round_prepare_15sitl.sh`
> 驱动的五次复测（对比：回退前 `test_record_26090814.md` 1/5）。

## 1. 环境与配置

- workspace：/home/yjq/catkin_swarm6-2（PX4 SITL 15 机，无头；源码已回退至 26090802 实施前）
- 参与集：A01–A12（reserve A13–A15 在场不参与）
- 编排：`round_prepare_15sitl.sh --round N --task prepare --timeout-s 150 --keep-ros-home-on-fail 1`
  ＋ `.tmp/scripts/run_5_prepare_matrix.sh`；每轮 clean→隔离 `.ros_home`→sim→onboard 15/15→
  GCS（bridge→backend）→`calibration_ready=true`→tcp→takeoff 15/15→prepare→RESULT→cleanup。
- `schedule_safety_margin_m: 1.0`；runtime accept 5s；safety `active`。
- 回退后构建与单测先行：`ego_planner_driver` 172、`swarm_uav_executor` 113、`tcp_to_ros` 57，
  均 0 failures（executor rostest `test_service_and_terminal_topic` 曾一次 flaky，单独重跑通过）。
- 旧失败轮留档 `.ros_home_round1/2/3/5`（回退前）已按要求删除。

## 2. 结果汇总

| 轮 | 起止 | 退出码 | 结果 | 详情 |
|---|---|---|---|---|
| 1 | 16:37:21–16:41:06 | 0 | **SUCCESS** | `failed=[]` ticks=1798（≈90s）12/12 COMPLETED |
| 2 | 16:41:06–16:44:41 | 0 | **SUCCESS** | `failed=[]` ticks=1660（≈83s）12/12 |
| 3 | 16:44:41–16:48:16 | 0 | **SUCCESS** | `failed=[]` ticks=1628（≈81s）12/12 |
| 4 | 16:48:16–16:52:01 | 0 | **SUCCESS** | `failed=[]` ticks=1738（≈87s）12/12 |
| 5 | 16:52:01–16:54:47 | 1 | **FAIL** | `failed=[A05, A06]` ticks=599（≈30s，机载执行中途） |

- 每轮任务前就绪一致（onboard 15/15、`calibration_ready=true`、takeoff 15/15），差异均在任务发出后的执行阶段。
- 失败轮自动留档：`.ros_home_round5`（332M，含 PX4 ulog / GCS 层日志 / 07_task JSON）；成功轮不留档。
- 全程日志：`.tmp/logs/matrix_rollback.log` 与 `.tmp/logs/round{1..5}_prepare.log`。

## 3. 结论

**结果：4/5 通过（R1–R4 SUCCESS，R5 FAIL）。**

- 与**回退前**源码复测（`test_record_26090814.md`，1/5，失败含机载偶发 ×3 ＋ 组级 5s 确认超时 ×1）相比，
  回退后回升至 **4/5**，与 26090722（margin=1.0，回退前更早基线）的 4/5 一致——支撑"26090802 实施是
  回退前 1/5 退化主因"的假设。
- R5（A05/A06，≈30s 机载执行中途双机失败）属 15 机 SITL 既有"机载 intent/监督偶发"问题族（同
  26090722 round5、26090814 R1/R3/R5 模式），非本次回退引入；此等偶发需机载链路侧单独排查。
- 编排链路（clean→就绪→标定→takeoff→任务→收口）5/5 轮稳定；剩余缺口为既有机载偶发。

## 4. 关联

- 回退提交：dd27824 / 395a9b4 / 89a6ff6（源码）；a5c77f9 / 1b77aaf（文档）
- 计划与状态：`src/tcp_to_ros/plans/README.md` 状态-26090802（已标回退）
- 历史对照：`test_record_26090814.md`（回退前 1/5）、`test_record_26090722.md`（4/5）
