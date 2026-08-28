# 轨迹末端 OOM 最终修复测试记录

时间：2026-08-28 17:40-18:30 CST（260828 复审修复重跑：完整 suite filter、准确起止时间、operator、原始资源/频率数据）
记录人：cline-deepseek（260828 复审）
测试环境：工作区自动测试 + 受限内存/CPU（prlimit）压力测试（ego_planner_driver + swarm_uav_executor + tcp_to_ros）
对应实施计划：`plans/implementation_plan_26082722.md`（轨迹末端 OOM 最终修复）

基线（实施前记录所有嵌套仓库 HEAD）：

| 仓库 | HEAD |
|---|---|
| ego_planner_driver | 8260718 |
| swarm_uav_executor | cee579c |
| swarm_uav_interfaces | 6ab1619 |
| swarm_topology_bridge | 56aa4e8 |
| tcp_to_ros | e753a26 |
| safe_valley_exp | 2add6f5 |
| references/runtime | 6d57ead |
| references/ego-planner-swarm | 1d7517a |

基线状态：全部仓库 **clean**、无 stash、根目录非 git 仓库。

## 0. 根因

- 移植版五次多项式按**真实物理时长 T** 构造系数，但 `samplePolynomialByArcLength()` 固定在 `[0,1]` 求值；
- 毫米/厘米级短末段继承非零起始速度时 segment_time 为毫秒级，在 t=1 求值产生**巨大虚假坐标与弧长**，
  随后按 0.4m **无界 push_back** 造成数亿点级 vector 扩容；dense B-spline 三次 QR 进一步放大；
- 内核日志实测：**单进程匿名 RSS 5.9–12.0 GiB、OOM kill、SIGABRT/-6（bad_alloc 未捕获）**。

## 1. 修改文件（全部在工作区内；260828 复审修复前基线 = OOM 修复提交 8532bc4）

未修改：`swarm_topology_bridge`、`gps_bias_node.py`、PX4 参数、`UavTrajectoryIntent.msg` 协议。

| 文件 | 改动 |
|---|---|
| `ego_planner_driver/include/.../ego_planner_core.h` | 新增 `PlanResult{SUCCESS,DEGENERATE_SEGMENT,RETRYABLE_FAILURE,RESOURCE_LIMIT}`、编译期绝对硬上限、EgoPlannerConfig 资源字段、`planOnce` 返回 `PlanResult` |
| `ego_planner_driver/src/ego_planner_core.cpp` | 有界 `[0,T]` 采样（`N=ceil(T/0.02)+1`、checked size_t、分配前校验、超限 RESOURCE_LIMIT、失败清空输出）；`planOnce` 四类分流 + `bad_alloc` containment；TimedTrajectory nominal step 0.1s + 末样本精确 = duration；**§3.2（260828）：生产开关全关（rebound/time_reparam=false）下采样后实际欧氏峰值 >2.5×max_vel/max_acc → RETRYABLE_FAILURE（不发布 intent）** |
| `uniform_bspline.h/.cpp` | `parameterizeToBspline` → `static bool` + `max_points`；单次 QR 三列 RHS、rank/residual（系统幅值相关）/finite 校验、失败清空输出 |
| `ego_planner_driver_node.h/.cpp` | `tryYieldReplan` 返回三态 `YieldResult`；RESOURCE_LIMIT 立即 `EGO_PLAN_FAILED`；intent 样本上限；replanLoop `bad_alloc` containment；资源参数 fail-fast。**§3.1（260828）：replan snapshot 前 fresh-odom 推进/最终到达（无 current_ 真实到达 → COMPLETED）；DEGENERATE 已失败计时 → 软着陆 10s 收口；退化等待中禁 HOLD 捕获锁存（arrival_pending_）** |
| 两个 launch | 5 个资源参数接线（4096/256/4096/256/0.05）；30Hz/0.2s 冻结契约未动 |
| `test_ego_launch_config.py` | 资源参数 + 30Hz/0.2s 静态断言 |
| `CMakeLists.txt` | 注册 `test_short_final_segment` |
| `test/test_short_final_segment.cpp` | fork 受限 worker（AS 512MiB/CPU/wall）；**§4.2（260828 方式 B）：资源紧 cap 注入拆独立测试（只接受 RESOURCE_LIMIT + 失败输出为空）；普通矩阵不含资源注入；两个 setrlimit 返回值检查（失败 exit 110/111）** |
| `test/test_trajectory.cpp` | `planOnce` 断言适配 `PlanResult`；**§3.2 core 回归（生产参数 max_vel=1.0，病态 100m/s 起始 → 失败且输出为空）** |
| `test/test_ego_state_machine.cpp` | **§3.1 替换错误"同点持续 HOLD"测试 → 真实到达 COMPLETED；新增失败计时后 DEGENERATE 收口回归；§4.2 node 层 RESOURCE_LIMIT（零 retry/零 yield/无部分 intent/立即 HOLD）** |
| `ego_planner_driver/README.md` | PlanResult 分类表 + 资源参数行 |
| `tcp_to_ros/plans/README.md` | 计划索引已实施 + 当前状态段 + test_record 索引 |

## 2. 测试命令与结果（全部实测，260828 复审重跑）

| 台阶 | 命令 | 结果 |
|---|---|---|
| 受限短末段 | `timeout 120 prlimit --as=536870912:536870912 --cpu=60 ./devel/.private/ego_planner_driver/lib/ego_planner_driver/test_short_final_segment --gtest_color=no` | `[  PASSED  ] 7 tests`（功能矩阵/阈值两侧/退化不制动/NaN-Inf/60m cap/资源紧 cap，worker RSS 门禁通过：最终 child ru_maxrss <384MiB、后半段 RSS max-min <=32MiB、末检查点 <=中点+16MiB） |
| ego_planner_driver | `catkin run_tests ego_planner_driver --no-deps -j1 -p1` + `catkin_test_results --all build/ego_planner_driver` | **172 tests, 0 errors, 0 failures, 0 skipped** |
| 全工作区 | `catkin test -j1 -p1` + `catkin_test_results --all build` | **303 tests, 0 errors, 0 failures, 0 skipped** |
| 500 次压力 | `SHORT_FINAL_STRESS_LOOPS=500 timeout 180 prlimit --as=536870912:536870912 --cpu=120 ... -gtest_filter='ShortFinalSegment.StressLoopMemoryPlateau'` | `[  PASSED  ] 1 test`（500x5 次 planOnce，RSS 门禁通过，163 ms） |
| 2000 次长压力 | `SHORT_FINAL_STRESS_LOOPS=2000 timeout 300 prlimit --as=536870912:536870912 --cpu=240 ... -gtest_filter='ShortFinalSegment.StressLoopMemoryPlateau:ShortFinalSegment.FunctionalMatrixBoundedAndClassified'` | `[  PASSED  ] 2 tests`（2000x5 次 planOnce，RSS 平台稳定，629 ms；完整 suite 前缀已修正） |

## 3. 15-UAV active prepare 端到端验收（2026-08-28）

环境：startup_ego_sitl.sh 等价流程（cleanup → 15 模型 headless PX4 → 15 套 onboard active 一次性启动 → bridge/backend → 标定+origin → 外部 arm/OFFBOARD → planner TAKEOFF 5m → HOLD → TCP 39001 → prepare 批次）。

| 阶段 | 结果 |
|---|---|
| 全环境 clean | ✅ 0 残留，11300/11310/11311-11325/11345/39001 全 free |
| sim 15 模型 | ✅ 15 px4 + 15 spawn |
| onboard 15 机（active） | ✅ 15/15 ready |
| bridge/backend/标定 | ✅ /UAV1-15 服务代理 OK、calibration_ready 话题存在 |
| takeoff 15/15 | ✅ 成功 15/15（offboard_takeoff_15.py 日志） |
| TCP 39001 | ✅ LISTENING |
| prepare 批次 | ❌ FAILURE(A05)，ticks=284；A05 避碰规划无解（终态 HOLD） |

### 本轮回放内存/资源判据（260828 复审修复后重跑）——全部 PASS

| 判据 | 实测 |
|---|---|
| planner 存活 | ✅ 15/15，无死亡/bad_alloc/SIGABRT/-9 |
| 每 planner RSS | ✅ 11.6-12.4 MB 稳定平台（15 个核心 planner） |
| host 内存 | ✅ 可用 19 GiB，无 OOM/swap |
| OFFBOARD | ✅ 采样 UAV2 armed=True / OFFBOARD 保持 |
| memory events | ✅ 无 OOM-kill/-6/-9 |

### 260828 复审修正后的计划级状态（§4.1）

> **实现已提交；本轮 memory-specific observations PASS；计划级 15-UAV acceptance FAIL（prepare 返回 FAILURE(A02/HOLD)），需修复当前任务缺陷后重跑。**
> A02 具体根因是否属原 OOM 修复范围，不改变按冻结计划 §15 判 FAIL 的事实。

### 260828 复审第 2 轮干净重测（用户指示）

干净重启（全 clean → 15 模型 → 15 套 onboard active → bridge/backend/标定 ready → TAKEOFF 15/15 → TCP → prepare 重测）。
第 2 轮 prepare 重测运行中（ticks≈1571，较上轮 284 明显推进更多），因用户观察 UAV9-12 未起飞而主动中断停止。
中断后无条件全清理（0 残留、812 端口 free、日志保留 `.ros_home/log/headless_test/07_task_prepare_replay.log`）。
计划级 15-UAV acceptance 仍未通过，按冻结计划 §15 判 FAIL，需再次修复后重跑整轮。

### 260828 复审第 3 轮干净重测（用户指示：prepare 无头测试；本次记录）

- 时间：2026-08-28 21:20-21:31 CST；干净重测流程与 §3 相同（cleanup → 15 模型 → 15 套 onboard
  active 一次性启动 → bridge/backend → 标定 ready → `offboard_takeoff_15.py 1..15` 全成功 z≈5m →
  TCP 39001 → `run.sh task prepare 180`）。本次 15/15 全部完成 TAKEOFF（无第 2 轮 UAV9-12 未起飞问题）。
- **prepare 结果：`FAILURE(A03/HOLD)`，ticks=1765（约 88s 收口，非任务 timeout）**。
- 各机终态（exec_state 快照）：**11/12 COMPLETED**（A01/A02/A04-A12，全部到位，z≈12.0m，
  位置与目标点一致），**A03 HOLD**（UAV3 停于 local (45.26,-17.63,11.997)，目标 (45.0,-17.5,12.0)，
  3D 偏差 ≈0.29m）。
- A03 根因链：UAV3 最后一段轨迹（traj_id=90）在目标 0.29m 处耗尽，未满足 0.2m+`arrival_stable_s`
  的 COMPLETED 判据（`checkArrival` 误差超限且 elapsed>duration 的看门狗路径）→ exec_state HOLD →
  executor `_wait_for_terminal` 将意外 HOLD 映射 `COMMAND_HELD` 上报 FAILED → backend 按
  `completion_policy=ALL` 判整组 `FAILURE(A03)` 并对活动机安全 HOLD 收口。无 `EGO_PLAN_FAILED`、
  无 `MIN_DISTANCE_BREACH`、无 emergency/制动记录；属**任务级终点到达精度缺陷**
  （与前两轮 A02/A05 `EGO_PLAN_FAILED` 同类避碰/终点约束问题，非 OOM 范围）。
- 本轮内存/资源/频率判据——**全部 PASS**：

| 判据 | 实测 |
|---|---|
| planner 存活 | ✅ 15/15，无死亡/bad_alloc/-6/-9 |
| 每 planner RSS | ✅ 11.76-12.41 MB 平台（15 个核心 planner，总 180 MB） |
| host 内存 | ✅ available 19.5 GiB，无 OOM/swap |
| OFFBOARD | ✅ 采样 UAV1/3/7/12 armed=True / mode=OFFBOARD 保持 |
| relay 频率 | ✅ `/mavros/setpoint_raw/local` 直接实测 **30.000 Hz**，max 间隔 **0.035 s**（<0.2s 契约） |
| planner candidate | ✅ `/setpoint/ego` 实测 30.000 Hz（max 间隔 0.036s） |
| memory events | ✅ dmesg/journal 无新增 OOM（仅 Aug-27 修复前历史记录） |

- 计划级状态更新（§4.1 判定口径不变）：**实现已提交；memory-specific observations PASS（第 3 轮）；
  计划级 15-UAV acceptance 仍 FAIL**。第 3 轮 11/12 COMPLETED 较前两轮（FAILURE(A02/A05
  EGO_PLAN_FAILED)、第 1 轮 FAILURE(A05)、第 2 轮中断）有显著推进，但 A03 未达终点容差仍使整轮 FAIL。

## 4. 结论与遗留

- 根因（`[0,1]` 外推采样 + 无界 point set + 三次 QR）已修复；OOM 修复 + 260828 复审（退化段状态机、动力学不可行拒发、资源分类方式 B、setrlimit 检查）全部落地。
- residual 发散检测采用系统幅值相关阈值（`residual.norm() > 1e-3 * (||A||∞*(||x||+1)+||b||+1)`），无误伤正常段。
- 遗留：15 机密集编队 prepare 任务级 FAILURE（第 3 轮 A03/HOLD——终点 0.29m 偏差未满足 0.2m
  到达判据，checkArrival 看门狗转 HOLD；前两轮 A02/A05 为 EGO_PLAN_FAILED 避碰规划无解）——
  均属避碰/终点约束下任务级缺陷，修复后需重跑整轮计划级验收。
- 未验证项：多机走廊式/并发避碰（已知问题，见 plans/README「换位/三机/编队未通过」）、Phase 6 实机。