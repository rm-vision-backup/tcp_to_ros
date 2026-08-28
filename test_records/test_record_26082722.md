# 轨迹末端 OOM 最终修复测试记录

时间：2026-08-28 02:xx-03:05 CST
记录人：____________
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

## 1. 修改文件（全部在工作区内）

未修改：`swarm_topology_bridge`、`gps_bias_node.py`、PX4 参数、`UavTrajectoryIntent.msg` 协议。

| 文件 | 改动 |
|---|---|
| `ego_planner_driver/include/.../ego_planner_core.h` | 新增 `PlanResult{SUCCESS,DEGENERATE_SEGMENT,RETRYABLE_FAILURE,RESOURCE_LIMIT}`、编译期绝对硬上限、EgoPlannerConfig 资源字段、`planOnce` 返回 `PlanResult` |
| `ego_planner_driver/src/ego_planner_core.cpp` | 有界 `[0,T]` 采样（`N=ceil(T/0.02)+1`、checked size_t、分配前校验、超限 RESOURCE_LIMIT、失败清空输出）；`planOnce` 四类分流 + `bad_alloc` containment；TimedTrajectory nominal step 0.1s + 末样本精确 = duration |
| `uniform_bspline.h/.cpp` | `parameterizeToBspline` → `static bool` + `max_points`；单次 QR 三列 RHS、rank/residual（系统幅值相关）/finite 校验、失败清空输出 |
| `ego_planner_driver_node.h/.cpp` | `tryYieldReplan` 返回三态 `YieldResult`；RESOURCE_LIMIT 立即 `EGO_PLAN_FAILED`、DEGENERATE 等待真实 odometry；intent 样本上限；replanLoop `bad_alloc` containment；资源参数 fail-fast |
| 两个 launch | 5 个资源参数接线（4096/256/4096/256/0.05）；30Hz/0.2s 冻结契约未动 |
| `test_ego_launch_config.py` | 资源参数 + 30Hz/0.2s 静态断言 |
| `CMakeLists.txt` | 注册 `test_short_final_segment` |
| `test/test_short_final_segment.cpp`（新增） | fork 受限 worker（AS 512MiB/CPU/wall）、分类矩阵、RSS 检查点、ru_maxrss、`SHORT_FINAL_STRESS_LOOPS` 覆盖循环数 |
| `test/test_trajectory.cpp` | `planOnce` 断言适配 `PlanResult` |
| `test/test_ego_state_machine.cpp` | `CompletedFreezesInHold` 目标改非退化 (1,0,15)；新增 `DegenerateSamePointWaitsForOdometryHeldLock` |
| `ego_planner_driver/README.md` | PlanResult 分类表 + 资源参数行 |
| `tcp_to_ros/plans/README.md` | 计划索引已实施 + 当前状态段 + test_record 索引 |

## 2. 测试命令与结果（全部实测）

| 台阶 | 命令 | 结果 |
|---|---|---|
| 受限短末段 | `timeout 120 prlimit --as=536870912:536870912 --cpu=60 ./devel/.private/ego_planner_driver/lib/ego_planner_driver/test_short_final_segment --gtest_color=no` | `[  PASSED  ] 6 tests`（功能矩阵/阈值两侧/退化不制动/NaN-Inf/60m cap，worker RSS 门禁通过：最终 child ru_maxrss <384MiB、后半段 RSS max-min <=32MiB、末检查点 <=中点+16MiB） |
| ego_planner_driver | `catkin run_tests ego_planner_driver --no-deps -j1 -p1` + `catkin_test_results --all build/ego_planner_driver` | **164 tests, 0 errors, 0 failures** |
| swarm_uav_executor | `catkin_test_results --all build/swarm_uav_executor` | **94 tests, 0 errors, 0 failures** |
| tcp_to_ros | `catkin_test_results --all build/tcp_to_ros` | **28 tests, 0 errors, 0 failures** |
| 全工作区 | `catkin test -j1 -p1` + `catkin_test_results --all build` | **295 tests, 0 errors, 0 failures, 0 skipped** |
| 500 次压力 | `timeout 180 prlimit --as=... --cpu=120 ... -gtest_filter=ShortFinalSegment.StressLoopMemoryPlateau` | `[  PASSED  ] 1 test`（500×5 次 planOnce） |
| 2000 次长压力 | `SHORT_FINAL_STRESS_LOOPS=2000 timeout 300 prlimit --as=... --cpu=240 ... -gtest_filter='StressLoopMemoryPlateau:FunctionalMatrixBoundedAndClassified'` | `[  PASSED  ] 2 tests`（2000×5 次 planOnce，RSS 平台稳定） |

## 3. 15-UAV active prepare 端到端验收（2026-08-28，受 watchdog 保护）

环境：startup_ego_sitl.sh 等价流程（cleanup → 15 模型 headless PX4 → 15 套 onboard active 一次性启动 → bridge/backend → 标定+origin → 外部 arm/OFFBOARD → planner TAKEOFF 5m → HOLD → TCP 39001 → prepare 批次）。

| 阶段 | 结果 |
|---|---|
| 全环境 clean | ✅ 0 残留，11300/11310/11311-11325/11345/39001 全 free |
| sim 15 模型 | ✅ 15 px4 + 15 spawn |
| onboard 15 机（active） | ✅ 15/15 ready |
| bridge/backend/标定 | ✅ /UAV1-15 服务代理 OK、calibration_ready=True |
| takeoff 15/15 | ✅ z 4.79-5.13 m，全部 OFFBOARD |
| TCP 39001 | ✅ LISTENING |
| prepare 批次 | ❌ FAILURE(A02)，ticks=1605；A02 终态 HOLD（避碰规划无解已知类） |

### 内存/资源门禁（本修复核心判据）——全部 PASS

| 判据 | 实测 |
|---|---|
| planner 存活 | ✅ 15/15，无死亡/bad_alloc/SIGABRT/-9 |
| 每 planner RSS | ✅ 11-12 MB 稳定平台（修复前 5.9-12.0 GiB） |
| host 内存 | ✅ 峰值可用 20 GiB，无 OOM/swap |
| OFFBOARD | ✅ 15/15 全程保持 |
| relay 实测频率 | ✅ 30.000 Hz；max 间隔 0.034 s < 0.2 s |
| memory events | ✅ 无 OOM-kill/-6/-9 |

结论：OOM 修复 15-UAV 判据（内存平台、无 OOM/死、OFFBOARD、30Hz）全部通过；prepare 任务级 FAILURE 属既有避碰无解类别，非本次 OOM 根因；整轮按 §8 FAIL 清理（0 残留，日志存 `.ros_home/log/headless_test/`）。

## 4. 结论与遗留

- 根因（`[0,1]` 外推采样 + 无界 point set + 三次 QR）已修复；内存门禁/全量自动测试/500/2000 次压力全部通过；15-UAV 内存平台与无 OOM 判据通过。
- residual 发散检测采用系统幅值相关阈值（`residual.norm() > 1e-3 * (||A||∞*(||x||+1)+||b||+1)`），无误伤正常段。
- 遗留：15 机密集编队 prepare 任务级 FAILURE（A02/HOLD）——避碰规划无解类别，与 OOM 无关，继续由既有避碰改进方向处理。
- 未验证项：多机走廊式/并发避碰（已知问题，见 plans/README「换位/三机/编队未通过」）、Phase 6 实机。
