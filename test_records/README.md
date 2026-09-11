# 测试记录目录（test_records）

> 本目录归档无头/有头测试的证据记录（按日期命名 `test_record_2608xxxx.md`），
> 对应 `plans/implementation_plan_2608xxxx.md` 实施计划。
> 启动流程与注意事项见本 README；`plans/` 为实施计划入口。

## 0. 测试记录索引

| 记录 | 内容 |
|---|---|
| `test_record_26082300.md` | 方案 B（机载分层航点 + C++ horizon/rebound）：prepare 首过（机间 0.64m 未达标） |
| `test_record_26082400.md` | 周期性零速修复 + PositionTarget/relay 唯一出口：prepare SUCCESS（机间 0.42m 未达标） |
| `test_record_26082420.md` | min-snap 单段重构：集群 prepare（距离门禁触发）、UAV1 776m 长轨迹 |
| `test_record_26082421.md` | EGO intent 时间契约与主动避碰审计 |
| `test_record_26082501.md` | 预测式避碰阶段 A–D 实施与自动测试 |
| `test_record_26082602.md` | 门槛解耦（固定 clearance）+ 周期碰撞 replan + 监督层修订 |
| `test_record_26082621.md` | plan-exec 循环重构 + 单一 replan 入口 + 方案 Y + 候选迭代 A/B |
| `test_record_26082700.md` | 双机正交 ×5（4/5）+ prepare 全 active + 软着陆修复 |
| `test_record_26082722.md` | 轨迹末端 OOM 修复 + 15-UAV acceptance（第 4 轮 SUCCESS） |
| `test_record_26082916.md` | 参数规范化 canonical YAML（317 tests；prepare SUCCESS） |
| `test_record_26083018.md` | `safe_valley_exp` 拆包 + 15-UAV prepare 三轮回归（预测窗 3s 后 SUCCESS） |
| `test_record_26090106.md` | t_off 排程实施；prepare 五次验收 **0/5**（机载 intent/监督） |
| `test_record_26090722.md` | prepare 五次验收 margin=1.0（d_ij=2.0m）：**4/5** |
| `test_record_26090814.md` | 26090802 实施期源码 prepare 复测：**1/5** |
| `test_record_26090816.md` | 26090802 回退后源码 prepare 复测：**4/5** |
| `test_record_26090902.md` | 排程参数 1.3m/0.6mps + SLOW 1.5 档 prepare 五次复测：**4/5**（R2 A08/A09 实际 3D 破距） |
| `test_record_26090919.md` | implementation_plan_26090900（direct BRAKE_HOLD）实施：单测 354/0/0；SITL 4 轮 0 SUCCESS（3 次真实 MIN_DISTANCE_BREACH → 组级 fan-out/10 机直锁实测） |
| `test_record_26091000.md` | direct BRAKE_HOLD 实施后干净 stack 5 轮 prepare 复测：**4/5**（R4 首次 infra=tcp 39001 残留；R4 补测真 MIN_DISTANCE → 组级直锁 10 机端到端再实测） |
| `test_record_26091103.md` | implementation_plan_26091101（实机部署适配）实施：自动测试 **367/0/0**；15-UAV prepare SITL **SUCCESS**（ticks=1648）；4 仓库 commits |
| `test_record_26091114.md` | 26091101 实施后 5 轮 prepare 复测：**4/5**（R2 真 MIN_DISTANCE → 组级直锁 10 机）；新留档规则 `.ros_home_round<n>_<s\|f>` 验证通过 |

> 记录命名 `test_record_yymmddhh.md`，时间后缀 = 文件生成时间；**实施状态以
> `plans/README.md`「实施计划（现存）」表为唯一权威**，测试记录只存证据。

## 1. 无头 vs 有头测试

| 维度 | 无头（headless） | 有头（GUI） |
|---|---|---|
| 仿真层 | `gui:=false`（`LIBGL_ALWAYS_SOFTWARE=1`） | `gui:=true`（Gazebo 窗口 + QGC 观察） |
| 用途 | **自主测试 / 回归**（脚本化、可批量、无人值守） | 人工诊断 / 现象观察（如 QGC 看机间画面） |
| 依赖 | 无显示依赖、资源省 | 需显示环境 |
| 注意 | 无头前清空 `.ros_home`（保留文件夹） | 有头与无头启动链相同，仅 `gui` 参数不同 |

**约定：自主测试一律使用无头**（`multi_uav_ego_15sim.launch gui:=false`）——避免 GUI 依赖与人工
操作，便于连续回归；**有头仅在需要 QGC/可视化人工观察或排障时使用**，两者启动链一致。

## 2. 完整启动流程（无头/有头共通，阶段 0→7）

> 推荐用正式入库的 `src/swarm_simulation/bashs/round_prepare_15sitl.sh`（见 §3.1）一键跑完
> 完整单轮（clean→sim→onboard→gcs→标定→tcp→takeoff→task→cleanup）；以下为人工分步的等价
> 步骤，阶段脚本统一用 `src/swarm_simulation/bashs/phases.sh`（见 §3.2），不再用旧 `.tmp/headless_test`。

### 阶段 0：预检
```bash
# 无残留仿真进程（Gazebo/PX4/ROS master/MAVROS 均为 0）
ps aux | grep -cE '[g]azebo|[p]x4|[r]osmaster|[m]avros'   # 期望 0
# 清空 .ros_home（保留文件夹），避免旧日志/pid 混淆
rm -rf .ros_home && mkdir -p .ros_home/log
```

### 阶段 1：仿真层（15 机 PX4 SITL）
```bash
bash src/swarm_simulation/bashs/phases.sh sim   # gui:=false, master 11300, gazebo 11345
# 等待：15 个 px4 进程 + 日志 Spawn status = 15
# 注：px4 sitl 进程**不注册 ROS 节点**（rosnode list 仅见 /gazebo /rosout），
#     就绪判定用 `ps aux | grep -c '[p]x4 '`（≥15），不要用 `rosnode list | grep sitl_`。
```

### 阶段 2：机载层（15 机独立 Master，**统一 active 一次性启动，不重启**）
```bash
# 15 机 MAVROS + executor 以统一监督模式一次性启动（默认 active，不再 shadow→active 重启）
EGO_SAFETY_SUPERVISOR_MODE=active bash src/swarm_simulation/bashs/phases.sh onboard   # 默认即 active
# 等待：02_onboard.log 中 "ready UAV" 达 15（11311..11325）
# 全部 15 机必须在同一次启动中 ready；不得对单机/子集做"重启/补启"。
```

### 阶段 3：GCS（bridge → backend → tcp，顺序避免 11310 竞态）
```bash
bash src/swarm_simulation/bashs/phases.sh gcs_bridge_only   # 先启动 bridge，等 /UAV1/uav_task 服务出现
bash src/swarm_simulation/bashs/phases.sh gcs_backend_only  # 再启动 backend
bash src/swarm_simulation/bashs/phases.sh tcp                # tcp_server_node 39001
```

### 阶段 4：标定（**必须在 takeoff 前，且各机 disarm**）
```bash
# 等 /group_a/calibration_ready = True（GCS 冻结 gp_origin 统一坐标系）
rosparam get /group_a/calibration_ready         # 期望 true（权威判据；backend 日志 stdout 缓冲可能为空，勿用日志判断）
rostopic echo -n 1 /group_a/calibration_ready   # 或话题方式（期望 data: True）
# 每机 gp_origin_confirmed 置位（坐标系统一后 local 距离 = 世界距离）
```
> 标定要求 A01-A06 全 disarm + 数据连续 2s + 航向 ≤30°；**armed 时 set_gp_origin 被拒**，
> 因此必须先标定再 takeoff。

### 阶段 5：起飞
```bash
python3 src/swarm_uav_executor/scripts/offboard_takeoff_15_sitl.py 1..15
# HOLD → arm → OFFBOARD；软起飞 z≈5m 由 ego TAKEOFF 状态自动完成
# 等待各机高度 >3m
```

> 已无"active 重启测试机"步骤：机载层在阶段 2 已统一 active 启动，
> 任何时刻不得再对机载层做 stop/start 单机或子集重启（会产生状态不同步）。

### 阶段 6：测试
```bash
# 批次任务（GCS TCP 全链路）
python3 references/runtime/task_test.py --backend tcp --group A --task prepare --timeout-s 180
# 双机正交避碰（直接调 executor 服务）
ROS_MASTER_URI=http://localhost:11310 python3 verification/ego_smoke/reciprocal_15sitl.py \
  --uav-a UAV1 --index-a 1 --uav-b UAV2 --index-b 2 --no-require-rebound \
  --execute --confirm-sitl I_UNDERSTAND_SITL
```

### 阶段 7：清理与生命周期收口（**强制**）
```bash
bash src/swarm_simulation/bashs/cleanup.sh   # 逆序停止 TCP→GCS→onboard→sim + 残留检查（后文 “cleanup.sh” 均指本正式版）
# 断言：gazebo/px4/rosmaster/mavros 进程数 = 0；测试端口（39001/11300/11310..11325）全部释放
```
- **测试结束（无论 PASS / FAIL / 超时）必须立即执行本阶段**；
- **测试完成后，如无需在线收集信息，先停止全部仿真进程，再进行数据分析**：
  数据分析（`.tmp/plots/analyze_latest_flight.py` ULog 绘图、planner/console 日志分析、
  离线回放 `replay_intent_consistency.py` 等）**全部读取落盘文件**（ULog/日志/JSON），
  **不依赖仿真进程运行**；而仿真进程持续占用 CPU/内存且可能覆盖/追加日志，干扰分析。
  仅当需要**在线收集**（实时订阅话题、录制 rosbag、观察实时 exec_state/机间距离）时才保持进程。
- **禁止任何"就地恢复 / 单机重启 / 二次启动"动作**——需要重跑一律先 cleanup 断言无残留，
  再回到阶段 0 整体干净重启（`bash src/swarm_simulation/bashs/round_prepare_15sitl.sh --round N`）。

## 3. 套壳脚本（从仿真层到软起动）

### 3.1 `round_prepare_15sitl.sh`（正式版单轮整套，推荐）

```bash
bash src/swarm_simulation/bashs/round_prepare_15sitl.sh --round N [--task prepare] [--timeout-s 150] [--keep-ros-home-on-fail 1]
# 退出码：0=task SUCCESS；1=失败（--keep-ros-home-on-fail 1 时已把 .ros_home 留档 .ros_home_roundN）；2=用法错
```

- 自动执行 **阶段 0→7**：clean → `.ros_home` 隔离 → sim(15 px4) → onboard(15/15 ready) →
  gcs bridge→backend + 标定（`/group_a/calibration_ready=true`）→ tcp(39001) → takeoff(15/15) →
  task(prepare, timeout 150s) → RESULT →（失败留档）→ 无条件 cleanup；
- 每阶段轮询就绪，任一阶段失败即退出非零并 cleanup；
- **失败后整体干净重启**（脚本自带 cleanup，断言无残留后再跑），**不就地重试 offboard、
  不单机/子集重启机载层**；
- 成功即退出码 0，失败轮飞行记录自动保留在 `.ros_home_roundN`。

### 3.2 `phases.sh`（分阶段控制，正式版）

```bash
bash src/swarm_simulation/bashs/phases.sh <phase>
# phase: sim | onboard | gcs_bridge_only | gcs_backend_only | tcp | takeoff | task [name] | status | cleanup | uav_z <idx>
```

- `gcs` 组合**已禁用**（两个 roslaunch 同时 auto-start master 11310 会竞争 XML-RPC，backend 冲突
  退出；一律 `gcs_bridge_only` → 就绪 → `gcs_backend_only` 分步）；
- 后台进程 pid 写入 `$HLOG/*.pid`（`.ros_home/log/headless_test`），供 `cleanup.sh` 精确逆序停止；
- 旧 `.tmp/headless_test/run.sh` 即本脚本的历史来源（08-30 后不再使用；2026-09-10 归档至
  `.tmp/archive_260910/headless_test/`）。

### 3.3 脚本速查（启动/编排一律用正式入库版）

| 脚本 | 用途 |
|---|---|
| `src/swarm_simulation/bashs/round_prepare_15sitl.sh` | **单轮整套（正式，推荐）**：clean→`.ros_home` 隔离→sim 15/15→onboard 15/15→gcs(bridge→backend)+标定→tcp 39001→takeoff 15/15→task prepare→RESULT→失败留档 `.ros_home_roundN`→cleanup；退出码 0=SUCCESS/1=FAIL/2=用法错 |
| `src/swarm_simulation/bashs/phases.sh` | 分阶段控制：sim/onboard/gcs_bridge_only/gcs_backend_only/tcp/takeoff/task/status/cleanup/uav_z |
| `src/swarm_simulation/bashs/cleanup.sh` | 逆序停止全部测试进程 + 端口/进程残留检查 |
| `src/swarm_simulation/bashs/status.sh` | 各阶段就绪状态检查（px4/master/服务/高度；**px4 就绪用进程数，不用 rosnode list**） |
| `src/swarm_simulation/bashs/env.sh` | 公共环境（`ROS_HOME`/`HLOG`/PX4/Gazebo 路径；日志 → `.ros_home/log/headless_test`） |
| `.tmp/scripts/run_5_prepare_matrix_260909b.sh` | 5 轮串联 runner（包装正式单轮；轮间 30s 静默 + **强杀 tcp_server 39001 残留**——bashs cleanup 对该进程有已知缺口，仍必需） |
| `.tmp/scripts/return_land_all.py` | RTL 落地全部测试机（任务后收尾；无正式入库版，仍可用） |
| `verification/ego_smoke/reciprocal_15sitl.py` | 双机正交避碰编排（预检 → staging → 屏障 START → 采集 → HOLD；属 verification） |
| `src/swarm_uav_executor/scripts/startup_offboard_ego.sh` | 逐机机载层启动/停止（`stop 1..15` 逆序停；`phases.sh onboard` 内部调用，默认 `EGO_SAFETY_SUPERVISOR_MODE=active`） |

> 以下 `.tmp` 脚本为**历史版本（已被 swarm_simulation/bashs 取代/停用；2026-09-10 已归档至
> `.tmp/archive_260910/`，勿再使用）**：`startup_ego_sitl.sh`（阶段 0→5 一键，末次使用
> 08-27/28）、`run_15uav_prepare.sh`（15-UAV prepare 旧编排，09-07 后被 round 脚本取代）、
> `run_headless_prepare_260824{,15}.sh`（08-24 旧式，仅历史参考）；`.tmp/headless_test/
> {run,cleanup,status,env}.sh` 为上述脚本的历史来源。
> `.tmp/plots/` 与 `.tmp/scripts/replay_intent_consistency.py` 等为**分析/绘图工具**（无正式
> 入库版、仍在用），不属于启动链，不随上表历史脚本清理。

## 4. 注意事项

1. **`.ros_home` 清空**（保留文件夹），避免旧日志/pid 混淆；测试后保留证据目录（`prepare_*.bag`/ULog/图）。
2. **标定先于 takeoff**：gp_origin 必须在 disarm 时冻结（armed 被拒）；坐标系未统一前
   MAVROS local 各自为原点，机间距离会虚近。
3. **GCS bridge 先启动再 backend**：两者竞争 11310 端口；先 bridge（等 `/UAV1/uav_task` 服务），
   再 backend，避免 bridge 起不来导致服务代理缺失。
4. **失败后整体干净重启，绝不就地恢复**：不重试 offboard、**不单机/子集重启机载层**；
   先 `cleanup.sh` 断言无残留，再 `round_prepare_15sitl.sh --round N` 从阶段 0 整体重来。
5. **机载层统一 active 一次性启动**：不再区分 shadow/active、不再"active 重启测试机"；
   `round_prepare_15sitl.sh` / 阶段 2（`phases.sh onboard`）以 `EGO_SAFETY_SUPERVISOR_MODE=active`
   （机载脚本默认）一次性启动全部 15 机，任一机 ready 缺失即判定本次启动失败（整体重来），不补启。
6. **无头环境变量**：`LIBGL_ALWAYS_SOFTWARE=1`（env.sh 已设置）；机载层日志重定向
   `ROS_HOME=$WS/.ros_home`。
7. **残留检查**：每次测试结束断言 gazebo/px4/rosmaster/mavros 进程 0、测试端口
   （39001/11300/11310..11325）全部释放。
8. **任务前坐标复核**：12 机 prepare 前确认 GCS 统一 local 坐标 + 实测初始间距；
   历史近距异常（0.47–0.85m）优先判定为原点/启动流程故障并拒绝发任务。

## 5. 测试生命周期铁律（agent 必读）

> 目的：杜绝"测试结束后 agent 尝试再次启动/重启/恢复进程"导致状态异常、节点意外退出。
> **agent 只允许在 `round_prepare_15sitl.sh`（`src/swarm_simulation/bashs/`）的一次性启动流程内
> （或按 §3.2 `phases.sh` 顺序分步）启动进程；启动流程之外的一切
> 启动/重启/就绪轮询动作一律禁止。**

1. **测试结束即收口**：无论 PASS / FAIL / 超时，测试脚本返回后**立即**执行
   `cleanup.sh` 停止全部进程并断言无残留；不得延迟、不得等待。
2. **结束后禁止自动重启**：cleanup 之后 agent **不得**自动再启动仿真/机载层/GCS/TCP 中任何
   组件；不得对单机做"就绪检查 → 补启/重启"（如 `startup_offboard_ego.sh <idx>` 单独重启）。
3. **重跑必须整体重来**：需要重跑测试时，先确认无残留（`cleanup.sh` + 进程/端口断言），
   再以 `round_prepare_15sitl.sh --round N` 从阶段 0 整体重来；**禁止在部分进程存活的环境上追加启动**。
4. **就地恢复即违规**：测试中某机载层节点意外退出时，**不得**就地重启该机；先整体
   `cleanup.sh`，将退出现象与日志记入测试记录，再从干净环境重跑。
5. **单机故障判定**：测试中某机异常（如 EGO_PLAN_FAILED / MIN_DISTANCE_BREACH / 节点退出），
   属于被测对象行为，**先记录、再整体收口**；禁止"修好它继续跑"。

## 6. 机载层节点意外退出（已知风险与处理）

- **现象**（26082700 实测 UAV1/UAV8）：某机 `uav_executor_UAVn` 收到 SIGINT 退出 →
  roslaunch `required=true` 连带关闭整个机载层 launch（MAVROS/driver/bridge 全退）→
  offboard loss → PX4 自动 LAND。
- **诱因**：GCS backend 经 swarm_topology_bridge 转发 service（`/UAVn/uav_hold` 等）偶发
  超时（bridge service 转发脆弱，高并发时更明显）；极端情况下 executor 异常退出。
- **预防**：
  1. 机载层统一 active 一次性启动（消除 shadow→active 重启引入的竞态/时序窗口）；
  2. 启动后固定轮询 15/15 `ready` 再进入下一步，不抢跑；
  3. 任务中不做任何机载层层面的操作（重启/参数注入/单机命令）。
- **处理**：出现即整体 `cleanup.sh` 收口 → 记录退出日志（`.ros_home/log/<会话>/uav_executor_UAVn-*.log`、
  `rosout.log`）→ 从干净环境重跑；**绝不就地重启该机**。

## 7. 数据分析原则（先停止仿真，再做分析）

> **测试完成后，如果不需要在线收集信息，先执行 `cleanup.sh` 停止全部仿真进程，再进行数据分析。**

### 7.1 为什么必须先停仿真

- 数据分析全部读取**落盘文件**（ULog、planner/console 日志、JSON/CSV），**不依赖进程运行**：
  - ULog 绘图：`.tmp/plots/analyze_latest_flight.py --goal-epoch <goal> --task-ticks <ticks>`
  - 离线回放：`.tmp/scripts/replay_intent_consistency.py --pair UAVn,UAVm --start <t> --end <t>`
  - 日志/metrics 分析：`runtime_logs/ego_planner/`、`.ros_home/log/headless_test/`、`.tmp/plots/*.json`
- 仿真进程持续占用 CPU/内存（15 机 SITL 达数十进程），且可能**继续追加/覆盖日志与 ULog**，
  干扰对"本次测试结果"的分析；保持运行只会拖慢分析并污染证据。

### 7.2 何时需要在线收集（才保持进程）

仅以下场景需要仿真进程存活：

- 实时订阅话题（`rostopic echo` exec_state / 机间距离 / setpoint 频率等）；
- 录制 rosbag（`rosbag record` 在线话题）；
- 需要二次观测/人工 QGC 观察实时飞行。

在线收集完成后，同样立即 `cleanup.sh` 收口，再基于收集到的落盘数据离线分析。

### 7.3 分析命令模板（cleanup 之后）

```bash
# 1. ULog 轨迹/速度/机间距绘图（goal-epoch 取自 backend 日志 "GCS_A goal ..."，ticks 取自 07_task_*.log）
python3 .tmp/plots/analyze_latest_flight.py --goal-epoch <goal_epoch> --task-ticks <ticks>

# 2. 机对回放（预测 vs 实际偏差、self_odom_dev、traj churn）
python3 .tmp/scripts/replay_intent_consistency.py --pair UAV5,UAV6 --start <t0> --end <t1>

# 3. 结构化事件/EMERGENCY 统计
grep -c 'level=EMERGENCY' runtime_logs/ego_planner/UAV*.log
```

## 8. 5 轮测试日志留档规则（2026-09-10 起）

> 适用于 5 轮 prepare 复测（`.tmp/scripts/run_5_prepare_matrix_260909b.sh` 或人工逐轮
> `src/swarm_simulation/bashs/round_prepare_15sitl.sh`）。历史旧命名 `.ros_home_roundN`
> 不再使用。

1. **每次跑 5 轮测试必须保存所有轮次的日志**（含 SUCCESS 轮），不得只留失败轮。
2. **每轮日志目录命名**：`.ros_home_round<n>_<s|f>`
   - `<n>` = 轮次编号（见第 3 条计数规则）；
   - `<s|f>` = 按本轮结果取一个字母：`SUCCESS` → `s`，`FAILURE`/`UNKNOWN` → `f`；
   - 例：第 1 轮 SUCCESS → `.ros_home_round1_s`；第 2 轮 FAILURE → `.ros_home_round2_f`。
3. **轮次计数只计入实际运行了 prepare 任务的轮次**：
   - 启动阶段即失败的轮次（sim/onboard/GCS/标定/tcp/takeoff 任一未就绪，如 tcp_server 39001
     端口残留导致新 server bind 失败等 infra 原因，prepare 任务未真正派发执行）**不计入轮次计数**，
     不占用 round 编号、不产生 `<s|f>` 目录；
   - 该次重跑沿用当前轮次编号，并在测试记录中注明“启动失败不计轮次，重跑”及失败原因。
4. 目录内容沿用失败留档惯例（整份 `.ros_home` 副本：sitl_iris_* ulog/机载 rosout/GCS 日志/
   07_task JSON），SUCCESS 轮同样落一份，便于回放比对与绘图证据复现。

