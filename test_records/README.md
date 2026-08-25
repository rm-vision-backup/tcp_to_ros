# 测试记录目录（test_records）

> 本目录归档无头/有头测试的证据记录（按日期命名 `test_record_2608xxxx.md`），
> 对应 `plans/implementation_plan_2608xxxx.md` 实施计划。
> 启动流程与注意事项见本 README；`plans/` 为实施计划入口。

## 1. 无头 vs 有头测试

| 维度 | 无头（headless） | 有头（GUI） |
|---|---|---|
| 仿真层 | `gui:=false`（`LIBGL_ALWAYS_SOFTWARE=1`） | `gui:=true`（Gazebo 窗口 + QGC 观察） |
| 用途 | **自主测试 / 回归**（脚本化、可批量、无人值守） | 人工诊断 / 现象观察（如 QGC 看机间画面） |
| 依赖 | 无显示依赖、资源省 | 需显示环境 |
| 注意 | 无头前清空 `.ros_home`（保留文件夹） | 有头与无头启动链相同，仅 `gui` 参数不同 |

**约定：自主测试一律使用无头**（`multi_uav_ego_15sim.launch gui:=false`）——避免 GUI 依赖与人工
操作，便于连续回归；**有头仅在需要 QGC/可视化人工观察或排障时使用**，两者启动链一致。

## 2. 完整启动流程（无头/有头共通，阶段 0→8）

> 推荐用 §3.1 `startup_ego_sitl.sh` 一键执行阶段 0→6；以下为各阶段人工执行的等价步骤。

### 阶段 0：预检
```bash
# 无残留仿真进程（Gazebo/PX4/ROS master/MAVROS 均为 0）
ps aux | grep -cE '[g]azebo|[p]x4|[r]osmaster|[m]avros'   # 期望 0
# 清空 .ros_home（保留文件夹），避免旧日志/pid 混淆
rm -rf .ros_home && mkdir -p .ros_home/log
```

### 阶段 1：仿真层（15 机 PX4 SITL）
```bash
bash .tmp/headless_test/run.sh sim          # gui:=false, master 11300, gazebo 11345
# 等待：15 个 px4 进程 + 日志 Spawn status = 15
```

### 阶段 2：机载层（15 机独立 Master）
```bash
bash .tmp/headless_test/run.sh onboard      # 等价 startup_offboard_ego.sh 1..15
# 等待：02_onboard.log 中 "ready UAV" 达 15（11311..11325）
```

### 阶段 3：GCS（bridge → backend → tcp，顺序避免 11310 竞态）
```bash
bash .tmp/headless_test/run.sh gcs_bridge_only   # 先启动 bridge，等 /UAV1/uav_task 服务出现
bash .tmp/headless_test/run.sh gcs_backend_only  # 再启动 backend
bash .tmp/headless_test/run.sh tcp                # tcp_server_node 39001
```

### 阶段 4：标定（**必须在 takeoff 前，且各机 disarm**）
```bash
# 等 /group_a/calibration_ready = True（GCS 冻结 gp_origin 统一坐标系）
rostopic echo -n 1 /group_a/calibration_ready   # 期望 data: True
# 每机 gp_origin_confirmed 置位（坐标系统一后 local 距离 = 世界距离）
```
> 标定要求 A01-A06 全 disarm + 数据连续 2s + 航向 ≤30°；**armed 时 set_gp_origin 被拒**，
> 因此必须先标定再 takeoff。

### 阶段 5：active 重启测试机（仅避碰专项需要；shadow 可跳过）
```bash
EGO_SAFETY_SUPERVISOR_MODE=active \
  bash src/safe_valley_exp/startup_offboard_ego.sh 1 2    # 示例：测试机 1、2
# 等待测试机 ready；重启后坐标系保持统一（gp_origin latch）
```

### 阶段 6：起飞
```bash
python3 src/safe_valley_exp/scripts/offboard_takeoff_15.py 1..15
# HOLD → arm → OFFBOARD；软起飞 z≈5m 由 ego TAKEOFF 状态自动完成
# 等待各机高度 >3m
```

### 阶段 7：测试
```bash
# 批次任务（GCS TCP 全链路）
python3 references/runtime/task_test.py --backend tcp --group A --task prepare --timeout-s 180
# 双机正交避碰（直接调 executor 服务）
ROS_MASTER_URI=http://localhost:11310 python3 verification/ego_smoke/reciprocal_15sitl.py \
  --uav-a UAV1 --index-a 1 --uav-b UAV2 --index-b 2 --no-require-rebound \
  --execute --confirm-sitl I_UNDERSTAND_SITL
```

### 阶段 8：清理
```bash
bash .tmp/headless_test/cleanup.sh           # 逆序停止 TCP→GCS→onboard→sim + 残留检查
# 断言：gazebo/px4/rosmaster/mavros 进程数 = 0
```

## 3. 套壳脚本（从仿真层到软起动）

### 3.1 `startup_ego_sitl.sh`（一键，推荐）
```bash
bash .tmp/scripts/startup_ego_sitl.sh [UAV_LIST] [SAFETY_MODE]
# 默认 UAV_LIST="1 2"、SAFETY_MODE=active
```
- 自动执行 **阶段 0→6**：预检 → sim → onboard → gcs(bridge→backend) → 标定 →
  active 重启测试机 → takeoff → 各机高度确认；
- 每步轮询就绪（`wait_until`），任一步失败即退出非零；
- **失败后整体干净重启**（先 `cleanup.sh` 再重跑脚本），**不就地重试 offboard**；
- 成功输出 READY + reciprocal 执行命令。

### 3.2 `headless_test/run.sh`（分阶段编排）
```bash
bash .tmp/headless_test/run.sh <phase>
# phase: sim | onboard | gcs | tcp | takeoff | task [name] | status | cleanup | uav_z <idx>
```
- `gcs` 一次性启动 bridge+backend（有 11310 竞态风险，推荐用 `gcs_bridge_only`/`gcs_backend_only` 分步）；
- 所有后台进程 pid 写入 `$HLOG/*.pid`，供 `cleanup.sh` 精确逆序停止。

### 3.3 其他辅助
| 脚本 | 用途 |
|---|---|
| `.tmp/headless_test/cleanup.sh` | 逆序停止全部测试进程 + 端口/进程残留检查 |
| `.tmp/headless_test/status.sh` | 各阶段就绪状态检查（px4/master/服务/高度） |
| `.tmp/scripts/return_land_all.py` | RTL 落地全部测试机（任务后收尾） |
| `.tmp/scripts/run_headless_prepare_26082415.sh` | 旧式 prepare 编排（逐机独立启动 + 就绪补充轮） |
| `verification/ego_smoke/reciprocal_15sitl.py` | 双机正交避碰编排（预检 → staging → 屏障 START → 采集 → HOLD） |
| `src/safe_valley_exp/startup_offboard_ego.sh` | 逐机机载层启动/停止（`stop 1..15` 逆序停） |

## 4. 注意事项

1. **`.ros_home` 清空**（保留文件夹），避免旧日志/pid 混淆；测试后保留证据目录（`prepare_*.bag`/ULog/图）。
2. **标定先于 takeoff**：gp_origin 必须在 disarm 时冻结（armed 被拒）；坐标系未统一前
   MAVROS local 各自为原点，机间距离会虚近。
3. **GCS bridge 先启动再 backend**：两者竞争 11310 端口；先 bridge（等 `/UAV1/uav_task` 服务），
   再 backend，避免 bridge 起不来导致服务代理缺失。
4. **失败后整体干净重启**：不就地重试 offboard；先 `cleanup.sh` 再 `startup_ego_sitl.sh`。
5. **active 模式仅用于避碰专项测试机**：生产/常规任务保持 shadow；active 冻结期遵循
   `plans/implementation_plan_26082500.md` 缺陷清单门禁。
6. **无头环境变量**：`LIBGL_ALWAYS_SOFTWARE=1`（env.sh 已设置）；机载层日志重定向
   `ROS_HOME=$WS/.ros_home`。
7. **残留检查**：每次测试结束断言 gazebo/px4/rosmaster/mavros 进程 0、测试端口
   （39001/11300/11310..11325）全部释放。
8. **任务前坐标复核**：12 机 prepare 前确认 GCS 统一 local 坐标 + 实测初始间距；
   历史近距异常（0.47–0.85m）优先判定为原点/启动流程故障并拒绝发任务。
