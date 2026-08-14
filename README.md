# catkin_swarm6-2 多无人机集群项目（Group A）说明与交接文档

> 本文档是整个工作空间的**总说明 + 交接给下一个 coding agent 的 handoff**。
> 它不是单包 README，而是 `catkin_swarm6-2` 工作空间的入口：说明项目目标、架构、
> 当前实现进度、测试步骤，以及换设备/换 workspace 编译运行时的注意点。
> 单包细节与历史过程见文末「文档索引」。

---

## 1. 项目概述

多无人机集群联合任务系统（至少 30 机，A/B 两组，湖南大学科技园 100m×150m 场地）。
本工作空间负责 **Group A 侦查集群**的地面站（GCS_A）与机载执行链路：

```text
references/runtime（行为树 / TCP 客户端，只读参考）
        │ TCP JSON Lines (39001)
        ▼
tcp_to_ros      GCS_A 地面站：TCP server → ExecuteGroupTask Action → backend 整组分发
        │
swarm_topology_bridge  跨 ROS Master 的 Topic 桥 + Service 代理（每台机各跑一个实例）
        │
swarm_uav_executor     每架 UAVn 的任务执行器（PREPARE/START/ABORT/HOLD + 状态上报）
        │
        ├─ ego_planner_driver    C++ B-spline 带时轨迹 + 机间互惠避碰（30Hz setpoint）
        └─ MAVROS / PX4 SITL     飞控（仿真或实机）
```

**坐标约定**：任务坐标按右/前/上解释；GCS_A 用 A01-A06 arm 前全球位置中位数冻结全局原点，
参考航向 = A01-A06 的圆周平均 yaw（30° 门禁）；A01-A15 全部回读确认 `gp_origin` 后才开放任务。

---

## 2. 目录结构

```text
catkin_swarm6-2/
├── src/                    开发代码（6 个独立 git 仓库，不 push）
│   ├── tcp_to_ros/             GCS_A 地面站主包（本文档所在包）
│   ├── swarm_uav_interfaces/   共享 ROS msg/srv 契约（所有机必须同版本）
│   ├── swarm_topology_bridge/  通用跨 Master 桥（Topic + Service 代理）
│   ├── swarm_uav_executor/     机载任务执行器 + 安全 HOLD
│   ├── ego_planner_driver/     C++ ego-swarm 风格轨迹驱动（B-spline + L-BFGS + 互惠避碰）
│   └── safe_valley_exp/        应用示例 + 15 机仿真 launch + 起飞脚本
├── references/             只读参考资料（runtime 行为树、docs 方案、ego-planner-swarm 原版）
├── verification/           工程验证脚本（不属于任何 git 仓库；ego_smoke、acceptance、plot 脚本）
├── .ros_home/              统一 ROS 日志目录（避免散落 ~/.ros）
├── build/ devel/ logs/     catkin 构建产物
└── docs 相关              （见 src/tcp_to_ros/docs 与 plans）
```

**重要**：`references/` 是参考而不是开发文件夹，改动一律不入 git；`verification/` 是工程验证脚本，
不属于任何 `src/*` 仓库；开发只改 `src/` 下六个仓库。

---

## 3. 当前实现进度（2026-08-14）

### 已实现并验证

| 能力 | 状态 | 说明 |
|---|---|---|
| 标定 | ✅ | A01-A06 采样冻结原点/航向，A01-A15 原点回读 15/15；**必须 disarm 时完成**（armed 时 `set_gp_origin` 被拒） |
| PREPARE/START/ABORT | ✅ | backend 整批并发确认，内部两阶段，runtime `timeout_s` 作整批绝对期限 |
| 分层动作 | ✅ | MOVE_TO 先 5m 检查点→15m 层→水平 EGO→目标 z；FOLLOW_ROUTE 12m；FAULT_EXIT 8m |
| 固定编队 | ✅ | coverage-1 = A01-A12（3×4），coverage-2 = A01-A06+A13-A15（3×3）；集合不匹配拒绝 |
| 按需起飞 | ✅ | takeoff 脚本：注入 RC 参数 → 切 HOLD/AUTO.LOITER → arm → OFFBOARD → ego 软起飞 5m → HOLD；15/15 成功 |
| link_health + safe_hold | ✅ | 替代已删除的 lease 机制：节点丢失 → backend 判定 LOST → 全悬停 → 手动接管 |
| 飞行安全租约（lease） | ❌ 已删除 | 见 plans/implementation_plan_26081406.md；时延敏感（renew 0.9s）+ 锁存难恢复，仿真与自组网实机都不可用 |

### 关键 bug 修复（本会话）

1. **hold 捕获与待规划 goal 竞态**（ego_planner_driver `192ad03`）：空闲 HOLD 下 `publishSetpoint` 在
   `onGoal` 清除 `steady_target_` 后立即重新捕获，导致 `replanLoop` 因 `steady_target_` 非空永不规划，
   新 MOVE_TO 卡 HOLD 至超时（曾只有 1/15 机靠时序幸运执行）。修复：待规划时不捕获。
2. **emergency_hold 崩溃**（tcp_to_ros `09a9e44`）：`cancel_goal_at_server` 是 client API，server 无此方法，
   超时/取消路径崩溃。修复：设置 `stop_event` 让执行线程自行发布终态。

### 未完成 / 待办

- prepare 六批次端到端重跑（竞态修复后尚未重跑；此前因竞态 TIMEOUT）
- 断 bridge → link_health LOST → safe_hold 全悬停的故障注入验证
- Phase 6 实机不上桨 / 上桨门禁（现场参数与 failsafe 冻结）

---

## 4. 测试步骤

最新实测手册（含步骤-预期-结果、可复制命令）见：

```text
src/tcp_to_ros/docs/test_record_26081402.md
```

核心流程（简化）：

```text
1. 构建：catkin build（6 包全过）
2. 启动仿真层：multi_uav_ego_15sim.launch（11300/11345，15 架 iris）
3. 启动 15 机载层：startup_offboard_ego.sh 1..15（11311..11325）
4. GCS_A：topology_group_a_sim.launch uav_name:=GCS_A（11310）
5. backend+标定：gcs_a_backend.launch（须 disarm，15/15 ready）
6. TCP server：tcp_server_node.py（39001）
7. 起飞：offboard_takeoff_15.py 1..15（先 HOLD → arm → OFFBOARD → 软起飞 5m）
8. 批次：task_test.py --backend tcp --group A --task <TASK> --timeout-s <T>
9. 清理：逆序停止，勿用宽泛 killall/pkill
```

批次 timeout 建议（1.0m/s）：prepare 150s；coverage-segment-1/2 480s；fault-exit 240s；recovery 240s；hold-end 150s。

---

## 5. 编译与运行（含换设备/换 workspace 注意）

### 5.1 前置

```bash
cd /home/ub20tg/catkin_swarm6-2   # 其他设备请替换为实际路径
source /opt/ros/noetic/setup.bash
catkin build                       # 6 包全量构建
source devel/setup.bash
# PX4/Gazebo（仿真）
source /home/ub20tg/PX4_Firmware/Tools/setup_gazebo.bash \
  /home/ub20tg/PX4_Firmware /home/ub20tg/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:/home/ub20tg/PX4_Firmware:/home/ub20tg/PX4_Firmware/Tools/sitl_gazebo"
# 统一 ROS 日志
mkdir -p /home/ub20tg/catkin_swarm6-2/.ros_home/log
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
```

### 5.2 换设备 / 新 workspace 必须注意

1. **6 个独立 git 仓库**：`src/` 下每包各自 `.git`，分别 commit，**不 push**；不把工作空间根建为 git 仓库。
2. **`references/` 与 `verification/`**：只读参考 / 工程验证，不是交付物，不入任何 git 仓库。
3. **`swarm_uav_interfaces` 版本一致**：所有 GCS_A、bridge、UAVn 主机必须安装、构建、source 完全相同的接口 commit，否则 ROS 类型 MD5 不匹配。
4. **端口约定**（多 Master 隔离）：仿真 11300、GCS_A 11310、UAVn = 11310+n（11311..11325）、Gazebo 11345、TCP 39001；PX4 SITL 仿真 TCP 4560..4574、MAVLink UDP 34580..34594。换环境需核对，尤其实机无独立 Master 端口时。
5. **无 RC SITL 参数**：起飞脚本自动注入 `COM_RCL_EXCEPT=4`、`NAV_RCL_ACT=0`；`COM_RC_IN_MODE` 默认 2（RC 全模式启用），无虚拟摇杆且处于手动模式（POSCTL）arm 会被拒（`manual control lost`）——takeoff 脚本已通过**先切 HOLD/AUTO.LOITER（Auto 模式免 RC）**规避。实机建议保留 RC 仅应急接管，`COM_RC_IN_MODE` 保持默认。
6. **标定需 disarm**：gcs_a_calibration 要求 A01-A06 全 disarm + 数据连续 2s + 航向 30° 门禁；armed 时 `gp_origin_receiver` 拒绝改原点。
7. **backend 单实例**：重复启动 `gcs_a_backend.launch` 会因 `gcs_a_calibration` 节点重名冲突，先停旧实例。
8. **executor `required=true`**：kill executor 会连带关闭整个机载层 launch（MAVROS/driver/bridge）→ offboard loss → LAND。
9. **C++ 节点空闲态是 HOLD（无 IDLE）**：exec_state 初始/空闲为 `HOLD`；文档/脚本里旧 `IDLE` 表述是历史遗留。
10. **日志**：统一写 `.ros_home/`；跨 Master 问题优先看 bridge 与对应 UAV 的 `uav_executor_UAVn-*.log`。
11. **工具会话超时**：30s 工具限制会杀前台长进程；启动仿真/机载层/批次用 `setsid nohup ... > log 2>&1 < /dev/null &` 后台运行。

### 5.3 最小部署包集合

```text
GCS_A:     tcp_to_ros + swarm_topology_bridge + swarm_uav_interfaces
每台 UAVn: swarm_uav_executor + swarm_topology_bridge + swarm_uav_interfaces
仿真层:    safe_valley_exp（launch + 起飞脚本）+ PX4/Gazebo
```

`safe_valley_exp` 不属于 GCS_A 必需部署；机载执行能力由 `swarm_uav_executor` + `ego_planner_driver` 提供。

---

## 6. 关键设计决策（给后续开发）

- **删除飞行安全租约（lease）**：原机制（renew 1Hz/TTL 5s/锁存）在仿真高负载与自组网电台（200-800ms 抖动）下必然失败；
  改为 `link_health`（MAVROS state 心跳 → DISCOVERED/HEALTHY/SUSPECT/LOST/RECOVERED）+ `safe_hold`。
  断链兜底：节点丢失 → backend LOST → 参与机全 HOLD → 手动接管。`UavSafetyLease.srv` 保留为 dead definition 不删。
- **C++ 主状态 = HOLD**：空闲悬停用 `steady_target_` 冻结位姿，不再每秒刷新（避免漂移）；`POSE_STALE` 仅保护规划输入。
- **分层轨迹**：非 follower 全部走 ego 带时轨迹；follower 用 PI 位置 + 固定槽位偏置；垂直段冻结 x/y/yaw。
- **无 RC 起飞顺序**：切 Auto 模式（HOLD/LOITER）→ arm → OFFBOARD，避免 PX4 手动模式 RC 检查。

---

## 7. 已知问题与安全边界

- `POSE_STALE` 只表示规划输入过期，不等于飞机停止；执行中位姿失联的最终兜底仍依赖 PX4 failsafe（`COM_OBL_ACT` 等）。
- 软件 HOLD 不替代 PX4 datalink/offboard/RC failsafe；实机前必须冻结 `COM_OF_LOSS_T`、`COM_OBL_ACT`、RC/datalink failsafe、急停职责、场地许可。
- 批次任务仍在验证中（prepare 竞态已修，未重跑）；勿以本 README 状态代替实测记录。
- 任何异常优先人工 HOLD/降落/急停；清理用精确 PID，勿用宽泛 `killall`/`pkill -f`。

---

## 8. 给下一个 coding agent 的快速指引

1. 先读 `plans/README.md`（Phase 编号 + 设计规约）与 `docs/test_record_26081402.md`（最新实测手册）。
2. 当前主任务：**重跑 prepare 六批次**（竞态修复 `192ad03` 后），随后做断 bridge → link_health LOST → safe_hold 故障注入验证。
3. 改代码只动 `src/` 六个仓库；每包独立 commit（Conventional Commit，英文 header + 中文 body），**不 push**。
4. 提交前 `catkin build` + 相关单测；测试记录写入 `docs/test_record_2608xxxx.md`（文件名后缀 = 2 位年月日时）。
5. 环境清理、端口、日志、无 RC 参数等注意点见本文档第 5 节。

---

## 9. 文档索引

| 路径 | 内容 |
|---|---|
| `src/tcp_to_ros/plans/README.md` | Phase 编号、全局设计规约、实施计划索引、里程碑、当前状态 |
| `src/tcp_to_ros/plans/implementation_plan_26081406.md` | **lease 删除**实施计划（方案 A） |
| `src/tcp_to_ros/docs/test_record_26081402.md` | 最新 15 机实测手册（步骤-预期-结果，可复制命令） |
| `src/tcp_to_ros/docs/tcp_to_ros_工程过程.md` | 开发日志（阶段、bug、决策） |
| `src/tcp_to_ros/docs/test_record_26081300.md` | 冻结坐标/按需起飞/分层轨迹验证记录 |
| `src/tcp_to_ros/README.md` | 本文件（项目总说明 + handoff） |
| `src/ego_planner_driver/README.md` | ego 轨迹驱动接口与安全说明 |
| `src/swarm_uav_executor/README.md` | 机载执行器接口与安全说明 |
| `src/swarm_topology_bridge/README.md` | 跨 Master 桥说明 |
| `references/` | 只读参考（runtime、方案文档、ego-planner-swarm 原版） |
| `verification/` | 工程验证脚本（ego_smoke、acceptance、plot 脚本，不入 git） |
