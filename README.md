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
        ├─ ego_planner_driver    C++ B-spline 带时轨迹 + 机间互惠避碰（30Hz PositionTarget 候选）
        └─ setpoint_relay        MAVROS `/mavros/setpoint_raw/local` 唯一发布者（仲裁/失效 HOLD）→ PX4
```

**坐标约定**：任务坐标按右/前/上解释；GCS_A 用 A01-A06（标定计算集）arm 前全球位置中位数冻结
全局原点，参考航向 = A01-A06 的圆周平均 yaw（30° 门禁），`mission_anchor=(0,0,0)`；冻结结果下发
A01-A15，15 架全部回读确认 `gp_origin` 后才开放任务。A01-A12 仍是 formation-1/prepare/coverage 成员集，
不得与标定计算集混淆。

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
└── docs 相关              （见 src/tcp_to_ros/test_records 与 plans）
```

**重要**：`references/` 是参考而不是开发文件夹，改动一律不入 git；`verification/` 是工程验证脚本，
不属于任何 `src/*` 仓库；开发只改 `src/` 下六个仓库。

---

## 3. 当前实现进度（历史里程碑）

> 详细实时状态以 `plans/README.md`「当前状态」为唯一权威；以下为本工作空间的早期里程碑摘要，
> 具体演进见 `plans/README.md` 与 `test_records/`。

### 已实现并验证

| 能力 | 状态 | 说明 |
|---|---|---|
| 标定 | ✅ | A01-A06 采样冻结原点/航向，A01-A15 原点回读 15/15；**必须 disarm 时完成**（armed 时 `set_gp_origin` 被拒） |
| PREPARE/START/ABORT | ✅ | backend 整批并发确认，内部两阶段，runtime `timeout_s` 作整批绝对期限 |
| 分层动作 | ✅ | planner TAKEOFF 软起飞至 5m 后 HOLD；任务 MOVE_TO 再进 15m 层→水平 EGO→目标 z；FOLLOW_ROUTE 12m；FAULT_EXIT 8m |
| 固定编队 | ✅ | coverage-1 = A01-A12（3×4），coverage-2 = A01-A06+A13-A15（3×3）；集合不匹配拒绝 |
| 起飞准备 | ✅ | 标定完成后，外部 `offboard_takeoff_15.py`：HOLD（失败回退 AUTO.LOITER）→ arm → OFFBOARD；planner TAKEOFF 软起飞 5m → HOLD；随后 executor PREPARE 只检查 armed/OFFBOARD/origin，GCS START 只启动已登记任务；15/15 成功 |
| link_health + safe_hold | ✅ | 替代已删除的 lease 机制：节点丢失 → backend 判定 LOST → 全悬停 → 手动接管 |
| 飞行安全租约（lease） | ❌ 已删除 | 时延敏感（renew 0.9s）+ 锁存难恢复，仿真与自组网实机都不可用；决策见 plans/README.md |

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

最新实测记录（含步骤-预期-结果、可复制命令）见：

```text
src/tcp_to_ros/test_records/test_record_26082400.md   （PVA 链路验收）
src/tcp_to_ros/test_records/test_record_26082300.md   （方案 B prepare）
```

核心流程（简化）：

```text
1. 构建：catkin build（6 包全过）
2. 启动仿真层：multi_uav_ego_15sim.launch（11300/11345，15 架 iris）
3. 启动 15 机载层：startup_offboard_ego.sh 1..15（11311..11325）
4. GCS_A：topology_group_a_sim.launch uav_name:=GCS_A（11310）
5. backend+标定：gcs_a_backend.launch（须 disarm；A01-A06 计算原点/航向，A01-A15 回读 15/15 ready）
6. TCP server：tcp_server_node.py（39001）
7. 外部起飞准备：offboard_takeoff_15.py 1..15（HOLD/AUTO.LOITER → arm → OFFBOARD → planner TAKEOFF 软起飞 5m → HOLD）
8. 批次：task_test.py --backend tcp --group A --task <TASK> --timeout-s <T>；executor PREPARE 只检查 armed/OFFBOARD/origin，全部 PREPARED 后 GCS START 启动任务
9. 清理：逆序停止，勿用宽泛 killall/pkill
```

批次 timeout 建议（1.0m/s）：prepare 150s；coverage-segment-1/2 480s；fault-exit 240s；recovery 240s；hold-end 150s。

---

## 5. 编译与运行（含换设备/换 workspace 注意）

### 5.1 前置（路径已变量化，换环境只改三个变量）

```bash
# 换设备/换用户名时，只需修改下面三个变量：
WS=/home/ub20tg/catkin_swarm6-2     # 工作空间根目录（含 src/devel/build）
PX4_ROOT=/home/ub20tg/PX4_Firmware  # PX4 固件根目录
PX4_BUILD="$PX4_ROOT/build/px4_sitl_default"

cd "$WS"
source /opt/ros/noetic/setup.bash
catkin build                        # 6 包全量构建（首次或代码变更后）
source "$WS/devel/setup.bash"
# PX4/Gazebo（仿真需要）
source "$PX4_ROOT/Tools/setup_gazebo.bash" "$PX4_ROOT" "$PX4_BUILD"
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4_ROOT:$PX4_ROOT/Tools/sitl_gazebo"
# 统一 ROS 日志（避免散落 ~/.ros）
mkdir -p "$WS/.ros_home/log"
export ROS_HOME="$WS/.ros_home"
export ROS_LOG_DIR="$WS/.ros_home/log"
```

#### 换环境必须修改的硬编码位置（当前使用绝对路径）

> 当前许多脚本/文档写死了 `/home/ub20tg/...`。换设备/换用户名后，除上面三个变量外，
> 还要改下列文件内的绝对路径（建议全局替换 `/home/ub20tg` 为实际家目录）：

| 文件 | 硬编码内容 | 说明 |
|---|---|---|
| `src/safe_valley_exp/startup_offboard_ego.sh` | 第 16-21 行：`ROS_HOME`/`ROS_LOG_DIR`/`WS`/`PX4_ROOT` | 机载层批量启动脚本；换环境必改 |
| `src/safe_valley_exp/scripts/offboard_takeoff_15.py` | `ROS_SETUP`/`WS` 常量 | 起飞脚本；换环境必改 |
| 本文档内命令 | 每节 `cd /home/ub20tg/catkin_swarm6-2`、`export ...=/home/ub20tg/...` | 复制命令前先替换 |
| `references/runtime/` 脚本（若本机运行 runtime） | 运行路径 | 可选，runtime 用相对路径为主 |

> 只依赖 ROS 标准安装（`/opt/ros/noetic`）的路径通常不用改；涉及 `PX4_Firmware`、`catkin_swarm6-2`、
> `ub20tg`（用户名）的都需按实际环境替换。

### 5.2 换设备 / 新 workspace 必须注意

1. **6 个独立 git 仓库**：`src/` 下每包各自 `.git`，分别 commit，**不 push**；不把工作空间根建为 git 仓库。
2. **`references/` 与 `verification/`**：只读参考 / 工程验证，不是交付物，不入任何 git 仓库。
3. **`swarm_uav_interfaces` 版本一致**：所有 GCS_A、bridge、UAVn 主机必须安装、构建、source 完全相同的接口 commit，否则 ROS 类型 MD5 不匹配。
4. **端口约定**（多 Master 隔离）：仿真 11300、GCS_A 11310、UAVn = 11310+n（11311..11325）、Gazebo 11345、TCP 39001；PX4 SITL 仿真 TCP 4560..4574、MAVLink UDP 34580..34594。`39001` 位于常见 Linux 临时端口范围，GCS 主机必须通过 `net.ipv4.ip_local_reserved_ports` 保留，否则 ROS 节点的随机 XML-RPC 端口可能抢占它。启动前运行 `rosrun tcp_to_ros check_tcp_port.py --port 39001`；修改系统配置后须确认输出 `OK`。换环境需核对，尤其实机无独立 Master 端口时。
5. **无 RC SITL 参数**：起飞脚本自动注入 `COM_RCL_EXCEPT=4`、`NAV_RCL_ACT=0`；`COM_RC_IN_MODE` 默认 2（RC 全模式启用），无虚拟摇杆且处于手动模式（POSCTL）arm 会被拒（`manual control lost`）——takeoff 脚本已通过**先切 HOLD/AUTO.LOITER（Auto 模式免 RC）**规避。实机建议保留 RC 仅应急接管，`COM_RC_IN_MODE` 保持默认。
6. **标定需 disarm**：gcs_a_calibration 要求标定计算集 A01-A06 全 disarm + 数据连续 2s + 航向 30° 门禁；冻结结果下发 A01-A15 并要求 15/15 回读确认。armed 时 `gp_origin_receiver` 拒绝改原点。A01-A12 仍是 formation-1/prepare/coverage 成员集，不得改成 A01-A06。
7. **backend 单实例**：重复启动 `gcs_a_backend.launch` 会因 `gcs_a_calibration` 节点重名冲突，先停旧实例。
8. **executor `required=true`**：kill executor 会连带关闭整个机载层 launch（MAVROS/driver/bridge）→ offboard loss → LAND。
9. **C++ 节点空闲态是 HOLD（无 IDLE）**：exec_state 初始/空闲为 `HOLD`；文档/脚本里旧 `IDLE` 表述是历史遗留。
10. **日志**：统一写 `.ros_home/`；跨 Master 问题优先看 bridge 与对应 UAV 的 `uav_executor_UAVn-*.log`。
11. **工具会话超时**：30s 工具限制会杀前台长进程；启动仿真/机载层/批次用 `setsid nohup ... > log 2>&1 < /dev/null &` 后台运行。

#### GCS 业务 TCP 端口保留

ROS 节点的 XML-RPC slave API 使用操作系统随机分配的 TCP 端口。若 `39001` 位于
`ip_local_port_range` 且未被保留，先启动的任意 ROS 节点都可能随机占用 runtime 接口端口；
这不是 topology bridge 的 ZMQ 端口冲突。`tcp_server_node.py` 会尽早绑定并在配置不安全时告警，
但要覆盖 TCP server 尚未启动或重启的窗口，部署主机仍必须持久保留该端口。

```bash
# 只读检查（无需 root）
cat /proc/sys/net/ipv4/ip_local_port_range
cat /proc/sys/net/ipv4/ip_local_reserved_ports
rosrun tcp_to_ros check_tcp_port.py --port 39001

# 运维人员在 GCS 主机上把 39001 合并进现有保留列表；不得覆盖已有条目。
# 持久配置写入 /etc/sysctl.d/ 后，再由运维人员执行 sysctl --system。
# 最终必须重新运行 check_tcp_port.py 并得到 OK。
```

### 5.3 最小部署包集合

```text
GCS_A:     tcp_to_ros + swarm_topology_bridge + swarm_uav_interfaces
每台 UAVn: swarm_uav_executor + swarm_topology_bridge + swarm_uav_interfaces
仿真层:    safe_valley_exp（launch + 起飞脚本）+ PX4/Gazebo
```

`safe_valley_exp` 不属于 GCS_A 必需部署；机载执行能力由 `swarm_uav_executor` + `ego_planner_driver` 提供。

---

## 6. 已落地的关键设计决策（帮助理解现有代码，非后续开发）

- **删除飞行安全租约（lease）**：原机制（renew 1Hz/TTL 5s/锁存）在仿真高负载与自组网电台（200-800ms 抖动）下必然失败；
  改为 `link_health`（MAVROS state 心跳 → DISCOVERED/HEALTHY/SUSPECT/LOST/RECOVERED）+ `safe_hold`。
  断链兜底：节点丢失 → backend LOST → 参与机全 HOLD → 手动接管。`UavSafetyLease.srv` 保留为 dead definition 不删。
- **C++ 主状态 = HOLD**：空闲悬停用 `steady_target_` 冻结位姿，不再每秒刷新（避免漂移）。
  `POSE_STALE` 已删除（26082621），HOLD 锁存进入时位姿，不随 pose 漂移。
- **分层轨迹**：非 follower 全部走 ego 带时轨迹；follower 用 PI 位置 + 固定槽位偏置；垂直段冻结 x/y/yaw。
- **起飞与任务职责**：标定 → 外部 `offboard_takeoff_15.py` 切 HOLD（拒绝时回退 AUTO.LOITER）→ arm → OFFBOARD →
  planner TAKEOFF 软起飞至 5m → HOLD；之后 executor PREPARE 只检查 armed/OFFBOARD/origin，GCS START
  只释放已登记任务。executor 不拥有 arm/OFFBOARD，START 不触发起飞，5m TAKEOFF 也不是 MOVE_TO 子阶段。
- **统一 3D 距离规范**：planner hard-clearance 按预测轨迹同一时刻的 3D 中心距 `<1.0m` 拒绝；executor
  runtime gate 按 fresh odom 的 3D 中心距 `<1.0m` 触发 HOLD；SafetyPredictor 以 `<=1.0m` 判定，边界更保守。
- **Safety 正式默认 active（2026-08-27）**：2026-08-25 的 shadow/active 禁用结论仅为历史状态，已被当前 active 状态取代。

---

## 7. 后续开发需求

以下为本阶段已识别的后续开发项，供下一个 coding agent 排期与实施：

### 7.a ego_planner_driver 仍未通过完整验证

- 已通过单测（状态机 8 项、轨迹测试）与单 Master 冒烟，但**多机 SITL 六批次端到端仍未跑通**。
- 曾修复 hold 捕获竞态（`192ad03`），但 prepare 重跑尚未进行；ego 轨迹在真实 PX4 下的执行
  稳定性（垂直→水平→垂直分层、长航点、机间互惠避碰）仍需多机验证。

### 7.b 航点密化（已完成）

- ✅ 已实现（26082621）：`EgoSwarmDriver` 按 `waypoint_densify_spacing=3.0` 均匀密化分层关键点后
  发布 `/waypoints`；C++ 按相邻关键点推进（plan-exec 分支 A/B），3m 密化点只作推进/末点参考，
  不进 B-spline（B-spline 初始点集由 C++ 侧 min-snap 单段按 0.4m 弧长等距采样生成）。
  早期思路见 `verification/plot_ego_bspline_waypoints.py`。

### 7.c 实机部署方案待确定

- 仿真用多 Master（11300/11310/11311-11325）就是为了模拟"地面站与每台机载电脑各有独立 IP"。
- 需要明确：哪些包部署在 GCS 地面站（tcp_to_ros + bridge + interfaces），哪些部署在每台机载
  电脑（executor + bridge + interfaces），MAVROS/PX4 在本机还是远端；形成实机 IP/主机名映射，
  并复核冻结 `uav_identity_map.yaml`（当前仅仿真规则 A0n→UAVn→192.168.5.x，实机必改）。

### 7.d safe_valley_exp 需要拆解

- 当前只借用了它的个别 launch（`uav_offboard_ego.launch`、`multi_uav_ego_15sim.launch`）和配置，
  其核心 flock 算法并未在工程中真正应用。需要把真正使用的 launch/配置/起飞脚本拆解到对应
  功能包（或新建仿真包），避免把示例包当成依赖、误导部署。

### 7.e takeoff 脚本需按实机重构

- 当前 `offboard_takeoff_15.py` 面向 15 机本地 SITL（固定端口 11311-11325、本机执行），
  实机每台机载电脑独立 IP/端口，无法直接使用。
- 重构方向：参数化 UAV 列表与 ROS Master URI，同时保持当前职责边界：外部脚本只负责本机
  HOLD/AUTO.LOITER → arm → OFFBOARD，planner 负责 TAKEOFF 软起飞 5m → HOLD；executor PREPARE
  只检查 armed/OFFBOARD/origin，GCS START 只启动任务。不得把 arm/OFFBOARD 迁入 executor，
  也不得让 START 触发起飞。

## 8. 已知问题与安全边界

- `POSE_STALE` 已删除（26082621）：pose 过期不再切换状态，HOLD 锁存进入时位姿；执行中位姿
  失联的最终兜底仍依赖 PX4 failsafe（`COM_OBL_ACT` 等）。
- 软件 HOLD 不替代 PX4 datalink/offboard/RC failsafe；实机前必须冻结 `COM_OF_LOSS_T`、`COM_OBL_ACT`、RC/datalink failsafe、急停职责、场地许可。
- 批次任务仍在验证中（prepare 竞态已修，未重跑）；勿以本 README 状态代替实测记录。
- 任何异常优先人工 HOLD/降落/急停；清理用精确 PID，勿用宽泛 `killall`/`pkill -f`。

---

## 9. 给下一个 coding agent 的快速指引

1. 先读 `plans/README.md`（Phase 编号 + 设计规约 + 当前状态）与 `test_records/test_record_26082700.md`（最新实测记录）。
2. 当前主任务：**双机交换 / 三机交叉 / prepare 批次通过**（双机正交 8/9 已过；走廊式/多机并发
   避碰仍不足）。`ego_planner_driver/README.md` 为本包状态机 + replan 策略的单一权威。
3. 改代码只动 `src/` 六个仓库；每包独立 commit（Conventional Commit，英文 header + 中文 body），**不 push**。
4. 提交前 `catkin build` + 相关单测；测试记录写入 `test_records/test_record_2608xxxx.md`（文件名后缀 = 2 位年月日时）。
5. 环境清理、端口、日志、无 RC 参数等注意点见本文档第 5 节。

---

## 10. 文档索引

| 路径 | 内容 |
|---|---|
| `src/tcp_to_ros/plans/README.md` | Phase 编号、全局设计规约、实施计划索引、里程碑、当前状态 |
| `src/tcp_to_ros/plans/implementation_plan_26082415.md` | min-snap 统一单段重构（当前） |
| `src/tcp_to_ros/plans/implementation_plan_26082300.md` / `26082322.md` | 方案 B / PositionTarget 契约基线 |
| `src/tcp_to_ros/test_records/test_record_26082400.md` / `26082300.md` | PVA 链路 / 方案 B prepare 测试记录 |
| `src/tcp_to_ros/README.md` | 本文件（项目总说明 + handoff） |
| `src/ego_planner_driver/README.md` | ego 轨迹驱动接口与安全说明 |
| `src/swarm_uav_executor/README.md` | 机载执行器接口与安全说明 |
| `src/swarm_topology_bridge/README.md` | 跨 Master 桥说明 |
| `references/` | 只读参考（runtime、方案文档、ego-planner-swarm 原版） |
| `verification/` | 工程验证脚本（ego_smoke、acceptance、plot 脚本，不入 git） |
