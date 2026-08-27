# ego_planner_driver 轨迹 OOM 最终修复实施计划

时间：2026-08-27 22:00 CST
状态：**待实施**

## 1. 背景与问题定义

2026-08-27 的 15-UAV active prepare 端到端运行在所有前置步骤通过、GCS_A 接受任务后，于轨迹末端附近出现系统资源耗尽：

- runtime 返回 `FAILURE`，`failed_robot_ids=[A03,A07,A10,A12]`，`ticks=1602`；
- 15 架机同步出现约 3–9 秒 setpoint、OFFBOARD control 和 simulator sensor 数据空洞；
- PX4 批量报告 simulator poll timeout、sensor/estimator timeout、MAVLink timesync 高延迟及 `Failsafe enabled: no RC and no offboard`；
- QGC 更新卡顿，随后多机丢失 OFFBOARD 并出现状态发散；
- 内核日志确认全局 OOM，多个 `ego_planner_driver` 单进程匿名 RSS 达约 5.9–12.0 GiB 后被 OOM killer 杀死；部分 planner 以 `SIGABRT/-6` 退出，符合未捕获 `std::bad_alloc`。

### 1.1 已确认根因

移植版 `OneSegmentPolynomial` 与原版一样，使用物理时间 `T` 约束五次多项式的起终点 P/V/A。但移植版 `samplePolynomialByArcLength()` 固定在 `[0,1]` 求值，没有接收真实 `segment_time`。

当滚动重规划进入毫米/厘米级短末段且继承非零起始速度时，`segment_time` 可小至毫秒级。五次多项式系数包含 `T` 的高阶逆幂，在 `t=1` 求值等于严重超出有效定义域，生成巨大虚假坐标与弧长。随后代码按每 0.4 m 无上限 `push_back()`，造成数亿点级 `std::vector<Eigen::Vector3d>` 扩容。

内核观察到的约 3/4.5/6/9/18 GiB 虚拟内存台阶，与 24 字节 `Eigen::Vector3d` 的 `std::vector` 容量倍增和扩容期间新旧缓冲区共存完全吻合。

如果无界 `point_set` 继续进入 `UniformBspline::parameterizeToBspline()`，该函数还会创建 `(K+4)×(K+2)` dense matrix，并对 x/y/z 重复三次 pivoted QR：内存为 `O(K²)`，计算约为 `O(K³)`，进一步放大故障。

### 1.2 原版与移植版差异

原版 `ego-planner-swarm` 不依赖专门内存管理器，而是通过规划不变量将 `K` 保持在几十点：

1. 单段多项式始终在真实物理时间 `[0,T]` 内采样；
2. 局部目标受 planning horizon 约束；
3. 起点距局部目标 `<0.2 m` 时拒绝继续构造局部轨迹；
4. `ts` 与 control-point spacing / max velocity 绑定；
5. 相邻样本距离必须 `<=1.5×ctrl_pt_dist`；
6. 初始化点集至少 7 点；
7. 滚动重规划复用旧 B-spline 剩余段，再追加有限的物理时间连接段；
8. 点数超过 `planning_horizon/ctrl_pt_dist×3` 时回退重新生成。

移植版保留了原版 dense B-spline 参数化，却丢失了上述关键上游约束，并新增固定 `[0,1]` 采样和无界弧长输出，因此把本应只处理几十点的 dense QR 暴露给了无界输入。

## 2. 修复目标与成功判据

### 2.1 目标

1. 修复五次多项式物理时间域错误。
2. 对所有 duration/count/size conversion 和轨迹分配建立硬资源边界。
3. 区分方向退化、普通可重试失败和资源超限，不把 planned state 误判为真实到达或紧急制动。
4. 资源超限或分配失败必须进入现有安全 HOLD 链，不得导致 planner 进程退出。
5. B-spline 参数化在调用方约束失效时仍能自行拒绝危险输入。
6. 保持现有 ROS/bridge/executor/任务协议和安全距离语义。
7. 增加不会危及宿主机的确定性短末段内存回归。
8. 最终通过自动测试、内存压力测试和受保护的 15-UAV active prepare 验收。

### 2.2 完成判据

- 短末段回归在 512 MiB address-space 限制内稳定通过，无 OOM、abort 或 timeout；
- 所有采样和参数化点数在分配前完成 finite、overflow 和 hard-limit 校验；
- `parameterizeToBspline()` 不再重复三次 QR，失败不残留旧/部分输出；
- `std::bad_alloc` 不逃出 replan thread；
- 资源失败进入 `EGO_PLAN_FAILED → executor HOLD → backend ALL HOLD`；
- `DEGENERATE_SEGMENT` 不替换 `current_`、不推进 target、不完成任务、不启动失败计时且不触发紧急制动；推进和完成只由真实 odometry 判定；
- 轨迹表保持现有 nominal step `0.1 s`：非末端间隔为 `0.1 s`，末间隔在 `(0,0.1 s]` 内以精确覆盖 duration；planner candidate-publication timer 与 setpoint-relay publication timer 保持 `30 Hz`；
- 外部状态、intent、GPS bias、PositionTarget、Safety active 和 1 m 三维球形门禁无回归；
- 相关包及全工作区自动测试 0 errors / 0 failures；
- 15-UAV prepare 成功，所有 planner 存活，无 `bad_alloc/-6/-9/OOM kill`、无 OFFBOARD 丢失；
- planner RSS 达到稳定平台，不随重规划次数持续增长；
- 测试结束后端口和进程全部 clean，证据归档完成。

## 3. 范围与明确不做事项

### 3.1 本次范围

- `ego_planner_driver` 数值核心的有界采样和结果分类；
- B-spline 参数化输入边界与 single-QR 求解；
- node 对资源失败、退化段和 `std::bad_alloc` 的安全收口；
- 跨机 intent consumer 的样本数边界；
- 生产 launch 参数接线；
- 单元、状态机、内存压力和 15-UAV 回归；
- planner README、计划索引和测试记录在实施阶段同步更新。

### 3.2 本次不做

- 不修改 `swarm_topology_bridge` 业务语义；
- 不修改 `UavTrajectoryIntent.msg` 协议；
- 不改变 GPS bias 修正位置或公式；
- 不恢复原版水平 1 m / 垂直 2 m 椭球；
- 不降低 Safety 等级或关闭主动监督；
- 不修改 PX4 增益和 failsafe 所有权；
- 不用 swap、增大主机内存或减少 UAV 数量作为最终修复；
- 不在本次安全修复中完整移植原版 rolling remainder initializer；
- 不在本次安全修复中引入 SparseQR/专用 banded solver 的数值行为变化；
- 不 commit、不 push，除非后续明确要求。

## 4. 必须保持的冻结契约

### 4.1 通信与时间契约

- `swarm_topology_bridge` 继续只做通用序列化/转发；
- intent `stamp` 继续表示实际激活 epoch；
- intent `t[]` 继续表示从 0 开始、严格递增、覆盖精确 duration 的相对时间；
- 消费端继续使用 `T_own = T_peer - peer_bias + own_bias`；
- bridge 不执行时间戳修正、轨迹裁剪或 OOM 业务处理。

### 4.2 状态和执行契约

外部状态集保持不变：

```text
HOLD
TAKEOFF
EXECUTING
COMPLETED
EGO_PLAN_FAILED
EGO_EXEC_TIMEOUT
EMERGENCY_BRAKE
BRAKE_HOLD
```

不把内部资源错误名称加入 `/exec_state`。

- `COMPLETED` 仍是唯一成功终态；
- `EGO_PLAN_FAILED` 仍由 executor 转换为失败 MotionResult，然后执行 HOLD；
- `EMERGENCY_BRAKE` 为执行中状态；
- `BRAKE_HOLD` 为安全制动后的终态失败并确认 HOLD；
- `setpoint_relay` 仍是 `/mavros/setpoint_raw/local` 唯一发布者。

### 4.3 规划权威与频率契约

- 内部 `PlanResult` 仅包含 `SUCCESS/DEGENERATE_SEGMENT/RETRYABLE_FAILURE/RESOURCE_LIMIT`；
- 滚动重规划起始 P/V/A 继续取自有效 `current_`；真实 odometry 继续独占 keypoint 推进和最终完成判定；
- 普通短段、方向退化及 final 段动力学不可行不得单独触发紧急制动；
- `TimedTrajectory` 的 nominal step 保持 `0.1 s`：非末端间隔为 `0.1 s`，末间隔在 `(0,0.1 s]` 内以精确覆盖 duration；planner candidate-publication timer 与 setpoint-relay publication timer 保持现有 `30 Hz`；
- relay candidate timeout 保持 `0.2 s`。

### 4.4 安全基线

- Safety 默认保持 `active`；
- planner hard-clearance：预测同刻 3D 中心距 `<1.0 m` 判失败；
- executor runtime gate：fresh odom 3D 中心距 `<1.0 m` 触发 HOLD；
- SafetyPredictor 保持更保守的 `<=1.0 m`；
- arm/OFFBOARD 所有权仍属于外部脚本；
- 软件 HOLD 不替代 PX4 failsafe。

### 4.5 15-UAV 生命周期

```text
全环境 clean
→ 15 模型仿真层
→ 15 套 active onboard 一次性启动
→ GCS bridge
→ backend 与 A01-A06 标定
→ A01-A15 origin 确认
→ 外部 HOLD/AUTO.LOITER → arm → OFFBOARD
→ planner TAKEOFF 5m → HOLD
→ executor PREPARE
→ GCS START
→ 任务
→ PASS/FAIL/timeout 后立即全清理
```

禁止单 UAV 重启、局部修补后继续同一轮测试。

## 5. 内部结果模型与退化段职责边界

`planOnce()` 只负责从给定规划起点到当前 segment target 生成一段轨迹。它不能依据 planned state 判断真实 UAV 已到达，也不能决定 waypoint 推进、任务完成或紧急制动。

滚动重规划时的 `start.position/start_vel/start_acc` 来自 `current_` 在当前规划时刻的期望 P/V/A；只有首次规划或没有有效 `current_` 时，start position 才来自真实 local pose，导数为零。因此：

```text
planned start ≈ segment target
```

只说明本次输入的 segment 几何方向已经退化，不等于：

```text
真实 UAV 已到 target
任务已完成
需要紧急制动
```

在 `ego_planner_core.h` 增加仅内部使用的：

```cpp
enum class PlanResult
{
  SUCCESS,
  DEGENERATE_SEGMENT,
  RETRYABLE_FAILURE,
  RESOURCE_LIMIT
};
```

将 `EgoPlannerCore::planOnce()` 从 `bool` 改为 `PlanResult`。这些值不进入 ROS `/exec_state`，不改变 executor/backend 协议。

### 5.1 `SUCCESS`

完整轨迹生成成功：

- 输入、采样、控制点和输出轨迹全部 finite；
- 点数未超过资源上限；
- B-spline 参数化和优化成功；
- 动力学与 clearance 校验通过。

node 按现有激活时刻复验、提交 `current_`、发布 intent 并保持 `EXECUTING`。

### 5.2 `DEGENERATE_SEGMENT`

当：

```text
||segment_target - planned_start.position|| < segment_direction_epsilon
```

本段无法稳定定义方向。特别是中间 keypoint 的：

```cpp
end_vel = normalized(segment_target - planned_start.position) * cruise_velocity;
```

会受浮点、插值和时间误差主导。core 必须在归一化方向、估算时长、构造 quintic 和分配 point set 前返回 `DEGENERATE_SEGMENT`。

该结果只表示“本次 segment 输入退化”，不检查 start velocity/acceleration 是否接近零，不声明真实到达，不触发 `EMERGENCY_BRAKE`，不覆盖当前轨迹末端速度。

Node 应在调用 core 前优先用真实 odometry 处理 target：

1. 中间 keypoint：若真实位置已进入 0.5 m，先推进到下一个有效 keypoint，再建立 snapshot；
2. 全局终点：仍只由真实位置 0.2 m + stable window 转入 `COMPLETED`；
3. 重复/过密 keypoint：按真实位置跳过，禁止把微小向量用于末速方向；
4. 若真实位置尚未满足推进/完成条件，但 planned start 已接近当前 target，则不生成新段、不设置零速 steady target、不改变状态，保留现有 `current_` 及其末端 P/V/A，等待真实 odometry 进入既有推进/到达门槛；
5. 若没有有效 `current_`，node 根据真实位置重新选择/推进 target；仍无法形成有效 segment 时按 `RETRYABLE_FAILURE` 处理，不把 planned state 当成真实到达。

core 保留同样的 `DEGENERATE_SEGMENT` 检查作为防御，防止 node 时序、重复点或未来调用方绕过前置逻辑。

`segment_direction_epsilon` 是方向有效性阈值，不是任务到达容差。默认 `0.05 m`：明显小于 0.5 m 中间推进门槛和 0.2 m 最终到达门槛，但足以避免厘米级短向量的方向被浮点、插值和时间误差主导。短段回归必须覆盖该阈值两侧。

### 5.3 `RETRYABLE_FAILURE`

用于普通、非资源型规划失败，例如：

- optimizer 未找到解；
- clearance 不通过；
- 动力学不可行；
- 普通有限数值输入无法形成合法轨迹。

保留现有 activation retry、yield 和 soft-landing：继续执行当前有效轨迹并按 1 s 定时/事件重试，连续失败达到 10 s 后才发布 `EGO_PLAN_FAILED`。

`RETRYABLE_FAILURE` 不自动触发紧急制动。`EMERGENCY_BRAKE` 仍只由既有 SafetyPredictor 紧急事件、正常 replan 失败和 TTC 门槛共同触发。

### 5.4 `RESOURCE_LIMIT`

用于确定性资源安全失败：

- dense sample、parameterization point 或 timed trajectory sample 数量超限；
- double→integer、size multiplication 或索引计算溢出；
- B-spline 参数化拒绝危险规模；
- 规划边界捕获 `std::bad_alloc`。

资源失败在同一事务内不 activation retry、不进入 yield，因为重复执行只会再次申请同类资源。node 清除部分候选，锁存现有安全目标，发布现有 `EGO_PLAN_FAILED`，由 executor/backend 完成整组 HOLD。

### 5.5 与外部状态的映射

| `PlanResult` | 外部处理 |
|---|---|
| `SUCCESS` | 提交新 `current_`，保持 `EXECUTING` |
| `DEGENERATE_SEGMENT` | 不提交新轨迹；保留现有 `current_`，由真实 odometry 推进/完成 |
| `RETRYABLE_FAILURE` | 保留现有轨迹并有限重试；超时后 `EGO_PLAN_FAILED` |
| `RESOURCE_LIMIT` | 不重试，立即 `EGO_PLAN_FAILED → HOLD` |

`COMPLETED` 仍只由真实最终到达条件产生。`EMERGENCY_BRAKE/BRAKE_HOLD` 仍只属于既有紧急避碰链。

## 6. 真实时长、有界弧长采样

`samplePolynomialByArcLength()` 只负责把一个具有合法物理时长的连续多项式转换为有界离散点集。它不判断真实 UAV 是否到达、不推进 keypoint，也不触发紧急制动。

在调用 sampler 前，core 必须先检查：

```text
||segment_target - planned_start.position|| < segment_direction_epsilon
```

若方向退化，直接返回 `DEGENERATE_SEGMENT`，不得继续归一化方向、估算段时长或构造 quintic。

重构 sampler：

```cpp
PlanResult samplePolynomialByArcLength(
    const OneSegmentPolynomial& poly,
    double duration_s,
    double spacing_m,
    std::size_t max_dense_samples,
    std::size_t max_parameterization_points,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    std::vector<Eigen::Vector3d>& out);
```

### 6.1 时间采样

- 仅在 `[0,duration_s]` 求值；
- 使用 `t_i = duration_s*i/(N-1)`，精确包含 `0` 和 `duration_s`；
- `N_requested = ceil(duration_s/0.02)+1` 使用 checked `size_t`；
- 默认 `max_arc_samples=4096`；
- `N_requested` 超限时拒绝本次规划，不通过无界循环生成点，也不静默降低时间分辨率。

`0.02 s` 是把连续多项式变为细折线、近似累计弧长的数值间隔，不是 B-spline control-point 间距，也不是最终 setpoint 发布周期。

### 6.2 数值验证

逐项检查：

- duration 和 spacing 有限且为正；
- start/end 和所有 polynomial sample `allFinite()`；
- 每段 chord length 有限；
- cumulative arc length 有限且不回退；
- interpolation span 和比例有限；
- 任何数量计算、加法、乘法和类型转换不溢出；
- 失败时清空 `out`，不暴露半成品。

普通有限数值输入无法形成合法轨迹时返回 `RETRYABLE_FAILURE`；样本规模、点数或 checked arithmetic 越过资源边界时返回 `RESOURCE_LIMIT`。

### 6.3 输出点数

在任何 `reserve()`/`push_back()` 前计算：

```text
required_points = ceil(total_arc_length/spacing_m) + 1
```

验证：

- 除法、ceil、加一和类型转换均不溢出；
- `required_points <= max_parameterization_points`；
- 默认 `max_parameterization_points=256`；
- 超限返回 `RESOURCE_LIMIT`；
- 不允许 silent truncation，因为截断会改变轨迹形状和 clearance 语义；
- 检查通过后一次 `reserve(required_points)`；
- 成功时首末位置精确，所有点 finite。

### 6.4 与段速度及紧急制动的边界

Sampler 不根据停车距离决定任务是否制动：

- 中间 keypoint 的设计末速约为指向 target 的 `1 m/s`，本来不要求停车；
- 全局终点的设计末速为零，但短 final segment 动力学不可行时属于普通规划失败，返回 `RETRYABLE_FAILURE`，保留现有 `current_` 并按既有 soft-landing 重试；
- `EMERGENCY_BRAKE/BRAKE_HOLD` 仍只由既有 SafetyPredictor 紧急事件、正常 replan 失败和 TTC 门槛共同触发；
- 任何普通短段、方向退化或末端速度条件都不得自行升级为紧急制动。

### 6.5 完整调用顺序

```text
Node 用真实 odometry 处理中间 keypoint 推进与最终稳定到达
→ Core 检查 planned start 到当前 target 的方向是否退化
  → 退化：DEGENERATE_SEGMENT
  → 有效：继续
→ 估算有限、正的 segment duration
→ 在真实 [0,T] 内有界细采样
→ 检查所有样本和累计弧长 finite
→ 分配前预计算 0.4 m 弧长重采样点数
→ 检查点数 <= 配置/绝对硬上限
→ 生成完整 point_set
→ 进入 B-spline 参数化
```

## 7. TimedTrajectory 输出边界

最终 B-spline 采样也必须受控：

- duration、step 和 `duration/step` 有限；
- 在转换为整数和五个 `reserve()` 前完成 checked arithmetic；
- 默认 `max_trajectory_samples=4096`；
- `t/pts/vel/acc/yaw` 必须等长；
- 所有样本必须 finite；
- 时间必须从 0 开始、严格递增、最后一个样本精确等于 duration；
- 任一检查失败不发布 intent，返回 `RETRYABLE_FAILURE` 或 `RESOURCE_LIMIT`。

## 8. B-spline 参数化边界

修改 `UniformBspline::parameterizeToBspline()`：

```cpp
static bool parameterizeToBspline(
    double ts,
    const std::vector<Eigen::Vector3d>& point_set,
    const std::vector<Eigen::Vector3d>& start_end_derivative,
    std::size_t max_points,
    Eigen::MatrixXd& ctrl_pts);
```

### 8.1 输入防御

- 入口立即清空 `ctrl_pts`；
- `ts` 有限且大于零；
- 点数满足最小要求且不超过配置上限；
- 配置默认上限 256，编译期绝对硬上限 1024；
- derivative 数量必须恰好 4；
- 所有点和导数 finite；
- `size_t→Eigen::Index/int` 前验证。

### 8.2 单次 QR

将 x/y/z 合并为 `(K+4)×3` RHS：

```cpp
Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(A);
Eigen::MatrixXd solution = qr.solve(B);
```

只做一次分解，检查：

- rank；
- residual；
- solution `allFinite()`；
- 只有完整成功后才赋值 `ctrl_pts`。

本次在严格 256 点默认上限下保留 bounded dense QR。SparseQR/banded solver 作为后续独立优化，避免在安全修复中同时引入大范围数值差异。

## 9. Node 结果分流与安全收口

第 5 节定义 core 可能返回什么；本节定义 node 收到结果后如何影响 `current_`、重试、状态和 executor/backend。外部 ROS 状态协议保持不变。

### 9.1 Core 调用前的真实位置处理

在建立 replan snapshot 和选择当前 target 前，node 使用 fresh local pose 再执行一次既有 target 处理：

1. 中间 keypoint：真实位置进入 0.5 m 时推进到下一个有效 keypoint；
2. 连续重复/过近 keypoint：继续按真实位置跳过；
3. 全局终点：仍只由真实位置 0.2 m + `arrival_stable_s` 转入 `COMPLETED`；
4. 完成上述处理后再选择 target、采样 planned start P/V/A 并调用 core。

这减少 callback 时序造成的旧 target 退化调用，但不取代 core 的 `DEGENERATE_SEGMENT` 防御。

### 9.2 `PlanResult` 分流

| Core 结果 | activation retry | yield | Node 行为 |
|---|---:|---:|---|
| `SUCCESS` | 仅既有激活时刻 clearance 复验失败时 | 既有裕度/失败分支 | 提交新 `current_`，发布 intent，保持 `EXECUTING` |
| `DEGENERATE_SEGMENT` | 否 | 否 | 不提交新轨迹、不启动失败计时、不改状态；保留现有 `current_` 及末端 P/V/A，等待真实 odometry 推进/完成 |
| `RETRYABLE_FAILURE` | 是 | 是 | 保留现有轨迹，按既有 1 s/事件 soft-landing 重试；连续失败 10 s 后 `EGO_PLAN_FAILED` |
| `RESOURCE_LIMIT` | 否 | 否 | 丢弃部分候选，立即安全收口为 `EGO_PLAN_FAILED → executor HOLD → backend ALL HOLD` |

`DEGENERATE_SEGMENT` 不是成功、失败或真实到达。它不创建零速度 `steady_target_`，不覆盖中间 keypoint 的设计末速，也不触发 `EMERGENCY_BRAKE`。

### 9.3 `SUCCESS` 提交契约

保持现有流程：

```text
生成完整有界候选
→ 按实际完成/激活时刻重新基准邻机轨迹
→ 用最新邻机 intent 再做 1 m hard-clearance
→ 在提交锁内替换 current_
→ 设置 plan_start_
→ 发布 ACTIVE intent
```

任何失败结果都不得发布部分 intent 或替换最后一个合法 `current_`。

### 9.4 `DEGENERATE_SEGMENT` 等待路径

收到 `DEGENERATE_SEGMENT` 时：

- 清除本次 planning request；
- 不执行 activation retry；
- 不调用 yield；
- 不设置或延续 `plan_fail_since_`；
- 不修改 `current_`、`plan_start_`、`steady_target_`、`safety_generation_` 或 `/exec_state`；
- setpoint 路径继续使用当前轨迹；轨迹耗尽后的既有固定末端 HOLD 行为保持；
- 下一次真实 odometry 进入 0.5 m/0.2 m 门槛时，由既有推进/完成逻辑改变 target 或状态；
- 若无有效 `current_`，node 根据真实位置重新选择/推进 target；仍无法形成有效 segment 时进入普通 `RETRYABLE_FAILURE`，不得把 planned state 当真实到达。

### 9.5 `RETRYABLE_FAILURE` soft-landing

保留现有语义：普通规划失败不立即中断 current trajectory。activation-time clearance 变化可有限重试；常规失败可尝试 yield；全失败后继续当前轨迹并按 1 s 定时、冲突或推进事件重试。连续失败达到 `plan_fail_timeout_s`（当前 10 s）后才锁存安全目标并发布 `EGO_PLAN_FAILED`。

普通短段、方向退化和 final 段动力学不可行都不得自行升级为 emergency brake。

### 9.6 `RESOURCE_LIMIT` 立即收口

资源失败不具备同事务重试价值。Node 必须：

1. 立即停止 activation retry 和 yield 生成；
2. 清空本次局部输出，保留最后一个完整合法状态直到 HOLD 接管；
3. 捕获 fresh pose/既有验证 HOLD 目标，禁止发布半成品轨迹；
4. 设置现有 `EGO_PLAN_FAILED`；
5. 由 executor 请求本机 HOLD，并由 backend `ALL` 策略 HOLD 其他活动参与机；
6. planner 进程继续存活，保留诊断能力。

不得通过重复规划、增大 limit、继续 nominal execution 或单机重启规避资源失败。

### 9.7 Yield 结果传播

`tryYieldReplan()` 不再只返回 bool，至少区分：

```text
COMMITTED
NOT_AVAILABLE
RESOURCE_LIMIT
```

若内部 nominal/candidate `planOnce()` 返回 `RESOURCE_LIMIT`，必须向上层传播并执行 §9.6；禁止降级成“无候选”后继续尝试其他档位或下一轮 activation retry。

`DEGENERATE_SEGMENT` 不进入 yield；它由 §9.4 处理。

### 9.8 紧急制动边界保持不变

本次不为普通短段提取或新增 brake 提交路径。现有 `tryEmergencyBrakeLocked()` 继续只在：

```text
SafetyPredictor 产生 emergency_pending
+ 正常 replan 失败
+ primary TTC < emergency_replan_time_s
```

时提交 `EMERGENCY_BRAKE → BRAKE_HOLD`。普通 `DEGENERATE_SEGMENT/RETRYABLE_FAILURE` 不触发紧急制动。

### 9.9 分配异常 containment

即使确定性点数门禁已存在，replan 事务/线程最外层仍设置窄范围 `std::bad_alloc` 边界：

- 转换为 `RESOURCE_LIMIT`；
- 不继续同一事务；
- 不进入 yield；
- 不发布部分 intent；
- 不允许异常逃逸导致 planner abort；
- 不广泛吞掉其他逻辑异常。


## 10. 跨机 intent 输入边界

在 consumer 任何 `reserve(n)` 前验证：

- t 与 sampled trajectory 长度一致；
- sample count 不超过 `max_neighbor_intent_samples`；
- 默认上限 256，正常 producer 约 51 点；
- duration、时间序列和坐标有限；
- 时间严格递增且覆盖 duration；
- 所有乘法、索引和类型转换不溢出。

非法或超限 intent：

- 节流告警；
- 拒绝本次输入；
- 不替换最后一个合法 neighbor intent；
- bridge 无改动。

## 11. 配置与生产接线

在 `EgoPlannerConfig`、node 参数加载和生产 launch 中同步加入四个资源上限与一个方向有效性阈值：

```text
max_arc_samples                 = 4096
max_parameterization_points     = 256
max_trajectory_samples          = 4096
max_neighbor_intent_samples     = 256
segment_direction_epsilon_m     = 0.05
```

### 11.1 参数语义

- `max_arc_samples`：限制第 6 节按 0.02 s 构建的多项式细折线；4096 点覆盖约 81.9 s。
- `max_parameterization_points`：限制 0.4 m 弧长重采样后送入 B-spline 的点集；256 点覆盖约 102 m，正常 3 m 局部段约 8–10 点。
- `max_trajectory_samples`：限制最终 `TimedTrajectory` 的离散 P/V/A/yaw 数组；nominal step 保持 `0.1 s`，非末端间隔为 `0.1 s`，末间隔在 `(0,0.1 s]` 内以精确覆盖 duration；planner candidate-publication timer 与 setpoint-relay publication timer 保持 30 Hz，不修改频率契约。
- `max_neighbor_intent_samples`：限制每条被 planner consumer 长期保存和重复复制的邻机 intent；正常 producer 约 51 点。
- `segment_direction_epsilon_m`：仅判断 planned start 到当前 target 的方向是否具有物理意义；小于 0.05 m 返回 `DEGENERATE_SEGMENT`。它不是 0.5 m 中间推进门槛，也不是 0.2 m 最终到达门槛。

不再配置 `degenerate_velocity_epsilon_mps` 或 `degenerate_accel_epsilon_mps2`：core 不根据速度/加速度宣称真实到达，也不因普通短段触发紧急制动。

### 11.2 生产接线

参数必须同时接入：

- `ego_planner_driver/launch/ego_planner_driver.launch`；
- `swarm_uav_executor/launch/uav_executor_ego.launch`；
- C++ `EgoPlannerConfig` 和 node 参数读取；
- launch/static regression tests。

不得只在 launch 声明而不被 C++ 消费，形成假配置。

### 11.3 配置合法性

Node 启动时验证：

- 所有 count limit 大于零；
- 配置 limit 不超过相应编译期绝对 hard limit；
- `segment_direction_epsilon_m` finite 且大于零；
- 正常配置能覆盖既有 60 m 数值回归和生产局部段。

危险配置采用 fail-fast：记录 `ROS_FATAL` 并拒绝 planner 启动，不静默 clamp。这样配置文件声明值与实际运行值一致。

这些参数是资源安全边界，不是通过调大数值解决规划失败的飞行调参。实施与验收中禁止临时增大 limit 掩盖异常轨迹。

## 12. 文件级改动预期

### 12.1 `ego_planner_driver`

- `include/ego_planner_driver/ego_planner_core.h`
- `src/ego_planner_core.cpp`
- `include/ego_planner_driver/uniform_bspline.h`
- `src/uniform_bspline.cpp`
- `include/ego_planner_driver/ego_planner_driver_node.h`
- `src/ego_planner_driver_node.cpp`
- `launch/ego_planner_driver.launch`
- `test/test_trajectory.cpp`
- `test/test_ego_state_machine.cpp`
- 新增 `test/test_short_final_segment.cpp`
- `CMakeLists.txt`
- `README.md`

### 12.2 `swarm_uav_executor`

- `launch/uav_executor_ego.launch`
- `test/test_ego_launch_config.py`

不改 executor 运行期错误协议和距离门禁实现。

### 12.3 `tcp_to_ros`

- `plans/README.md`：实施阶段加入计划索引和实时状态；
- `test_records/test_record_26082722.md`：实施验证完成后新增证据。

### 12.4 明确不改

- `swarm_topology_bridge`
- `swarm_uav_interfaces/msg/UavTrajectoryIntent.msg`
- `gps_bias_node.py`
- PX4 参数和控制增益

## 13. 测试计划

### 13.1 受限短末段测试

新增独立 `test_short_final_segment`，由父进程分别 fork 功能 worker 和压力 worker：

- functional worker：`RLIMIT_AS=512 MiB`、`RLIMIT_CPU=10 s`，父进程 wall timeout `15 s`；
- stress worker：`RLIMIT_AS=512 MiB`、`RLIMIT_CPU=30 s`，父进程 wall timeout `45 s`；
- 父进程超时后终止 worker；
- 结构化分类结果和内存检查点通过 pipe 返回；worker 在预热结束、中点及后半段每 25 次读取 `/proc/self/statm` 当前 RSS，并同时记录 `getrusage(RUSAGE_SELF).ru_maxrss`；父进程通过 `wait4()` 获取退出状态、signal 和最终 child `ru_maxrss`。

功能 worker 在限制内遍历完整分类矩阵一次：

```text
distance:
0, 1e-6, 0.001, 0.01, 0.049, 0.05, 0.051, 0.10, 0.19, 0.21, 0.49, 1.0 m

speed:
0, 0.2, 0.5, 1.0 m/s

direction:
正向、反向、正交

segment:
final / non-final
```

压力 worker 选择代表性的退化、0.05 m 阈值边界、普通短段和正常段，分别累计至少 500 次；不要求在单个 10 s worker 中重复整个笛卡尔积矩阵。

断言：

- worker 正常退出；
- 无 signal、bad_alloc、timeout；
- finite/default-limit 条件下，`distance < 0.05 m` 必须返回 `DEGENERATE_SEGMENT`；`distance == 0.05 m` 和 `distance > 0.05 m` 不得返回 `DEGENERATE_SEGMENT`，只允许 `SUCCESS` 或 `RETRYABLE_FAILURE`；资源上限注入用例独立断言 `RESOURCE_LIMIT`；
- 成功轨迹所有数组同长且 finite；
- 首末位置和时间准确；
- 点数/样本数不超过上限；
- 内存门禁固定为：最终 child `ru_maxrss < 384 MiB`；后半段当前 RSS 检查点 `max-min <= 32 MiB`；最后一个检查点不得高于中点超过 `16 MiB`。

### 13.2 Core 测试

在 `test_trajectory.cpp` 增加：

- T<1、T=1、T>1 的 `[0,T]` 采样域；
- planned start-target 距离严格小于 `0.05 m` 时返回 `DEGENERATE_SEGMENT`，不得归一化微小方向或构造 quintic；
- 距离严格等于或大于 `0.05 m` 的 finite/default-limit 短段不得返回 `DEGENERATE_SEGMENT`，只允许 `SUCCESS` 或 `RETRYABLE_FAILURE`；
- parameterization cap 边界和 cap+1；
- arc/output sample cap；
- NaN/Inf、非法 duration/spacing；
- parameterizer 非法 ts、点数、导数、非有限输入；
- single QR 输出 shape、finite、boundary residual；
- 保留 60 m 正常路径，证明默认 256 点不误伤现有回归。

### 13.3 状态机测试

在 `test_ego_state_machine.cpp` 增加：

- `DEGENERATE_SEGMENT` 有有效 `current_` 时不得修改 `current_`、`plan_start_`、`steady_target_`、`safety_generation_` 或 `/exec_state`，不得启动或延续 `plan_fail_since_`；
- 真实 odometry 进入中间 0.5 m 后才推进，进入最终 0.2 m 稳定窗后才 `COMPLETED`；
- 无有效 `current_` 且输入方向退化时，node 必须依据 fresh odometry 重新选择/推进 target；仍无有效 segment 时进入 `RETRYABLE_FAILURE`，不得完成任务或触发紧急制动；
- 普通短段或 final 段动力学不可行保持 `RETRYABLE_FAILURE`/soft-landing，不进入 `EMERGENCY_BRAKE`；
- 既有 Safety emergency + replan 失败 + TTC 门槛仍能独立触发 `EMERGENCY_BRAKE → BRAKE_HOLD`；
- `RESOURCE_LIMIT` 不 activation retry、不 yield；
- `YieldLowIdStandOnRequestsReplan` 不再抛 `std::bad_alloc`；
- 资源失败不发布部分 intent。

### 13.4 Launch 与协议回归

- 生产 limits 参数存在且被 C++ 读取；
- Safety 默认仍为 active；
- 1 m 三维球形边界测试不变；
- `EGO_PLAN_FAILED`、`EMERGENCY_BRAKE`、`BRAKE_HOLD` executor 映射不变；
- GPS bias 和 intent epoch/t[] 测试不变；
- 验证 `TimedTrajectory` nominal step 保持 `0.1 s`：非末端间隔为 `0.1 s`，末间隔在 `(0,0.1 s]` 内且末 timestamp 精确等于 duration；
- planner candidate-publication timer 与 setpoint-relay publication timer 的 launch/default 和 node 实际读取值必须等于 `30 Hz`，relay candidate timeout 必须等于 `0.2 s`；
- 在定义的 10 s 稳态 wall-clock 窗内，planner candidate 与 relay output 实测平均频率均须在 `[27,33] Hz`，且 relay output 任一相邻消息间隔必须 `<0.2 s`；
- 静态 launch 值、node 实际读取值和运行观测必须一致。

## 14. 验证顺序

1. 记录所有嵌套仓库 HEAD 和 status，不覆盖已有修改。
2. 先增加受限复现测试，确认旧逻辑的复现故障被隔离在受限子进程内，不终止 test runner 或危及宿主机。
3. 实施退化分类、真实时长采样和点数边界。
4. 实施 bounded single-QR parameterizer。
5. 实施 node 结果分流、退化段等待、yield 资源传播、bad_alloc containment 和 intent 输入限制。
6. 同步生产 launch 和文档。
7. 构建：`ego_planner_driver swarm_uav_executor tcp_to_ros`。
8. 使用外层 `timeout + prlimit` 运行 targeted short-final test。
9. 使用 `-j1 -p1` 串行运行相关包测试，避免并发测试重新放大资源错误。
10. 运行全工作区测试，以当前实际测试数重新基线，要求 0 errors / 0 failures。
11. 先运行单 planner 500 次门禁压力测试；通过后运行 2000 次长压力测试，两者均检查当前 RSS 平台、`ru_maxrss` 预算和资源上限。
12. 只有全部自动门禁通过后，才能运行受保护 15-UAV active prepare。
13. 完成后写测试记录并更新实时状态。

## 15. 15-UAV 内存保护与验收

首次全量重跑必须使用 cgroup 或 watchdog：

- 保留至少 `max(8 GiB, 25% physical RAM)` 给宿主机；
- 记录 `memory.current`、`memory.peak`、`memory.events`；
- 记录每个 planner RSS/PSS；
- 记录 host MemAvailable/swap；
- 记录 raw setpoint 实际频率和最大间隔；
- 记录 `TimedTrajectory` 表间隔、planner candidate-publication 与 setpoint-relay output 实际频率；
- 记录 PX4 OFFBOARD 变化；
- 记录 planner point/control/sample 峰值。

任何以下事件均立即判 FAIL 并全环境清理：

```text
std::bad_alloc
SIGABRT/-6
SIGKILL/-9
OOM/OOM kill
planner required-node death
OFFBOARD loss
sample/control limit violation
TimedTrajectory nominal step/末样本契约失败，或 planner candidate/setpoint relay 的 10 s 稳态实测平均频率不在 [27,33] Hz
raw setpoint 最大间隔达到或超过既有 relay candidate timeout 0.2 s
任务 timeout/failure
```

禁止单机恢复后继续本轮。

## 16. 实施顺序与提交边界

实施与验证顺序以 §14 为唯一依据，本节只规定提交边界：

- core、node、resource limits、launch 与对应测试必须保持为兼容集合，不拆成会留下危险分配路径或假配置的中间状态；
- README、计划索引和测试记录随最终验证结果同步更新；
- 仅在用户明确要求后，才分别在对应嵌套仓库 commit；不自动 push。

## 17. 回滚方案

- 实施前分别记录 `ego_planner_driver`、`swarm_uav_executor`、`tcp_to_ros` 的 commit 和 dirty 状态；
- 不使用 root-level reset，工作空间根目录不是 Git 仓库；
- 不覆盖/回滚现有 Safety、距离、GPS bias 和启动流程修改；
- 代码 limits、launch 参数和测试必须作为兼容集合一起回滚；
- 禁止只撤掉 exception containment 而保留危险分配路径；
- 禁止只调大点数上限掩盖合法轨迹被拒绝；
- 如果新资源边界拒绝正常任务，停止 SITL，保留证据，调整几何推导或算法，不降低安全门禁。

## 18. 后续独立优化建议

以下内容不阻塞本次最终 OOM 修复：

1. 将 dense parameterization 改为 SparseQR 或专用 banded least-squares；
2. 完整恢复原版“旧 B-spline 剩余段 + 连接段”rolling initializer；
3. 评估连续 planning-horizon local target，减少相邻 keypoint 短段；
4. 复用 optimizer gradient scratch buffers；
5. 修正 terminal control points “参与优化后又被覆盖”的混合语义；
6. 对外部超远全局目标和所有 ROS 动态数组统一建立消息级资源预算。

这些优化必须另立计划、独立 A/B，不与本次安全修复捆绑。
