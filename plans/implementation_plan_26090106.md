# MOVE_TO / FAULT_EXIT 基于 t_off 的 GCS 机间避碰实施计划

**计划日期：** 2026-09-01 06:00（文件生成时间，24 小时制）  
**来源：** `.tmp/reviews/papers_theory_review_for_time_offset_model.md` 第 11 章、`.tmp/notes/multi_uav_time_offset_theoretical_model.md`

## 1. 可执行性结论

第 11 章可以迁移为本项目的可执行方案。方案属于 GCS 一次性启动时序协调，不改变 EGO planner 的轨迹优化，不把 `t_off` 下沉到机载，也不修改 runtime 文件。

迁移时必须补齐两个现有代码缺口：

```text
1. GCS 当前只订阅 uav_task_state 和 mavros/state，需增加各 UAV 当前位姿缓存。
2. 当前 START 确认使用固定 confirm_deadline_s=4s，需增加独立的 t_off 调度预算，避免等待偏移超过 4s 后被错误判定为 START 失败。
```

除此之外，现有接口已经具备所需生命周期：

```text
GCS Action
  → 并发 PREPARE / task service
  → 按 t_off 调度 START / task_control service
  → 机载收到 START 后立即执行
```

## 2. 已冻结的设计决策

```text
适用命令：MOVE_TO、FAULT_EXIT
排程位置：tcp_to_ros GCS
起点来源：bridge 转发的各 UAV 当前 local pose 的水平位置
预测对象：二维水平直线段
水平速度：YAML 参数，默认 0.7 m/s
目标点：MOVE_TO target_pose；FAULT_EXIT 最终退出 waypoint
优化目标：包含 UAV1 的完整 t_off 方差最小
负偏移：允许，求解后整体平移为非负发送时间
实际等待：GCS 按 t_off 调度 START
机载行为：不接收、不保存、不解析 t_off，收到 START 后立即执行
状态反馈：失败和 fail id 使用已有 runtime 任务结果；schedule_status 仅写 GCS 日志
runtime 文件：本计划不修改
安全兜底：保留现有距离门禁、HOLD、yield/replan 和失败收口
```

## 3. 当前源码接入事实

| 位置 | 当前事实 | 本计划改动 |
|---|---|---|
| `scripts/gcs_a_backend_node.py` | `_confirm_participants()` 并发发送任务；`_control_participants(..., "START")` 并发发送 START | 增加排程调用、位姿缓存和按目标时刻发送 START |
| `scripts/gcs_a_backend_node.py` | `_subscribe_uav_states()` 当前订阅任务状态和 `mavros/state` | 增加 `/UAVn/mavros/local_position/pose` 订阅及新鲜度缓存 |
| `config/backend_defaults.yaml` | 有确认、重试和链路参数 | 增加排程速度、距离、搜索和 START 等待预算 |
| `swarm_topology_bridge/config/topology_group_a_sim.yaml` | 已转发 `mavros/local_position/pose` | 仿真无需增加桥接主题 |
| `swarm_uav_interfaces` | 已有 `UavTask`、`UavTaskControl` | 不修改消息/服务定义 |
| `swarm_uav_executor` | PREPARE 后等待 START，收到 START 启动任务 | 不增加 t_off 状态；仅验证等待预算不破坏现有 timeout |
| `references/runtime` | 已定义 MOVE_TO、FAULT_EXIT 任务 | 不修改任务文件 |

当前 `formation_offsets.yaml` 只用于 FOLLOW_ROUTE 编队槽位，不作为本计划的 MOVE_TO/FAULT_EXIT 起点。

## 4. 理论模型到代码的映射

### 4.1 任务输入

对同一批命令中的每架参与 UAV：

```text
p0_i = bridge 当前 pose 的 (x,y)
pg_i = MOVE_TO target_pose 的 (x,y)
       或 FAULT_EXIT 最终 waypoint 的 (x,y)
v_xy = horizontal_speed_mps
```

当前任务执行前存在明显间隔，按 UAV 已悬停处理。排程只使用二维水平位置；不读取、不投影、不比较任何高度坐标。通信和调度误差进入安全距离裕度。

### 4.2 水平运动模型

$$
\Delta p_i^{xy}=p_{g,i}^{xy}-p_{0,i}^{xy}
$$

$$
L_i^{xy}=\|\Delta p_i^{xy}\|,\qquad
T_i^{xy}=\frac{L_i^{xy}}{v_{xy}}
$$

启动偏移为 $\tau_i$ 时：

$$
I_i=[\tau_i,\tau_i+T_i^{xy}]
$$

任务坐标目标必须经过现有 `task_point_to_local_enu()` 变换后再与 bridge local pose 比较，避免坐标系混用。

### 4.3 两两安全集合

对每一对 UAV：

$$
\delta_{ij}=\tau_j-\tau_i
$$

在对应水平运动段的同时运动区间内计算：

$$
 r_{ij}(t)=a_{ij}+b_{ij}t
$$

$$
 D_{ij}^{2}(t)=\|r_{ij}(t)\|^{2}
$$

通过区间端点和二次函数极小点求：

$$
D_{ij,\min}(\delta_{ij})
=\min_{t\in I_{ij}(\delta_{ij})}D_{ij}(t)
$$

安全集合为：

$$
\mathcal S_{ij}
=\{\delta\in\mathbb R:D_{ij,\min}(\delta)\ge d_{ij}\}
$$

其中：

$$
 d_{ij}=d_{\mathrm{geom}}+d_{\mathrm{margin}}
$$

实现必须保留安全集合的所有不连续区间，并直接进行区间边界分析与区间组合搜索；不得将固定时间步长网格作为安全性判定或可行解搜索的替代。

### 4.4 多机方差优化

固定 UAV1：

$$
\tau_1=0
$$

求解：

$$
\min_{\tau_1,\ldots,\tau_N}
\frac{1}{N}\sum_{i=1}^{N}(\tau_i-\bar\tau)^2
$$

约束：

$$
\tau_j-\tau_i\in\mathcal S_{ij},
\qquad 1\leq i<j\leq N
$$

实现采用安全区间组合搜索：

```text
1. 为每个 UAV 对生成候选安全区间。
2. 深度优先选择区间组合。
3. 将已选区间转成差分约束。
4. 求解固定组合的凸二次方差问题。
5. 以等待上限、组合上限和当前方差下界剪枝。
6. 保存方差最小的已验证可行解。
```

搜索失败时返回 `NO_FEASIBLE_SCHEDULE`，不通过继续增加等待掩盖无解。允许返回搜索上限内找到的已验证可行解，并在日志标记为非全局最优。

### 4.5 非负发送时间

求解结果允许包含负偏移：

$$
\tau_i'=\tau_i-\min_k\tau_k
$$

GCS 以统一单调时钟参考点 $t_0$ 调度：

$$
 t_{\mathrm{start},i}=t_0+\tau_i'
$$

整体平移不改变相对偏移、方差和安全性。

## 5. GCS 工程实现

### 5.1 新增排程模块

在 `tcp_to_ros` 内新增纯计算模块，建议放在独立 Python 模块中，避免把几何和优化逻辑直接堆入 action backend。

输入：

```text
command
参与 UAV 的任务层 Axx ID
已转换到 local ENU 的二维起点/目标
horizontal_speed_mps
geometric distance
safety margin
搜索和等待上限
```

输出：

```text
success / failure
validated tau_i'
每对 UAV 的安全区间和预测最小距离
failure_reason
fail_ids
schedule_status（仅供 tcp_to_ros 日志）
```

### 5.2 位姿订阅和快照

在 GCS 初始化订阅每架已启用 UAV 的：

```text
/<exec_target>/mavros/local_position/pose
```

缓存：

```text
x, y
消息时间戳
GCS 接收单调时间
```

开始排程前生成一次一致快照。位姿缺失或超过 `schedule_pose_timeout_s` 时，排程失败并返回对应 Axx；不使用旧位姿继续计算。

`topology_group_a_sim.yaml` 已包含 `mavros/local_position/pose` 转发，当前仿真直接复用该配置。

### 5.3 Action 执行顺序

在现有 `_run_goal()` 中对 `MOVE_TO` 和 `FAULT_EXIT` 增加前置排程：

```text
1. 收集任务参与 UAV 和目标。
2. 读取 bridge 位姿快照。
3. 读取二维水平位姿快照和目标点。
4. 计算全部安全集合和多机 t_off。
5. 排程失败：不调用 PREPARE，不发送任何 UAV 任务；向 runtime 返回失败和 fail id；写 GCS 日志。
6. 排程成功：沿用现有并发 PREPARE。
7. 全部 PREPARE accepted 后，按 t_off 调度 START。
8. 继续现有终态等待和异常处理。
```

其他命令沿用当前流程，不进入该排程器。

### 5.4 START 调度

当前 START 是并发 fan-out。修改为：

```text
1. 计算统一单调参考时刻 t0。
2. 每架 UAV 建立独立调度 worker。
3. worker 等待 t0 + tau_i'。
4. 到时调用现有 UavTaskControl START。
5. 记录计划发送时间、实际发送时间和响应延迟。
```

不得用一个循环顺序发送并累加 service 响应时间。

`confirm_deadline_s=4s` 不能继续作为延迟 START 的唯一预算。YAML 只配置两项附加预算：

```text
start_service_budget_s
start_dispatch_jitter_margin_s
```

其中 `start_service_budget_s` 覆盖 START service 的单次等待、重试和重试间隔；`start_dispatch_jitter_margin_s` 覆盖调度线程唤醒、GCS 本地排队和正常通信抖动。程序在获得排程结果后自动计算本批 START 的总截止时间：

```text
start_schedule_deadline_s
    = max(tau_i')
    + start_service_budget_s
    + start_dispatch_jitter_margin_s
```

`start_schedule_deadline_s` 是程序内部的派生值，不作为 YAML 手工配置项。

` t_0` 必须在全部 PREPARE 返回 accepted 后重新建立，不能在排程计算或 PREPARE 开始前固定。程序以该时刻为 START 调度基准，并使用已平移的 `tau_i'` 计算每个 worker 的目标发送时刻。`schedule_max_start_delay_s` 只限制 `max(tau_i')`；若该值超限，排程在 PREPARE 前失败。`start_service_budget_s` 和 `start_dispatch_jitter_margin_s` 只用于派生 START 截止时间，不改变各 UAV 的相对 `tau_i'`。

同时校验任务 `timeout_s` 足以覆盖 PREPARE 后等待和实际 MOVE_TO/FAULT_EXIT 执行时间。该调整不改变消息结构。

## 6. 失败和日志契约

### 6.1 GCS 排程状态

只在 `tcp_to_ros` 日志记录：

```text
SCHEDULE_OK
GEOMETRY_UNSUITABLE
PAIR_SAFE_SET_EMPTY
GLOBAL_SCHEDULE_INFEASIBLE
SCHEDULE_LIMIT_REACHED
```

同时记录：

```text
mission_id / command_id / command
参与 UAV 和 fail id
pose snapshot age
二维起点、二维目标和预测时段
t_off / 选中安全区间
预测最小距离与 d_ij
计算耗时
实际 START 发送偏差
```

### 6.2 runtime 返回

排程前置失败时使用现有任务失败结果：

```text
status = FAILED
error_code = 既有失败码或新增 schedule-specific error code
failed_uavs = 相关 Axx
message = 简要失败原因
```

`schedule_status`、候选区间和详细诊断不进入 runtime 状态接口，也不进入机载消息。

### 6.3 执行阶段

实际启动偏差、轨迹跟踪误差和机间距离门禁触发继续由现有机载状态处理。排程计划不增加新的机载状态，也不改变现有 HOLD/yield/replan 逻辑。

## 7. 参数计划

在 `src/tcp_to_ros/config/backend_defaults.yaml` 增加：

```text
move_to_schedule_enabled: true
fault_exit_schedule_enabled: true
horizontal_speed_mps: 0.7
schedule_geometric_distance_m: 1.0
schedule_safety_margin_m: <按当前仿真通信和跟踪误差标定>
schedule_pose_timeout_s: 1.0
schedule_time_epsilon_s: <正数>
schedule_max_start_delay_s: <任务允许值>
schedule_max_interval_combinations: <资源上限>
schedule_max_compute_time_s: <资源上限>
start_service_budget_s: <START service 等待、重试和重试间隔预算>
start_dispatch_jitter_margin_s: <调度线程和正常通信抖动裕度>
```

参数必须通过现有 GCS launch/YAML 链路加载；不在代码中复制第二套默认值。

## 8. 测试计划

本轮实施和验收暂不测试 `FAULT_EXIT`。`FAULT_EXIT` 的实现设计保留在计划范围内，专门测试和验收延后；本轮最终成功率只统计干净重启后的 `prepare` 测试。

### 8.1 排程器单元测试

覆盖：

```text
同向平行：tau 接近同步
相向：存在冲突时生成非零偏移
正交交叉：验证安全区间边界
不同航程：验证同时运动区间端点
负 tau：验证整体平移
不连续安全集合：确认不跳过窄可行区间
两两可行但全局无解
安全集合为空
搜索上限命中
```

每个测试同时验证：

```text
所有两两最小距离 >= d_ij
UAV1 参与方差计算
平移前后相对偏移和方差一致
```

### 8.2 GCS 集成测试

覆盖：

```text
pose topic 缺失/过期 → 不 PREPARE，返回 fail id
MOVE_TO 正常排程 → PREPARE 并发，START 按 tau 发送
排程无解 → 不发送任务，runtime 收到 FAILED
START 发送抖动 → 日志记录实际偏差
自动预算派生 → 验证 `max(tau_i') + start_service_budget_s + start_dispatch_jitter_margin_s` 的计算结果
任务 timeout 不足 → 前置拒绝而非运行中误超时
非 MOVE_TO/FAULT_EXIT → 现有流程不回归
```

### 8.3 仿真验收

当前仅进行 Gazebo 仿真。`swarm_simulation` 提供物理仿真效果，`tcp_to_ros` 使用同一套排程代码。所有排程预测、实际启动偏差、安全收口和任务结果检查都并入现有 `prepare` 测试，不新增专项场景。

### 8.4 最终五次干净重启 prepare 验收

在新增 `src/tcp_to_ros//test_records` 新建测试记录文档，时间后缀与计划文件相同。

本轮最终验收只执行现有 `prepare` 测试，不执行 `FAULT_EXIT`。每一轮必须从干净状态开始：

```text
1. 停止上一轮遗留的 ROS、bridge、GCS、UAV、仿真和测试进程。
2. 清理上一轮的临时状态、端口占用、ROS 日志隔离目录和测试结果标记。
3. 重新启动完整 prepare 测试环境。
4. 执行同一组 prepare 测试并记录结果。同步记录排程预测的最小水平中心距、实际 START 偏差、HOLD/yield/replan 事件和最终任务结果，确保整条证据链可追溯。
5. 无论成功或失败，都执行完整清理后再开始下一轮。
```

连续完成 5 轮，每轮测试结果都记录在测试记录文档中，最后整理表格在对话框报告测试结果和结论

任何一轮发生启动失败、任务拒绝、超时、进程退出、链路异常或清理不完整，均计为该轮失败；不得通过重试同一轮覆盖失败记录。

预计修改：

```text
src/tcp_to_ros/scripts/gcs_a_backend_node.py
src/tcp_to_ros/scripts/tcp_to_ros/time_offset_scheduler.py
src/tcp_to_ros/config/backend_defaults.yaml
src/tcp_to_ros/test/test_time_offset_scheduler.py
src/tcp_to_ros/plans/README.md（登记本计划）
```

按本计划不修改：

```text
swarm_uav_interfaces 的 msg/srv 定义
swarm_uav_executor 的 t_off 接口
ego_swarm.py 的轨迹执行逻辑
ego_planner_driver 的规划核心
references/runtime 的任务文件
swarm_simulation 的 Gazebo spawn 位置
```

## 9. 完成判据

本计划完成需同时满足：

```text
1. MOVE_TO 和 FAULT_EXIT 均能从 bridge pose 建立排程输入。
2. 排程器保留全部安全区间，并使用区间边界分析与区间组合搜索。
3. 方差优化包含 UAV1，负 tau 可正确整体平移。
4. 排程成功后 PREPARE 仍按当前并发模型执行。
5. START 按 t_off 调度，且不累积顺序 service 延迟。
6. 排程前无解不会向 UAV 发布任务，并向 runtime 返回失败和 fail id。
7. schedule_status 只在 tcp_to_ros 日志中出现。
8. 机载不解析 t_off，现有安全状态机无新增排程状态。
9. 仿真使用同一套排程实现和 Gazebo 物理环境。
10. 单元、GCS 集成和仿真验收记录可追溯；最终 `prepare` 验收完成 5 次干净重启并报告测试结果和结论。
```

实施完成后，再单独评估 planner yield 触发频率；本计划不以删除 yield 作为完成条件。
