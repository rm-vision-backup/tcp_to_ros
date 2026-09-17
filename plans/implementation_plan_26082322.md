# 周期性零速修复、1 秒重规划与统一 PositionTarget 控制链路实施计划

日期：2026-08-23 22:00

状态：**待实施**

## 1. 目标与边界

本计划修复 prepare/FOLLOW_ROUTE 场景中与约 4 秒滚动重规划同步的速度周期性降零，并消除
多个业务节点竞争同一飞控 setpoint 入口的问题。实施后，EGO 轨迹和 follower PI 都发布独立的
ENU `mavros_msgs/PositionTarget` 候选命令；`setpoint_relay_node.py` 负责仲裁、校验、失效
HOLD，并以固定频率独占发布 `/mavros/setpoint_raw/local`。同时将 EGO 常规滚动重规划阈值从
当前 4 秒缩短为 **1.0 秒**，对齐原版 EGO-Swarm 的常规时间阈值，降低约 1 m/s、航路交叉场景中
局部轨迹和邻机 intent 陈旧的风险。

本计划只描述后续代码实施，不在本次文档任务中修改业务代码。既有
`implementation_plan_26082300.md` 是冻结历史计划，不回填修改。航点密化按当前 launch 已落地的
`ego_swarm/waypoint_densify_spacing=0.4 m` 描述，不沿用旧计划的 2 m 数值。

不在本计划范围内：

- 修改 TCP 协议、任务批次或 GCS_A 分发逻辑；
- 修改 EGO 避碰代价、分层高度和 PX4 控制器增益；
- 移植原版 EGO-Swarm 的独立安全检查、在线轨迹碰撞预测和紧急停止状态机；这些机制仅作为本计划
  的设计对照与后续安全增强背景；
- 在业务层手工执行 ENU/NED 换轴；坐标变换仍交给 MAVROS raw local setpoint 插件；
- 保留任何并行直发 `/mavros/setpoint_position/local` 或 `/mavros/setpoint_raw/local` 的保活脚本。

## 2. 根因与当前链路

当前高置信根因链如下：

1. `ego_planner_driver` 约每 4 秒滚动重规划，新段位置起点锚定 UAV 实际位置；
2. 规划器内部保留了轨迹速度/加速度语义，但节点只发布 `geometry_msgs/PoseStamped`；
3. relay 原样转发到 `/mavros/setpoint_position/local`，PX4 收不到速度、加速度前馈；
4. 新段切换时位置误差接近零，飞控位置环驱动量周期性下降；
5. follower PI 同样把计算出的速度积分成下一帧位置，再与 EGO 共用 `/setpoint`，来源竞争且再次
   丢失显式速度语义。

ULog 已显示移动位置 setpoint 连续存在，而 `trajectory_setpoint.vx/vy/vz` 全为 NaN；因此主要
问题不是 relay 丢帧，而是“用纯位置接口表达有导数的时变轨迹”。当前 relay 也没有唯一出口、
freshness/finite 校验、模式切换清理或输入失效 HOLD。

### 2.1 原版 EGO-Swarm 安全机制对照（本计划不实施）

原版 `ego-planner-swarm` 不是只依靠常规滚动重规划周期保证动态安全：

- FSM 约以 100 Hz 检查执行状态，`fsm/thresh_replan_time=1.0 s` 作为常规滚动重规划阈值；接近
  最终目标并进入 `fsm/thresh_no_replan_meter=1.0 m` 范围后，可不再按时间重复规划；
- 独立 safety timer 约以 20 Hz（`0.05 s`）运行；其中静态障碍分支扫描当前轨迹未来有效段，邻机
  分支根据本机和邻机轨迹检查机间距离；
- 原版该回调的邻机分支用轨迹绝对起始时间换算邻机当前轨迹时刻，并与本机当前轨迹位置比较；它
  独立于新 intent 到达事件持续运行，但源码并未在该分支中完整扫描未来同时间轴上的两条轨迹，
  不应将其夸大为完整的未来双轨迹碰撞预测；
- 预测到冲突后立即尝试从当前轨迹重规划；若重规划失败且预计冲突进入
  `fsm/emergency_time=1.0 s`，则进入紧急停止，否则切换到常规重规划状态。

这些机制的作用是把“常规轨迹更新”和“高频安全监测”分离：即使邻机不再发布新 intent，执行中
仍可根据已有带时轨迹发现未来冲突，并在碰撞迫近时触发重规划或急停。当前 `ego_planner_driver`
主要依赖周期触发和邻机新 intent 触发，不具备上述完整独立安全闭环。本计划只将常规重规划阈值
缩短到 1.0 秒；**不新增 safety timer、不移植碰撞预测回调、不新增紧急停止状态机**。因此 1 秒
阈值是降低轨迹陈旧风险的改进，不应被描述为等价替代原版安全机制，也不单独构成航路交叉安全证明。

## 3. 冻结后的控制链路

```text
ego_planner_driver (30 Hz)
  /setpoint/ego       PositionTarget: position + velocity + acceleration + yaw
                                \
                                 > setpoint_relay (30 Hz timer, sole owner)
                                /    -> /mavros/setpoint_raw/local
EgoSwarmDriver follower PI
  /setpoint/follower  PositionTarget: position + velocity + yaw

/direct_control_active  Bool: false 选 EGO，true 选 follower
/mavros/local_position/pose: relay 捕获安全 HOLD 的位置和 yaw
```

冻结规则：

- 两个业务发布者只发布候选命令，回调不接触 MAVROS 输出话题；
- relay 是 `/mavros/setpoint_raw/local` 的唯一发布者，以参数化的固定频率（默认 30 Hz）输出；
- `direct_control_active=false` 选择 EGO，`true` 选择 follower；模式切换时清空新选中源的有效标记，
  在收到切换后的新鲜候选前持续 HOLD；
- 选中源过期或包含 NaN/Inf 时进入 HOLD，**不得自动回退到未选中源**，避免旧 EGO 轨迹或旧 PI
  命令意外接管；
- HOLD 从最新且新鲜的本机 local pose 捕获固定 position/yaw。没有可用 pose 时，不构造原点目标；
  保持上一个已验证的 HOLD，启动阶段尚无任何安全目标时发布节流错误并阻止 OFFBOARD 就绪判定；
- relay 每帧重写 `header.stamp=ros::Time.now()`，保留 `frame_id="map"` 作为诊断信息；
  `coordinate_frame` 使用 `FRAME_LOCAL_NED`，业务字段仍按 ROS ENU 填写，由 MAVROS 完成 ENU↔NED。

## 4. PositionTarget 契约

所有候选与输出统一使用 `mavros_msgs/PositionTarget`：

| 模式 | `coordinate_frame` | 启用字段 | `type_mask` |
|---|---|---|---:|
| EGO 轨迹 | `FRAME_LOCAL_NED` | position、velocity、acceleration、yaw | `IGNORE_YAW_RATE` = **2048** |
| follower PI | `FRAME_LOCAL_NED` | position、velocity、yaw | `IGNORE_AFX \| IGNORE_AFY \| IGNORE_AFZ \| IGNORE_YAW_RATE` = **2496** |
| HOLD | `FRAME_LOCAL_NED` | position、yaw | `IGNORE_VX \| IGNORE_VY \| IGNORE_VZ \| IGNORE_AFX \| IGNORE_AFY \| IGNORE_AFZ \| IGNORE_YAW_RATE` = **2552** |

约束：

- 不设置 `FORCE`；acceleration 字段按加速度而非力解释；
- EGO 的 position/velocity/acceleration 必须来自同一轨迹、同一采样时刻；yaw 使用同一时刻的轨迹
  yaw，yaw rate 暂不启用；
- follower 的 PI 输出直接写入 `velocity`，同时把 `leader position + formation offset` 写入
  `position`；不再仅发布 `own_pose + velocity * dt` 的一步位置。限幅保持现有水平/垂直速度限制；
- HOLD 的 velocity/acceleration 字段可显式清零，但由 mask 忽略；固定 position/yaw 是权威语义；
- relay 校验所有**启用字段**有限，校验 frame 和 mask 恰好属于上述契约；忽略字段不得参与有限性
  判定，但发布者仍应初始化为零，便于日志审计；
- 首轮 SITL 必须用 `/mavros/setpoint_raw/target_local` 和 PX4 ULog 确认 MAVROS 变换方向，禁止因
  `FRAME_LOCAL_NED` 名称在业务层重复换轴。

## 5. 分阶段实施

### 阶段 A：EGO 输出完整轨迹导数

修改：

- `src/ego_planner_driver/include/ego_planner_driver/ego_planner_driver_node.h`
  - 引入 `mavros_msgs/PositionTarget`；把 `publishPose()` 改为统一命令构造/发布接口；
  - `lastSetpoint()` 及测试缓存类型迁移为 `PositionTarget`；
  - 如 `TimedTrajectory` 尚无解析采样接口，增加返回同一时刻 P/V/A/yaw 的测试可见 helper。
- `src/ego_planner_driver/src/ego_planner_driver_node.cpp`
  - `setpoint_pub_` 改发 `/setpoint/ego` 的 `PositionTarget`；
  - 执行轨迹时在一次采样中取得 P/V/A/yaw，按 EGO mask 发布；不得再对离散 position setpoint
    做跨帧差分来代替规划轨迹导数；
  - TAKEOFF、steady target、HOLD、POSE_STALE 恢复路径发 position/yaw HOLD mask；
  - 轨迹耗尽后不得继续携带末段非零导数；先切固定 HOLD，再执行 arrival 状态转换；
  - 保留 `direct_control_active` 对 EGO 候选发布的门控，避免 follower 模式持续产生无意义候选；
    真正的 MAVROS 所有权由 relay 保证。
- `src/ego_planner_driver/include/ego_planner_driver/ego_planner_core.h`、
  `src/ego_planner_driver/src/ego_planner_core.cpp`（仅在需要时）
  - 暴露基于原 B-spline/带时轨迹的 P/V/A 同时采样，不改变优化器代价和 0.4 m 上层密化基线；
  - 对采样时间统一 clamp，切换时 P/V/A 使用同一个 `t_switch`。
- `src/ego_planner_driver/launch/ego_planner_driver.launch`
  - 增加候选输出话题参数，默认 `/setpoint/ego`；保留 30 Hz 和现有其他规划参数；
  - 将 `replan_period_s` 从 4.0 缩短为 **1.0**，作为常规滚动重规划阈值；不在本阶段新增原版
    safety timer、独立碰撞预测或紧急停止逻辑。
- `src/ego_planner_driver/README.md`
  - 更新消息类型、候选话题、字段契约，删除“PoseStamped 直接控制”的描述。

依赖：`ego_planner_driver` 的 CMake/package 已声明 `mavros_msgs`，原则上不新增包；实施时只需确认
头文件与导出依赖完整。

### 阶段 B：follower PI 发布显式速度候选

修改：

- `src/swarm_uav_executor/src/swarm_uav_executor/drivers/ego_swarm.py`
  - `_setpoint_pub` 类型改为 `PositionTarget`，默认话题 `/setpoint/follower`；
  - `_follower_loop()` 每次 PI 计算生成 position + velocity + yaw 候选，按 2496 mask 发布；
  - 继续按现有频率重发最后候选，但每次重发更新 stamp；leader/own pose stale、取消、距离门禁失败
    时先结束候选源有效性，再由 finally 发布 `direct_control_active=false`；
  - 模式 Bool 的顺序固定为：进入 follower 时先置 true、等待 relay HOLD 生效后发首帧新候选；退出时
    先停止 follower 候选，再置 false，relay 清理 EGO freshness 并等待 EGO 新帧；
  - PI 积分器在每次进入/退出、leader stale 和取消时清零，保留现有限幅与 anti-windup 行为。
- `src/swarm_uav_executor/config/executor_defaults.yaml`（若话题参数集中配置）
  - 增加 follower 候选话题配置，不引入第二个 MAVROS 输出参数。

依赖：`swarm_uav_executor` 已声明 `mavros_msgs`，不新增包。

### 阶段 C：relay 成为唯一出口并实现安全仲裁

修改：

- `src/swarm_uav_executor/scripts/setpoint_relay_node.py`
  - 从 PoseStamped 无状态复制重构为可单测的缓存/仲裁类；
  - 订阅 `/setpoint/ego`、`/setpoint/follower`、`/direct_control_active` 和
    `/mavros/local_position/pose`；
  - 回调只缓存消息、接收 monotonic 时间和模式 generation；ROS Timer 默认 30 Hz 唯一发布；
  - 分别维护源 freshness，默认 `candidate_timeout_s=0.2`、`local_pose_timeout_s=1.0`，超时参数必须
    大于正常候选周期且小于 PX4 OFFBOARD 丢失窗口；
  - 模式变化递增 generation 并作废新选中源旧缓存；只接受切换后收到的消息；
  - 对 coordinate frame、mask、启用字段 finite、候选接收时间做校验；header stamp 只用于诊断，
    freshness 使用本机接收 monotonic 时间，避免仿真时间跳变；
  - 无效/过期候选立即输出固定 position/yaw HOLD；进入 HOLD 时锁存一次 fresh local pose，避免每帧
    跟随带噪位姿形成移动目标；恢复候选时解除内部 HOLD；
  - 发布器唯一目标改为 `/mavros/setpoint_raw/local`，消息类型为 PositionTarget。
- `src/swarm_uav_executor/launch/uav_executor_ego.launch`
  - 删除旧 `source_topic=/setpoint` 和 `target_topic=/mavros/setpoint_position/local`；
  - 配置两个候选话题、模式话题、local pose、raw target、30 Hz 和两个 timeout；
  - 注释明确 relay 是唯一 MAVROS setpoint owner。
- `src/safe_valley_exp/launch/uav_offboard_ego.launch`
  - 透传需要按仿真/实机覆盖的 relay 参数；默认值一致时不重复配置。
- `src/swarm_uav_executor/README.md`、`src/safe_valley_exp/README.md`、相关 verification 文档
  - 操作检查从 `/mavros/setpoint_position/local` 迁移为 `/mavros/setpoint_raw/local`；
  - 明确禁止额外 hold/keepalive 发布者，更新 rosbag/ULog 采集项。

### 阶段 D：移除旧 Pose 链路与旁路发布

全仓搜索并处理：

- 删除业务控制中的 `/setpoint` `PoseStamped` 发布/订阅；`/local_pose` 的 PoseStamped 感知链路保留；
- 删除 `/mavros/setpoint_position/local` 的 EGO keepalive、测试 publisher、launch remap 和操作说明；
- 审计 `/mavros/setpoint_raw/local`，除 relay 外不允许任何运行时 publisher；
- `safe_valley_exp/scripts/flock_comm.py` 属于旧链路时，不把它与新 EGO launch 同时启动；若仍需兼容
  独立 legacy 场景，必须通过互斥 launch 参数禁用其 MAVROS setpoint publisher，而非竞争发布；
- 更新 launch/smoke 脚本中的 `rostopic hz/type/echo` 断言。

## 6. 测试计划

### 6.1 C++ 单元测试

更新 `src/ego_planner_driver/test/test_ego_state_machine.cpp` 和必要的轨迹测试：

- EGO 执行帧为 `FRAME_LOCAL_NED`、mask=2048，P/V/A/yaw 均有限；
- 匀速测试轨迹的 velocity 非零且方向正确，acceleration 与轨迹采样一致；
- 同一 `t_switch` 前后 P/V/A 无由消息链路造成的归零；
- launch/参数测试确认 `replan_period_s=1.0`；周期触发在约 1 秒达到条件，且不得因本次修改产生
  额外的 20 Hz safety timer 或紧急停止状态机行为；
- TAKEOFF、HOLD、POSE_STALE、COMPLETED/轨迹耗尽输出 mask=2552，位置/yaw 固定；
- `direct_control_active=true` 时 EGO 不发布候选，恢复 false 后首帧 stamp 和字段正确；
- 迁移测试 helper，理想控制器回灌改用 `PositionTarget.position`。

### 6.2 Python 单元测试

更新 `src/swarm_uav_executor/test/test_ego_swarm_driver.py`：

- follower 发布 PositionTarget，frame 正确、mask=2496；
- target position 包含 leader offset，velocity 等于限幅后的 PI 输出，不再只断言一步位置积分；
- 水平/垂直限幅、积分复位、leader lost、取消和完成路径正确；
- 断言 direct-control Bool 进入/退出顺序。

新增 `src/swarm_uav_executor/test/test_setpoint_relay.py`：

- false/true 模式只选 EGO/follower；
- 切换后旧缓存不能穿透，收到新 generation 候选前持续 HOLD；
- candidate timeout、NaN/Inf、错误 frame/mask 均触发 HOLD；
- 未选中源即使更新也不能接管；
- HOLD 锁存 fresh pose/yaw，不随后续 pose 漂移；pose stale 时复用上个安全 HOLD，不生成 (0,0,0)；
- timer 每 tick 恰发一帧、更新 stamp，输出始终是 PositionTarget；
- 恢复有效候选后退出内部 HOLD。

### 6.3 ROS 集成测试

新增 relay rostest，启动 fake pose、两个候选源和 relay：

- `rostopic info /mavros/setpoint_raw/local` 显示唯一 publisher 为 relay；
- 输出稳定 30 Hz，候选源 10/30 Hz 抖动时输出无空档；
- EGO→follower→EGO 切换期间输出只出现旧目标 HOLD 或新源有效帧，不出现交错源；
- 停止选中 publisher 后不晚于 `candidate_timeout_s + 1 tick` 进入位置/yaw HOLD；
- `/mavros/setpoint_position/local` 无发布者。

构建与自动测试命令：

```bash
cd /home/yjq/catkin_swarm6-2
catkin build ego_planner_driver swarm_uav_executor safe_valley_exp
catkin test ego_planner_driver swarm_uav_executor
catkin_test_results --all build
```

### 6.4 PX4 SITL 验收

先单机，再 2 机 leader/follower，最后重跑 12 机 prepare：

1. OFFBOARD 前确认 raw setpoint 连续 >=20 Hz，且只有 relay 发布；
2. `/mavros/setpoint_raw/target_local` 与 PX4 ULog `trajectory_setpoint` 中 EGO 执行段 P/V/A 有限；
3. follower 段 velocity 有限，position 为 leader+offset；HOLD 段固定位置/yaw；
4. 确认运行时 `replan_period_s=1.0`，重规划切换前后 position、velocity、acceleration 无非预期
   阶跃，实际水平速度不再按修复前约 4 秒周期降至零；
5. 将 `replan_period_s` 临时改为 3 秒、5 秒做诊断 A/B，实际速度谱不得随之出现对应主峰；测试后
   必须恢复 1.0 秒基线，不得把较长诊断值带入 prepare 验收或实机配置；
6. 注入 EGO/follower 候选停止、NaN、模式快速切换和 local pose stale，验证 HOLD 与 OFFBOARD 保活；
7. prepare 使用当前 **0.4 m** 航点密化，复核最大速度、终点误差、最小机间距、任务终态和 CPU；
8. 保存 rosbag、relay 日志和每机 ULog，记录修复前后速度谷值、setpoint 导数和重规划时刻对齐结果。

验收门槛：自动测试 0 failure；raw 输出无断流；无多 publisher；EGO ULog velocity/acceleration 不再
为全 NaN；正常巡航无与 replan 参数锁定的周期性零速；HOLD/失效注入不产生跳原点或旧源接管；
运行时常规重规划阈值为 1.0 秒；原有高度分层、0.4 m 密化、终点容差和机间安全不回归。由于本计划
不实施原版独立安全检查，交叉航路安全仍以实测最小机间距和后续安全增强验证为准。

## 7. 实施顺序与提交边界

1. 先增加 PositionTarget 构造/采样与单测，不切 launch；
2. 实现 relay 仲裁类及纯单测；
3. 迁移 follower 候选及其单测；
4. 最后一次性切 launch 到两个候选话题和 raw local 唯一出口，避免中间版本同时向两个 MAVROS
   插件发命令；
5. 清理旧 Pose 控制链路和旁路发布者；
6. 完成 build/test、单机/双机 SITL，再执行 12 机 prepare。

建议按仓库拆分提交，但切换提交应保证各仓库兼容版本清晰记录。任何实机验证前必须重新执行
不上桨门禁，不得仅凭 SITL 结果上桨。

## 8. 回滚方案

- 实施前分别记录 `ego_planner_driver`、`swarm_uav_executor`、`safe_valley_exp` 仓库基线 commit；
- 回滚必须同时恢复发布者、relay、launch 和测试，禁止只把 relay 改回 PoseStamped 而保留
  PositionTarget 候选，或同时启用 position/raw 两个 MAVROS 插件入口；
- 若 raw local SITL 坐标/字段验证失败，停止 OFFBOARD 测试并整体回滚到基线，不以业务层临时换轴
  或修改 PX4 增益绕过；
- 回滚后的临时 Pose 链只用于恢复既有能力，不视为周期性零速已修复；问题保持开放，直到完整
  P/V/A 链路通过验收。

## 9. 完成定义

- EGO、follower、HOLD 三种 PositionTarget 契约和 mask 均有自动测试；
- relay 是 `/mavros/setpoint_raw/local` 唯一发布者，旧 `/setpoint` Pose 控制链路已删除；
- 候选 freshness、finite 校验、generation 隔离和固定位置/yaw HOLD 均通过故障注入；
- C++/Python 单测、rostest、catkin 构建全部通过；
- `replan_period_s=1.0` 已在 launch、运行时参数和 SITL 记录中确认；文档明确该阈值不等价替代
  原版 20 Hz 独立安全检查、碰撞预测和紧急停止机制；
- 单机、双机和 12 机 prepare 的 rosbag/ULog 证明速度导数已传入 PX4，周期性零速消失；
- 相关 README、launch、操作手册和测试记录均更新，明确当前 0.4 m 密化基线与回滚点。
