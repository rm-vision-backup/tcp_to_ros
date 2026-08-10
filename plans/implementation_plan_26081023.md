# Implementation Plan

[Overview]

实现四种完整动作命令（MOVE_TO / FOLLOW_ROUTE / FAULT_EXIT / HOVER），其中 FOLLOW_ROUTE follower 实时跟踪 leader+自身编队偏置并用位置层 PI 控制缓解延迟，其余所有自主路径（MOVE_TO、FOLLOW_ROUTE leader、FAULT_EXIT）统一交给 ego 实时规划以保留机间互惠避碰能力。

依据 `src/tcp_to_ros/docs/test_record_26081021.md` 场景 2+ 测试结果：FOLLOW_ROUTE follower（P2）、FAULT_EXIT、HOVER 当前均被按 P1 边界拒绝/失败，本计划把这些命令在机载执行层（`swarm_uav_executor` + `ego_planner_driver`）补齐。分发层（GCS_A/TCP/bridge）无需改动。

关键设计约束（已与需求方确认）：
- 高度分层：MOVE_TO@15m、FOLLOW_ROUTE@12m、FAULT_EXIT@8m；高度不符合时**纯垂直先行**（driver 分两阶段：① 纯垂直过渡到目标高度层（ego 规划纯垂直轨迹）→ ② 水平运动）。
- HOVER：收到指令时读取并**冻结**当前位置，持续向飞控发布该冻结位置（不持续跟随自身当前位置）；不做水平移动、不走 ego 避碰。
- FOLLOW_ROUTE follower：leader 位姿来源 = `/<leader>/mavros/local_position/odom`（bridge 桥接后带 leader namespace 前缀，ENU 坐标系）；编队偏置来自 GCS_A 预置的固定编队槽位（ENU 坐标）；本机 target = leader 位姿 + 偏置；在位置层面对偏差做 **PI 控制**输出 setpoint；follower 不走 ego 避碰。
- 所有"非跟随、需要移动"的自主动作（MOVE_TO、FOLLOW_ROUTE leader、FAULT_EXIT、纯垂直过渡）统一交给 ego 实时规划（`ego_planner_driver` 已有 `planOnce`，带机间互惠避碰）。

[Types]

单句描述：`MotionGoal` 扩展承载 follower 语义所需的层高指示与目标执行层高度，`ExecutorConfig` 扩展编队槽位与 PI 增益配置。

详细类型定义（改动 `src/swarm_uav_executor/src/swarm_uav_executor/models.py`）：

```python
@dataclass(frozen=True)
class MotionGoal:
    x: float
    y: float
    z: float
    yaw: float
    waypoints: Tuple[Tuple[float, float, float, float], ...] = ()  # FOLLOW_ROUTE leader 航线（含沿程高度，应为 12m 层）
    leader_id: str = ""                 # follower=leader 的 A-id；leader/单机为空
    formation_follow: bool = False      # True=follower 编队跟随语义
    command: str = "MOVE_TO"            # MOVE_TO / FOLLOW_ROUTE / FAULT_EXIT / HOVER
    layer_z: float = 15.0               # 本命令的目标执行高度层（15/12/8/冻结值），驱动层据此做纯垂直先行
    formation_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # follower 相对 leader 的 ENU 编队偏置（来自 GCS_A 预置槽位）
```

`MotionGoal` 保持 frozen dataclass；新增字段全部带默认值，不破坏现有构造点。

新增配置（同文件 `ExecutorConfig` 追加字段，均带默认值）：

```python
@dataclass(frozen=True)
class ExecutorConfig:
    # ... 既有字段 ...
    follower_p_gain: float = 1.0        # 位置偏差比例增益
    follower_i_gain: float = 0.1        # 位置偏差积分增益
    follower_limit_xy: float = 2.0      # follower setpoint 最大水平补偿量 m/s 等价（按周期换算）
    follower_limit_z: float = 1.0       # follower 最大垂直补偿量 m/s
```

新增驱动层常量（`ego_swarm.py`）：

```python
_LAYER_MOVE_TO = 15.0
_LAYER_FOLLOW_ROUTE = 12.0
_LAYER_FAULT_EXIT = 8.0
_LAYER_TOL = 0.5          # 已接近目标层，跳过垂直过渡的容差
_HOVER_SETPOINT_HZ = 30.0
_FOLLOW_POSE_TIMEOUT_S = 1.0   # leader odom 超时判为丢失 → HOLD/LEADER_LOST
```

[Files]

单句描述：在 `swarm_uav_executor` 包内修改 driver/校验/配置/launch/测试，`ego_planner_driver` 仅文档无需代码改动。

- 修改 `src/swarm_uav_executor/src/swarm_uav_executor/models.py`：`MotionGoal` 加 `layer_z`/`formation_offset` 字段；`ExecutorConfig` 加 PI 增益与限制字段。
- 修改 `src/swarm_uav_executor/src/swarm_uav_executor/validation.py`：
  - `validate_task_request` 中移除 FOLLOW_ROUTE follower 的 `NOT_IMPLEMENTED` 拒绝（`leader_id != uav_id` 时不再 raise，改走 follower 校验）。
  - `validate_route_assignment`：`formation_follow=True` 时要求 `leader_id` 非空且不等于本机、`target_pose` 或 `waypoints` 仅作 leader 参考，follower 不要求 waypoints；返回 `MotionGoal` 时带 `formation_follow=True`、`leader_id`、`formation_offset`（由配置/任务提供，缺省 (0,0,0)）。
  - `build_goal`：为 MOVE_TO/FAULT_EXIT/HOVER 填充 `layer_z`（15/8/忽略），FOLLOW_ROUTE leader 填充 12。
  - 保留层高约束（MOVE_TO z≈15 容差、FOLLOW_ROUTE waypoints z≈12、FAULT_EXIT z≈8），HOVER 仅要求 1..16 冻结高度合法。
- 修改 `src/swarm_uav_executor/src/swarm_uav_executor/drivers/ego_swarm.py`（核心）：
  - `__init__` 增加参数：`follower_p_gain`、`follower_i_gain`、`follower_limit_xy`、`follower_limit_z`、`layer_move_to=15.0`、`layer_follow_route=12.0`、`layer_fault_exit=8.0`、`formation_offsets`（dict uav_id→(x,y,z) 供 follower 用，缺省空 dict）、`leader_odom_topic_prefix`（如空则自动按 `/<leader>/mavros/local_position/odom` 构造）。
  - 增加 leader odom 订阅：`rospy.Subscriber("<leader_name>/mavros/local_position/odom", Odometry, cb)`（follower 时才订阅；leader_name 从 `goal.leader_id` 映射 exec_target，例 A02→UAV2；namespace 统一为 UAVn）。
  - 新增 `_on_leader_odom`：锁内缓存 `_last_leader_pose`、`_last_leader_odom_mono_s`。
  - `start_move_to(goal, ...)` 按 `goal.command` 分派：
    - HOVER：冻结当前本机位姿（读 `_last_pose`），进入 `_hover_loop`（30Hz 发布冻结 PoseStamped 到 setpoint_pub，直到 cancel/deadline）；返回 success 且 state 走 COMPLETED（复用 exec_state 置 COMPLETED 的方式——见下"终态"）。
    - FOLLOW_ROUTE 且 `formation_follow=True`：进入 `_follower_loop`（10Hz）：读 leader odom → target = leader_pos + formation_offset → 当前位姿偏差 → PI 输出补偿 → 发布为 30Hz setpoint；leader odom 超时 `_FOLLOW_POSE_TIMEOUT_S` → 返回 `MotionResult(False,"LEADER_LOST",...)`；到达容差稳定后返回 success。
    - FOLLOW_ROUTE leader：现有 `_publish_waypoints` 路径；但先在需要时插入纯垂直过渡（见下"分层编排"）。
    - MOVE_TO / FAULT_EXIT：先完成纯垂直过渡（若 |本机z - layer_z| > 0.5 且目标 xy 不在当前层则先发垂直目标 `(cur_x, cur_y, layer_z)` 给 ego 等其 COMPLETED），再发最终 goal/waypoints 给 ego。
  - 新增 `_vertical_transition` 与 `_follower_loop` 与 `_hover_loop` 内部方法。
  - 终态处理：驱动层当前依赖 C++ `exec_state`（IDLE→EXECUTING→COMPLETED/HOLD）。对 HOVER/FOLLOW_ROUTE follower 这类不经 C++ 规划的命令，由驱动层在成功/失败时向 C++ 节点发 `hold` 或复用现有机制把 `exec_state` 置为 COMPLETED（例如通过 `_issue_hold` 后再调度）。具体：驱动层在 HOVER/follower 成功时发布一次 `COMPLETED` 状态（沿用 `_on_state` 注入 `String("COMPLETED")` 的现有模式或通过新 topic 回写），保持 executor 状态机一致。
- 修改 `src/swarm_uav_executor/config/executor_defaults.yaml`：`ego_swarm` 段增加 `follower_p_gain: 1.0`、`follower_i_gain: 0.1`、`follower_limit_xy: 2.0`、`follower_limit_z: 1.0`、`layer_move_to: 15.0`、`layer_follow_route: 12.0`、`layer_fault_exit: 8.0`。
- 修改 `src/swarm_uav_executor/launch/uav_executor_ego.launch`：转发 `formation_offsets`（YAML 字典参数，可空）、`leader_odom_topic_prefix`（可空）。
- 修改 `src/swarm_uav_executor/src/swarm_uav_executor/executor.py`：无逻辑改动（`start_move_to` 已是唯一入口）；确认 HOVER/FOLLOW_ROUTE follower 返回的 MotionResult 与新终态一致。
- 修改 `src/swarm_uav_executor/launch/uav_executor_mock.launch`：不需要（mock 不实现真实运动，仅登记命令列表，恢复 4 命令硬编码已支持）。
- 修改测试：`test_validation.py`（follower 校验用例，不再断言 NOT_IMPLEMENTED）、`test_ego_swarm_driver.py`（新增 HOVER/follower/垂直过渡用例）、可能新增 `test_follow_route_follower.py`。
- 文档：`src/tcp_to_ros/CONTEXT.md` 增加"编队槽位/leader 参考/误差积分/高度分层"已定语义（若缺）；不修改 C++ 代码。

[Functions]

单句描述：新增/改造驱动与校验函数以支撑分层编排、follower PI 跟踪与 HOVER 冻结发布，全部位于 Python 执行层。

- 修改 `validate_task_request(request, identity, supported_commands)`（validation.py）：移除 follower NOT_IMPLEMENTED；按 `leader_id / formation_follow` 分支校验。
- 修改 `build_goal(request)`：MOVE_TO/FAULT_EXIT 填 `layer_z`；FOLLOW_ROUTE leader 填 12；follower 填 `formation_offset`。
- 修改 `EgoSwarmDriver.start_move_to(self, goal, cancel_event, deadline)`（ego_swarm.py）：按 command 分派到 `_plan_vertical_transition` / `_plan_horizontal` / `_follower_loop` / `_hover_loop`。
- 新增 `EgoSwarmDriver._plan_vertical_transition(self, layer_z, cancel, deadline)`：若 |now_z - layer_z| > 0.5，发垂直目标 `(cur_x, cur_y, layer_z)` 给 ego，等待 COMPLETED，返回 sub-result；失败原样向上抛。
- 新增 `EgoSwarmDriver._plan_horizontal(self, goal, cancel, deadline)`：调用 `_publish_goal` 或 `_publish_waypoints`（leader）并等待 COMPLETED。
- 新增 `EgoSwarmDriver._follower_loop(self, goal, cancel, deadline)`：10Hz 读 leader odom；target = leader_pos + offset；偏差 = target - own_pos；v = P*e + I*∫e；clamp 到 limit；把期望位置 `own_pos + v*dt` 发布到 setpoint_pub（30Hz）；leader 超时 → LEADER_LOST；到位稳定 → success。
- 新增 `EgoSwarmDriver._hover_loop(self, captured_pose, cancel, deadline)`：30Hz 发布冻结位置到 setpoint_pub；到位稳定 → success。
- 新增 `EgoSwarmDriver._on_leader_odom(self, msg)`：缓存 leader 位姿及单调时钟。
- `EgoSwarmDriver.from_ros_params` 读取新增参数。
- 校验层辅助：`_validate_follower_assignment(assignment, leader_id, identity)`（新函数，校验 leader_id 存在、waypoints/offset 合法）。

[Classes]

单句描述：`EgoSwarmDriver` 类扩展 follower/HOVER/分层能力，保持 `MotionDriver` 抽象接口不变。

- 修改 `EgoSwarmDriver`（ego_swarm.py）：新增成员 `_leader_pose`、`_leader_pose_mono_s`、`_formation_offsets`、PI 参数、`_hover_capture`；新增三个运行时方法（见 Functions）；`hold()` 保持语义（HOVER/follower 可由外部 `hold` 打断）。
- 不新增/删除其他类；`MotionDriver` 抽象、`MockMotionDriver`、`MavrosPositionDriver` 不变（MavrosPositionDriver 不参与这些命令，仍用于 MOVE_TO 回退）。

[Dependencies]

单句描述：无新增第三方包，仅新增对 `nav_msgs/Odometry`（leader odom）的消息依赖（已有 safe_valley_exp 引用，`swarm_uav_executor` package.xml/CMakeLists 需确认 `nav_msgs` 已声明）。

- `nav_msgs`：`swarm_uav_executor/package.xml` 与 `CMakeLists.txt` 已由 catkin 间接依赖（通过 mavros_msgs/geometry_msgs 传递），如需直接订阅 `nav_msgs/Odometry` 应显式声明 `nav_msgs` 为 build/exec 依赖（若缺则补）。
- 运行依赖：`ego_planner_driver`（已有）、`mavros_msgs`（已有）、`swarm_topology_bridge` 需在拓扑 `topology_group_a_sim.yaml` 中确认已桥接 `mavros/local_position/odom`（当前配置仅含 `mavros/state`/`uav_task_state`/`trajectory_intent`，**需新增 `mavros/local_position/odom` topic 桥接**，使 GCS_A→每个 UAV 及 UAV 间可收到 `/<leader>/mavros/local_position/odom`）。此为多 Master 部署前提。

[Testing]

单句描述：单元测试覆盖 follower PI/HOVER/分层编排/校验语义，并更新场景 2+ 验收使 FOLLOW_ROUTE follower 由失败转为通过。

- `test_validation.py` 扩展：follower 请求不再被 NOT_IMPLEMENTED 拒绝；`build_goal` 对 follower 填充 `formation_offset`、leader 不填；MOVE_TO/FAULT_EXIT `layer_z` 正确。
- `test_ego_swarm_driver.py`（或新 `test_follow_route_follower.py`）：
  - follower：注入模拟 leader odom，验证 target=leader+offset、PI 输出 clamped、leader 超时→LEADER_LOST；
  - HOVER：发布 `HOVER` goal，断言发布器持续输出冻结位置（前后两次相同），不跟随新自身位姿；
  - 分层：本机 z=5、goal z=15 时断言先收到垂直目标 `(cur_x,cur_y,15)` 并等 COMPLETED 后再收水平目标；
  - FAULT_EXIT 同理（先 8m 纯垂直，再水平）。
- 复用 `verification/ego_smoke/` 运行冒烟验证 HOVER/follower 单机回灌。
- 回归：`catkin build` + `catkin_test_results --all build` 保持 0 failure；`rostest swarm_uav_executor uav_executor_rostest.test` PASS。
- 端到端回归：按 `docs/test_record_26081021.md` 场景 2+ 重跑，预期 `coverage-segment-1/2` 的 follower 由 FAILURE 变为 SUCCESS（leader 现走 ego 已有 FEA，follower 走 PI 跟踪）；`fault-exit`/`hold-end` 亦转为 PASS（需 mock 支持返回 COMPLETED——mock 已支持 4 命令，驱动层命令语义在 mock 下仅返回 COMPLETED，因果自动化验收通过）。

[Implementation Order]

单句描述：按"配置/模型 → 校验 → 驱动能力 → launch/拓扑 → 测试与回归"顺序推进，每步可独立验证。

1. `models.py`：`MotionGoal` 加 `layer_z`/`formation_offset`；`ExecutorConfig` 加 PI/limit 字段。
2. `executor_defaults.yaml` + `uav_executor_ego.launch`：新增参数传递与默认值。
3. `validation.py`：移除 follower NOT_IMPLEMENTED；`build_goal` 填 `layer_z`/`formation_offset`；新增 follower 校验辅助。
4. `test_validation.py`：更新/新增 follower/层高/layer_z 断言，跑通。
5. `ego_swarm.py`：订阅 leader odom；新增强制分层/`_plan_vertical_transition`/`_plan_horizontal`/`_follower_loop`/`_hover_loop`；`start_move_to` 分派。
6. `test_ego_swarm_driver.py`（+follower 测试）：HOVER 冻结、follower PI/超时、垂直先行、FAULT_EXIT 两阶段，跑通。
7. `swarm_topology_bridge/config/topology_group_a_sim.yaml`：新增 `mavros/local_position/odom` topic 桥接（并同步 `swarm_topology_bridge` 侧若有独立拓扑副本）。
8. 构建 + `catkin_test_results --all build` 回归通过。
9. 单机（无 MAVROS）冒烟：HOVER、follower（假 leader）、分层、FAULT_EXIT。
10. SITL/多 Master 端到端：UAV1..A12（coverage）、A07..A12（fault-exit）、A01..A15（recovery/hold）重跑场景 2+；更新 `test_record_2608xxxx.md` 新测试记录并按用户约定命名。