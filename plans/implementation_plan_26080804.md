# Implementation Plan

[Overview]
在保持 GCS_A 现有整组分发语义稳定的前提下，抽离共享 ROS 接口并实现只控制本机的机载执行适配层，首版闭环支持 `MOVE_TO` 与安全 `HOLD`，同时为后续航线规划、编队和机间避碰预留驱动边界。

本计划对应 `docs/tcp_to_ros_后续开发计划.md` 的 Phase 4，并把 Phase 5 尚未完成的安全与弱网验证作为并行质量门禁，而不是继续机载开发的前置阻塞项。当前可以认为 GCS_A 的**核心开发链路已经达到机载接入基线**：整组拆分、并发确认、幂等、`ALL` 聚合、失败即停、单 Master 状态闭环以及跨 Master Service 冒烟均已通过。当前不能把 GCS_A 描述为“全部完善”或“最终验收完成”，因为跨 Master 状态 Topic 完整聚合、4 秒确认边界、Action cancel/TCP 断线触发 HOLD、弱网重连下 Service 超时与迟响应仍需补齐。

实现采用三个清晰边界：`tcp_to_ros` 继续作为 GCS_A 地面站业务包，只管理整组任务与聚合；新建 `swarm_uav_interfaces` 作为 GCS 与机载共同依赖的纯 ROS 接口包；新建 `swarm_uav_executor` 作为部署在每架 UAV 本机 ROS Master 中的执行适配包。共享接口不再由地面站业务包定义，从而避免机载程序反向依赖 `tcp_to_ros`。`swarm_topology_bridge` 仍只做通用序列化代理，不理解任务含义，也不取得任何无人机控制权。

首版机载执行器只实现 `MOVE_TO` 和独立安全 `HOLD`。每个执行器实例由启动参数绑定唯一 `(uav_id, exec_target)`，只接受与本机身份完全匹配的请求，只调用本机动作驱动，只发布本机任务状态。即使桥接网络允许看到其他 UAV 的 Service 或 Topic，本实例也不得转发、代执行或停止其他飞机。后续为避碰订阅其他飞机状态只授予“感知”能力，不授予跨机控制权。

动作层采用可替换 driver 端口。首版提供确定性 mock driver 和 direct MAVROS position-setpoint driver：mock 用于 CI、多 Master 和故障注入；MAVROS driver 用于 SITL/不上桨联调，持续发布本机目标位姿并根据本机定位反馈判断“到达且稳定”。后续 EGO Planner、`FOLLOW_ROUTE`、leader-followers 编队和机间避碰通过新增 driver 或上层协调模块接入，不改变冻结的 `UavTask`、`UavHold` 和 `UavTaskState` 跨机契约。

```text
runtime / Behavior Tree
        |
        | TCP JSON Lines：整组 COMMAND / STATUS
        v
+----------------------- GCS_A ROS Master ------------------------+
| tcp_to_ros                                                      |
|   tcp_server_node -> ExecuteGroupTask(Action, GCS 本地)          |
|                         |                                        |
|   gcs_a_backend_node ---+--- 拆分 / 并发确认 / ALL 聚合 / HOLD    |
|                         |                                        |
|   depends on swarm_uav_interfaces                               |
+-------------------------|----------------------------------------+
                          |
            swarm_topology_bridge（通用 Service/Topic 代理）
                          |
       +------------------+------------------+
       |                                     |
+------v------ UAV1 ROS Master -----+  +------v------ UAVn ROS Master -----+
| swarm_uav_executor                |  | swarm_uav_executor                |
| identity = A01 / UAV1             |  | identity = Ann / UAVn             |
| /UAV1/uav_task                    |  | /UAVn/uav_task                    |
| /UAV1/uav_hold                    |  | /UAVn/uav_hold                    |
| /uav_task_state -> bridge         |  | /uav_task_state -> bridge         |
|         |                         |  |         |                         |
| UavTaskExecutor                   |  | UavTaskExecutor                   |
|  + validator / idempotency        |  |  + validator / idempotency        |
|  + state reporter                 |  |  + state reporter                 |
|  + MotionDriver port              |  |  + MotionDriver port              |
|       + MockMotionDriver          |  |       + MockMotionDriver          |
|       + MavrosPositionDriver      |  |       + MavrosPositionDriver      |
|       + future EgoPlannerDriver   |  |       + future EgoPlannerDriver   |
|                 |                 |  |                 |                 |
|            local MAVROS/PX4 only  |  |            local MAVROS/PX4 only  |
+-----------------------------------+  +-----------------------------------+
```

安全边界如下：正常任务同一时刻最多一个；不同任务键在忙时被明确拒绝；相同任务键重复请求只返回当前状态或缓存终态，绝不再次启动动作；HOLD 始终高优先级，可抢占当前正常任务并锁存当前安全目标；Service 回调不得等待长动作完成；`accepted=true` 仅表示校验、幂等登记及动作入口接管成功；终态只能通过 `UavTaskState` 发布。任务本地超时、MAVROS 断开、定位失效、driver 异常均进入 `FAILED` 并尝试本机 HOLD，但 PX4/MAVROS 独立 failsafe 仍是实机前必须现场冻结的外部安全层。

本计划不实现 Group B、不实现完整跨机 ROS Action、不让 GCS_A 参与位置控制、不在首版实现 EGO Planner/`FOLLOW_ROUTE`/`FAULT_EXIT`/编队算法，也不以示例任务坐标进行上桨飞行。坐标系、原点、高度基准、yaw 单位与方向、稳定悬停判据、A01-A15 映射和 PX4 failsafe 未经人工复核时，只允许 mock、SITL 和不上桨联调。

[Types]
共享类型从 `tcp_to_ros` 迁入独立接口包，线上的字段与语义保持不变，只改变 ROS 类型包名。

### `swarm_uav_interfaces` 冻结消息与 Service

1. `swarm_uav_interfaces/Pose3DYaw.msg`
   - `float64 x`、`y`、`z`：目标或航点位置；单位与坐标系由部署配置冻结，消息本身不隐式换算。
   - `float64 yaw`：目标航向；首版内部统一按 rad 处理，实机输入约定必须现场确认。

2. `swarm_uav_interfaces/TaskAssignment.msg`
   - `string uav_id`：任务层身份，必须与请求顶层 `uav_id` 及本机配置一致。
   - `bool formation_follow`：为后续 follower 语义保留；首版 `MOVE_TO` 必须为 `false`。
   - `string target_id`：为目标任务保留；首版允许空字符串且不参与控制。
   - `Pose3DYaw target_pose`：`MOVE_TO` 的唯一目标位姿。
   - `Pose3DYaw[] waypoints`：为 `FOLLOW_ROUTE`/`FAULT_EXIT` 保留；首版 `MOVE_TO` 必须为空。

3. `swarm_uav_interfaces/UavTaskState.msg`
   - 原字段原序迁移：`protocol_version/mission_id/command_id/uav_id/exec_target/command/status/status_seq/detail_stage/error_code/message/reported_at`。
   - 主状态只允许 `ACCEPTED`、`COMPLETED`、`FAILED`；`COMPLETED` 和 `FAILED` 为终态。
   - `status_seq` 为同一任务键内严格单调递增的 `uint32`，重复发布同一状态也必须使用更大的序号；GCS_A 按任务键和序号去重。
   - `reported_at` 使用 ROS time 秒值；仿真必须兼容 `/use_sim_time`。

4. `swarm_uav_interfaces/UavTask.srv`
   - 请求字段原序迁移：`protocol_version/mission_id/group_id/command_id/uav_id/exec_target/command/timeout_s/leader_id/assignment`。
   - 唯一键固定为 `(mission_id, command_id, uav_id)`。
   - 公共校验：`protocol_version == "1.0"`、`group_id == "GroupA"`、ID 非空、`timeout_s > 0`、请求身份与本机身份完全一致、`assignment.uav_id == uav_id`。
   - 首版命令白名单只有 `MOVE_TO`；目标四个数值必须有限，waypoints 必须为空，`formation_follow == false`。
   - 响应保持 `bool accepted/string status/string error_code/string message`；`accepted=true` 只表示本机已接管，不表示运动完成。

5. `swarm_uav_interfaces/UavHold.srv`
   - 请求字段原序迁移：`protocol_version/mission_id/command_id/uav_id/exec_target/reason`。
   - HOLD 请求也必须校验协议、非空 ID 和本机身份；`reason` 仅记录，不按文案分支。
   - 响应保持 `bool accepted/string error_code/string message`；接受后稳定悬停结果通过被打断任务的 `UavTaskState` 报告。

### `tcp_to_ros` 本地类型

- `ExecuteGroupTask.action` 与 `GroupTaskState.msg` 仍属于 `tcp_to_ros`，因为它们是 GCS_A 本地整组业务契约，不供机载节点实现。
- `ExecuteGroupTask.action` 中 assignments 类型改为 `swarm_uav_interfaces/TaskAssignment[]`，字段和 TCP 映射不变。
- `tcp_to_ros/Pose3DYaw`、`TaskAssignment`、`UavTaskState`、`UavTask`、`UavHold` 在一次性迁移后删除，不保留同名双定义；所有参与节点必须在同一部署批次升级并重新构建，避免 MD5/type name 不匹配。

### `swarm_uav_executor` 内部数据结构

- `TaskKey(mission_id: str, command_id: str, uav_id: str)`：不可变、可哈希，唯一标识一次本机任务。
- `ExecutorIdentity(uav_id: str, exec_target: str, group_id: str = "GroupA")`：启动后不可修改；禁止从请求动态切换本机身份。
- `ExecutorConfig`：包含 `protocol_version`、driver 类型、Service/Topic 名称、frame、setpoint 频率、定位超时、位置/yaw 容差、稳定时长、终态重发参数、幂等缓存上限和是否允许自动切换 OFFBOARD/解锁；所有数值启动时校验为有限且位于安全范围。
- `TaskRecord`：包含 `key/request_fingerprint/command/status/status_seq/detail_stage/error_code/message/accepted_at/deadline_at/terminal_at/cancel_event`；状态迁移只允许 `NEW -> ACCEPTED -> COMPLETED|FAILED`。
- `MotionGoal(target_pose, frame_id, timeout_s)`：与 ROS 消息解耦的 driver 输入值对象。
- `MotionResult(outcome, detail_stage, error_code, message)`：`outcome` 只允许 `COMPLETED/FAILED/CANCELLED`；`CANCELLED` 由执行器映射为原任务 `FAILED`，例如 `COMMAND_HELD`。
- `HoldGoal(reason, captured_pose)`：driver 的本机悬停输入；不包含其他 UAV 身份。
- `DriverHealth(ready, connected, pose_fresh, mode, armed, message)`：供接收校验和故障状态使用。

### 首版错误码与细节阶段

- 校验/接收：`INVALID_PROTOCOL`、`WRONG_GROUP`、`IDENTITY_MISMATCH`、`INVALID_ASSIGNMENT`、`UNSUPPORTED_COMMAND`、`INVALID_TIMEOUT`、`EXECUTOR_BUSY`、`DUPLICATE_CONFLICT`、`DRIVER_NOT_READY`。
- 执行/安全：`MOVE_TIMEOUT`、`POSE_STALE`、`MAVROS_DISCONNECTED`、`SETPOINT_STREAM_FAILED`、`COMMAND_HELD`、`HOLD_FAILED`、`DRIVER_EXCEPTION`。
- `detail_stage` 首版使用：`VALIDATING`、`DISPATCHED`、`MOVING`、`SETTLING`、`HOLDING`、`TERMINAL`。枚举值写入 README 和测试，不在 bridge 内解释。

[Files]
新增两个独立 catkin 包并迁移共享接口，随后更新 GCS_A 的生成依赖、bridge 类型字符串、launch 与测试引用。

### 新建文件

#### `src/swarm_uav_interfaces/`（独立 Git 仓库、纯接口包）

- `.gitignore`：忽略 catkin/IDE/Python 临时产物，不忽略 msg/srv。
- `README.md`：记录接口语义、唯一键、状态机、版本兼容与升级要求。
- `CMakeLists.txt`：使用 `message_generation` 生成 3 个 msg 和 2 个 srv，导出 `message_runtime`。
- `package.xml`：ROS Noetic package format 2；只声明 `catkin`、`message_generation`、`message_runtime`、`std_msgs` 等接口所需依赖。
- `msg/Pose3DYaw.msg`、`msg/TaskAssignment.msg`、`msg/UavTaskState.msg`：按 `[Types]` 定义。
- `srv/UavTask.srv`、`srv/UavHold.srv`：按 `[Types]` 定义。

#### `src/swarm_uav_executor/`（独立 Git 仓库、机载应用包）

- `.gitignore`、`README.md`、`CMakeLists.txt`、`package.xml`、`setup.py`：catkin Python 包元数据、部署和安全说明。
- `config/executor_defaults.yaml`：无身份信息的通用阈值；身份必须由每机 launch/部署配置显式注入。
- `launch/uav_executor_mock.launch`：单机 mock 执行器，参数化 `uav_id/exec_target/namespace`。
- `launch/uav_executor_mavros.launch`：本机 MAVROS driver 入口；默认不自动 arm、不自动切 OFFBOARD。
- `scripts/uav_executor_node.py`：唯一 ROS 可执行入口，只负责参数装配、节点生命周期和安全退出。
- `src/swarm_uav_executor/__init__.py`：包入口。
- `src/swarm_uav_executor/models.py`：内部不可变类型、状态和错误常量。
- `src/swarm_uav_executor/validation.py`：公共字段、身份和 `MOVE_TO` assignment 校验。
- `src/swarm_uav_executor/task_store.py`：有界线程安全幂等缓存与请求指纹冲突检测。
- `src/swarm_uav_executor/state_reporter.py`：状态序号、ROS 消息转换及终态重发。
- `src/swarm_uav_executor/executor.py`：Service 接收、状态机、工作线程、超时和 HOLD 抢占编排。
- `src/swarm_uav_executor/drivers/__init__.py`。
- `src/swarm_uav_executor/drivers/base.py`：`MotionDriver` 抽象端口。
- `src/swarm_uav_executor/drivers/mock.py`：可配置延时、失败和 HOLD 的确定性测试 driver。
- `src/swarm_uav_executor/drivers/mavros_position.py`：本机 setpoint 发布、定位反馈、稳定判据与 HOLD。
- `test/test_validation.py`、`test/test_task_store.py`、`test/test_executor.py`、`test/test_mock_driver.py`、`test/test_mavros_position_driver.py`：单元测试。
- `test/uav_executor_rostest.test`、`test/test_uav_executor_ros.py`：Service/Topic 集成测试。

### 修改文件

#### `src/tcp_to_ros/`

- `CMakeLists.txt`：加入 `swarm_uav_interfaces` catkin 依赖；删除共享 msg/srv 的生成项；`ExecuteGroupTask.action` 继续在本包生成。
- `package.xml`：加入 `swarm_uav_interfaces` build/build_export/exec 依赖，移除已迁出接口对应的本地生成依赖但保留 Action 所需 `message_generation/runtime`。
- `action/ExecuteGroupTask.action`：assignment 类型由 `tcp_to_ros/TaskAssignment[]` 改为 `swarm_uav_interfaces/TaskAssignment[]`。
- `scripts/gcs_a_backend_node.py`：导入新包的 `UavTask/UavHold/UavTaskState`；业务算法和超时语义不变。
- `scripts/tcp_server_node.py` 及 `scripts/tcp_to_ros/*`：将 `Pose3DYaw/TaskAssignment` 导入切到新包；TCP JSON 契约不变。
- `scripts/uav_mock_node.py`：迁移期间先切换到新接口以做 A/B 对照；`swarm_uav_executor` mock 通过全部回归后删除该旧 mock。
- `scripts/task_event_injector.py`、`scripts/mock_backend_node.py`：只在实际引用迁出类型时更新 import，不改变行为。
- `config/topology_group_a_sim.yaml`：Service 和 Topic 的 ROS 类型字符串改为 `swarm_uav_interfaces/UavTask`、`swarm_uav_interfaces/UavHold`、`swarm_uav_interfaces/UavTaskState`；端口和路由保持不变。
- `launch/topology_group_a_sim.launch`：UAV 角色由 `tcp_to_ros/uav_mock_node.py` 切到 `swarm_uav_executor/uav_executor_node.py` 的 mock driver；GCS_A 角色行为不变。
- `launch/gcs_a_backend_test.launch`、`launch/gcs_a_backend_fail.launch`：用新执行器的 mock 配置替代旧 mock 节点，保留故障注入场景。
- `test/test_gcs_a_backend_client.py` 及引用 `TaskAssignment` 的测试：更新类型 import，并保持既有断言。
- `README.md`、`CONTEXT.md`、`docs/tcp_to_ros_后续开发计划.md`、`docs/tcp_to_ros_测试记录.md`：记录接口包所有权、Phase 4 实现和新增验证证据；Phase 编号仍只以后续开发计划为权威。

#### `src/swarm_topology_bridge/`

- 通用 bridge 代码原则上不改；仅其 README 或示例配置中若硬编码 `tcp_to_ros/Uav*`，改为新接口包类型名。
- 不向 `package.xml` 增加 `swarm_uav_interfaces` 业务依赖；动态类型解析继续由部署侧安装相同接口包满足。

### 删除或移动文件

- 在所有消费者完成同批迁移并通过构建后，删除：
  - `src/tcp_to_ros/msg/Pose3DYaw.msg`
  - `src/tcp_to_ros/msg/TaskAssignment.msg`
  - `src/tcp_to_ros/msg/UavTaskState.msg`
  - `src/tcp_to_ros/srv/UavTask.srv`
  - `src/tcp_to_ros/srv/UavHold.srv`
- `src/tcp_to_ros/msg/GroupTaskState.msg`、`action/ExecuteGroupTask.action`、`srv/EmergencyHold.srv` 不移动。
- 新 executor mock 完成等价回归后删除 `src/tcp_to_ros/scripts/uav_mock_node.py`，避免两套机载语义并存。
- 不移动或复用 `src/safe_valley_exp` 代码；它只作为算法与部署参考，后续通过 driver 适配而非复制耦合。

### 仓库与配置约束

- 在 `src/swarm_uav_interfaces/` 和 `src/swarm_uav_executor/` 内分别执行 `git init`，形成两个独立仓库；工作空间根目录不新建 Git 仓库，也不把它们登记为本仓 submodule。
- 两个新仓库分别维护自己的提交历史；首次提交使用用户规定的简洁 Conventional Commit 格式。
- 构建只使用 `catkin build`；不得直接改 `build/`、`devel/`、`logs/`。
- 多机必须安装并 source 完全相同 commit 的 `swarm_uav_interfaces`；启动日志打印接口包版本/commit（可由部署注入）与本机身份，便于现场审计。

[Functions]
新增函数围绕请求校验、幂等登记、非阻塞执行、状态发布和 driver 生命周期组织，GCS_A 原有聚合函数只做类型迁移。

### 新函数

#### `src/swarm_uav_executor/validation.py`

- `validate_task_request(request: UavTaskRequest, identity: ExecutorIdentity, supported_commands: Collection[str]) -> None`：执行公共字段、Group A、本机身份、timeout 和 assignment 一致性校验；失败抛出带稳定错误码的 `RequestValidationError`。
- `validate_move_to_assignment(assignment: TaskAssignment) -> MotionGoal`：验证 finite 坐标、首版禁用字段并转换为内部 goal。
- `validate_hold_request(request: UavHoldRequest, identity: ExecutorIdentity) -> None`：验证协议、ID 和本机身份，不解释 reason 文案。
- `task_key_from_request(request: UavTaskRequest) -> TaskKey`：只由冻结三元组构造键。
- `request_fingerprint(request: UavTaskRequest) -> str`：对规范化业务字段生成稳定摘要；同键不同内容返回 `DUPLICATE_CONFLICT`。

#### `src/swarm_uav_executor/task_store.py`

- `get(key: TaskKey) -> Optional[TaskRecord]`：线程安全读取当前/终态记录。
- `register(record: TaskRecord) -> RegisterResult`：原子登记；区分新任务、完全重复和冲突重复。
- `update(key: TaskKey, transition: TaskTransition) -> TaskRecord`：在锁内验证合法状态迁移并递增序号。
- `active() -> Optional[TaskRecord]`：返回唯一活动正常任务。
- `prune(now: float) -> None`：仅淘汰超过 TTL 的终态，绝不淘汰活动任务；同时遵守最大记录数。

#### `src/swarm_uav_executor/state_reporter.py`

- `publish(record: TaskRecord) -> None`：转换并发布 `UavTaskState`，保证 identity、任务键和序号一致。
- `publish_transition(key: TaskKey, status: str, detail_stage: str, error_code: str = "", message: str = "") -> TaskRecord`：原子更新后立即发布。
- `start_terminal_republish(record: TaskRecord) -> None`：按配置有限次重发终态，提高 Topic 至少一次可见性；不生成新动作。
- `shutdown() -> None`：停止 timer，避免节点退出后继续访问 publisher。

#### `src/swarm_uav_executor/drivers/base.py`

- `start_move_to(goal: MotionGoal, cancel_event: threading.Event, deadline: rospy.Time) -> MotionResult`：阻塞在执行工作线程中，不在 ROS Service callback 中运行。
- `hold(goal: HoldGoal, deadline: rospy.Time) -> MotionResult`：立即接管本机目标并等待稳定或超时。
- `health() -> DriverHealth`：无副作用地返回接收任务所需健康状态。
- `shutdown() -> None`：停止 setpoint/timer，进入可预测安全退出。

#### `src/swarm_uav_executor/drivers/mavros_position.py`

- `_pose_callback(message: PoseStamped) -> None`：缓存本机最新 pose 与接收时间。
- `_state_callback(message: mavros_msgs/State) -> None`：缓存连接、模式和 armed 状态。
- `_set_target(target: PoseStamped) -> None`：原子替换本机 setpoint。
- `_publish_setpoint(event: rospy.TimerEvent) -> None`：按配置频率持续发布当前目标，不因 Service 返回停止。
- `_target_error(current: PoseStamped, target: PoseStamped) -> Tuple[float, float]`：计算位置范数和归一化 yaw 误差。
- `_wait_until_stable(target: PoseStamped, cancel_event: threading.Event, deadline: rospy.Time) -> MotionResult`：要求误差连续落在阈值内达到 `settle_duration_s`；任一时刻越界则重置稳定计时。
- `_capture_hold_pose() -> PoseStamped`：只在定位新鲜时捕获本机当前位置；保留 frame 和当前 yaw。

#### `src/swarm_uav_executor/executor.py`

- `handle_task(request: UavTaskRequest) -> UavTaskResponse`：短时完成校验、去重、busy 判定、driver readiness、登记和线程提交；重复请求返回缓存状态；不得等待动作终态。
- `handle_hold(request: UavHoldRequest) -> UavHoldResponse`：高优先级校验并设置活动任务 cancel event，原子进入 HOLD 编排；重复 HOLD 安全幂等。
- `_run_task(key: TaskKey, goal: MotionGoal) -> None`：工作线程调用 driver，处理 deadline、异常和终态转换。
- `_run_hold(interrupted: Optional[TaskRecord], reason: str) -> None`：捕获本机位姿并调用 driver HOLD；成功后把被中断正常任务终结为 `FAILED/COMMAND_HELD`，失败则报告 `FAILED/HOLD_FAILED` 并记录高严重度日志。
- `_fail_active_and_hold(error_code: str, message: str) -> None`：用于本地 timeout、定位失效、MAVROS 断连等安全路径，确保只作用于本机。
- `shutdown() -> None`：拒绝新任务、请求本机 HOLD、等待有界时间并关闭 reporter/driver。

#### `scripts/uav_executor_node.py`

- `load_config_from_ros_params() -> Tuple[ExecutorIdentity, ExecutorConfig]`：强制读取显式身份并验证配置。
- `build_driver(config: ExecutorConfig) -> MotionDriver`：只允许注册表中的 `mock` 或 `mavros_position`，未知类型启动失败。
- `main() -> None`：初始化节点、装配依赖、注册本机 namespaced Service、安装 shutdown hook 并 spin。

### 修改函数

- `src/tcp_to_ros/scripts/gcs_a_backend_node.py` 中构造 `UavTaskRequest`、`UavHoldRequest` 和处理 `UavTaskState` 的现有函数：只替换生成类型 import，不改变 1.0 秒单次等待、首次加两次重试、0.2 秒间隔、4.0 秒整组 deadline、`ALL` 聚合和失败即停语义。
- `src/tcp_to_ros/scripts/tcp_server_node.py` 中 JSON assignment 到 ROS message 的现有转换函数：输出 `swarm_uav_interfaces/TaskAssignment`；字段验证和 TCP 状态格式不变。
- bridge 的 Service 请求/响应函数不修改签名或业务逻辑；通过 YAML 中的新类型字符串动态解析新生成类。

### 移除函数

- 删除 `uav_mock_node.py` 时一并移除其中 mock Service callback、延时终态线程和状态发布函数；迁移策略是在新 `MockMotionDriver + UavTaskExecutor` 通过原有正常、重复和失败即停用例后再删除。
- 不移除 GCS_A 的任何聚合、HOLD 或幂等函数。

[Classes]
新增一个单机应用服务、一个状态/缓存组合和可替换 motion driver 层，不修改 GCS_A 的领域职责。

### 新类

- `RequestValidationError`（`validation.py`）：携带 `error_code` 和可审计 message，不包含 ROS 发布副作用。
- `TaskStore`（`task_store.py`）：线程安全、有界、进程内幂等缓存；首版不承诺重启持久化，节点重启策略在 Phase 6 前另行冻结。
- `StateReporter`（`state_reporter.py`）：唯一允许生成 `status_seq` 和发布 `UavTaskState` 的组件，避免多个线程竞争序号。
- `MotionDriver`（`drivers/base.py`）：抽象基类/Protocol，定义 `start_move_to/hold/health/shutdown`；不得包含 ROS bridge 或其他 UAV 路由。
- `MockMotionDriver`（`drivers/mock.py`）：实现可控成功、失败、延时、取消、未就绪和 HOLD 失败，用于单元与集成测试。
- `MavrosPositionDriver`（`drivers/mavros_position.py`）：只连接本机 MAVROS namespace；维护连续 setpoint 流、pose/state 新鲜度和稳定完成判据。
- `UavTaskExecutor`（`executor.py`）：Service application layer；持有一个 identity、一个 `TaskStore`、一个 `StateReporter` 和一个 `MotionDriver`，保证单活动任务和 HOLD 抢占。

### 修改类

- `GcsABackendNode`（`src/tcp_to_ros/scripts/gcs_a_backend_node.py`）：仅把 ROS 类型依赖切到 `swarm_uav_interfaces`；不得将动作执行判据、MAVROS client 或 planner 逻辑加入该类。
- `TcpToRosServerNode`（`src/tcp_to_ros/scripts/tcp_server_node.py`）：仅更新 assignment message 构造类型，不改变 TCP 接口。
- `UavMockNode`、`TaskRecord`、`UavMockPolicy`（`src/tcp_to_ros/scripts/uav_mock_node.py`）：迁移窗口内只切换共享 ROS 类型；等价回归后随旧 mock 文件一起删除。

### 后续扩展类（本轮不实现）

- `EgoPlannerDriver(MotionDriver)`：把目标/航点提交给 EGO Planner，并用 planner/里程计结果形成终态；不得改变跨机 Service。
- `FollowRouteCoordinator`：在本机解释 leader 或 follower assignment；leader 执行 route，follower 使用本机感知与编队算法。其他 UAV 状态只作为输入，不产生跨机控制调用。
- `InterUavAvoidanceProvider`：汇总邻机状态并向 planner 提供动态约束；Group A 默认无环境障碍不等于无需机间避碰，尤其需覆盖编队位置交换时的冲突消解。

### 移除类

- `UavMockNode`、`TaskRecord`、`UavMockPolicy`（`src/tcp_to_ros/scripts/uav_mock_node.py`）：迁移完成后整体由 `UavTaskExecutor + MockMotionDriver` 取代；不保留兼容包装层。

[Dependencies]
新增两个本地 catkin 包依赖并复用 ROS Noetic/MAVROS 现有生态，不引入新的全局 Python 包或替代通信框架。

- `swarm_uav_interfaces`：`catkin`、`message_generation`、`message_runtime`；如 msg 未使用 Header，则不额外引入 `geometry_msgs`，保持接口最小化。
- `swarm_uav_executor`：
  - build/exec：`rospy`、`swarm_uav_interfaces`、`geometry_msgs`、`mavros_msgs`、`std_msgs`、`tf` 或 `tf.transformations`（只用于 quaternion/yaw 转换）。
  - test：`rostest`、`rosunit`；纯 Python 测试优先使用标准库 `unittest`，避免新增 pip 依赖。
- `tcp_to_ros`：新增 `swarm_uav_interfaces` 依赖；继续依赖 `swarm_topology_bridge`、Action/message 生成依赖。
- `swarm_topology_bridge`：不新增编译期业务包依赖；运行目标环境必须能解析配置声明的接口类型。
- 不新增 `multimaster_fkie`、multi-agent-suite、`nimbro_network`、完整 Action bridge 或全局安装项。
- MAVROS 和 PX4 的具体版本沿用现有机载环境；若 `mavros_msgs` 在 CI 主机缺失，包构建应明确失败或通过 catkin 条件隔离测试，不得以静默 stub 掩盖部署依赖。
- 新包目录各自初始化 Git，但不配置 remote、不执行 push；remote 和权限由用户后续决定。

[Testing]
采用“纯逻辑单测 → ROS 单 Master → 多 Master bridge → MAVROS SITL/不上桨 → Phase 5 安全弱网回归”的分层验证，任何实机上桨测试都必须等待现场参数与 failsafe 人工签字。

### 静态与构建验证

1. `catkin build swarm_uav_interfaces swarm_uav_executor tcp_to_ros` 成功；随后 source 工作空间。
2. `rossrv show swarm_uav_interfaces/UavTask`、`UavHold` 和 `rosmsg show UavTaskState` 字段与冻结文档完全一致。
3. 搜索源树确认不存在运行时代码继续导入 `tcp_to_ros.msg.TaskAssignment/Pose3DYaw/UavTaskState` 或 `tcp_to_ros.srv.UavTask/UavHold`。
4. bridge 配置中不再出现旧接口类型字符串；GCS_A 本地 Action 和 EmergencyHold 仍属于 `tcp_to_ros`。
5. 两个新目录分别具有独立 `.git`，工作空间根不新增仓库/submodule 关系。

### `swarm_uav_executor` 单元测试

- 正确 `MOVE_TO` 被接受且 Service 快速返回，随后状态严格为 `ACCEPTED -> COMPLETED`。
- 协议、Group、`uav_id`、`exec_target`、assignment ID、timeout、NaN/Inf、waypoints、formation flag 和未知 command 分别被稳定错误码拒绝，且 driver 从未被调用。
- 同键同内容在执行中返回当前 `ACCEPTED`，终态后返回缓存终态；driver 调用次数始终为 1。
- 同键不同内容返回 `DUPLICATE_CONFLICT`；不同键且已有活动任务返回 `EXECUTOR_BUSY`。
- 并发重复 Service 调用只原子登记一次，无双启动、无 status_seq 重复。
- HOLD 抢占活动任务，旧任务终态为 `FAILED/COMMAND_HELD`，driver 不再向旧目标推进；重复 HOLD 不造成不安全切换。
- 本地执行 timeout、取消、driver exception、pose stale、MAVROS disconnect 和 HOLD failure 均产生预期状态及日志。
- `status_seq` 严格递增；终态重复发布不改变终态内容且不会重启任务。
- TaskStore 只淘汰过期终态，不淘汰活动项；容量达到上限时行为可预测。
- MAVROS driver 测试使用 fake publishers/callback 注入验证：setpoint 频率、frame/yaw 转换、稳定窗口重置、定位新鲜度、取消响应和 shutdown。

### 单 Master ROS 集成

- 启动一个 mock executor，确认仅注册其本机 `/UAV1/uav_task`、`/UAV1/uav_hold`，状态发布到配置 Topic。
- 尝试向 UAV1 实例发送 A02/UAV2 请求，必须 `IDENTITY_MISMATCH`，且无状态或控制副作用。
- 用 `gcs_a_backend_test.launch` 替换旧 mock 后复跑：正常三机 `MOVE_TO -> COMPLETED`、重复 command 返回缓存终态、UAV2 失败触发 A01/A03 HOLD，结果与现有测试记录一致。
- 验证 Service callback 返回时间不随动作时长增长，`accepted=true` 后由 Topic 提供终态。

### 多 Master/bridge 集成

- 每个 UAV Master 启动独立 executor + bridge，GCS_A Master 启动 bridge + backend；GCS_A 上代理 Service 数量和命名正确。
- 完成真实状态 Topic 跨 Master 闭环：UAV `ACCEPTED/COMPLETED/FAILED` 经 bridge 变为 `/UAVn/uav_task_state` 并被 GCS_A 聚合。
- 验证 1.0 秒单次等待、首次 + 2 次重试、0.2 秒间隔和 4.0 秒整组决策边界；迟到响应不得复活失败任务。
- 注入状态重复、乱序、slow-joiner、bridge 重启和短时断网；GCS_A 按 key/seq 去重，executor 不重复动作。
- Action cancel、TCP 断线和任一机失败必须对所有已接受且未终止 UAV 发出 HOLD，并逐机记录确认失败。

### MAVROS SITL 与不上桨验证

- driver 在 MAVROS 未连接、pose 过期或配置 frame 不合法时拒绝正常任务，不虚报 ACCEPTED。
- 在已由外部安全流程完成 mode/arm 条件时，持续 setpoint 频率满足 PX4 OFFBOARD 要求；默认配置不得自行 arm。
- `MOVE_TO` 只有位置误差和 yaw 误差连续满足阈值达到稳定时长才 `COMPLETED`；瞬时穿越容差不算完成。
- HOLD 捕获本机当前 pose 并持续发布，原 MOVE_TO 不再恢复；定位丢失时走 PX4/MAVROS 独立 failsafe 并报告 `HOLD_FAILED`。
- 记录 rosbag/日志，包含任务键、本机 identity、状态序号、target/current error、MAVROS health 和 HOLD 原因，但不得高频刷完整 payload。

### 验收门禁

- **允许开始 Phase 4 实现**：现有 GCS_A 基线已满足，无需等待所有 Phase 5 场景完成。
- **允许声明 Phase 4 首版完成**：共享接口迁移、mock/direct MAVROS driver、身份隔离、幂等、MOVE_TO/HOLD 单元及单 Master 测试全部通过。
- **允许声明 GCS_A + 机载仿真闭环完成**：跨 Master Topic、4 秒边界、cancel/断线 HOLD、弱网超时/迟响应全部通过并写入测试记录。
- **允许实机上桨**：现场复核 IP/hostname/ID 映射、坐标系/yaw/阈值、MAVROS namespace、PX4 failsafe、急停职责和测试场地；在此之前禁止使用示例坐标直接飞行。

[Implementation Order]
按“先冻结共享契约、再建立可测执行内核、最后切换 GCS 和开展跨 Master/MAVROS 验收”的顺序实施，以避免接口双定义和安全路径不可测。

1. **建立变更基线与迁移清单**：保存现有 `tcp_to_ros` 构建/单 Master/跨 Master Service 冒烟结果；确认工作树无无关修改，不触碰 build/devel/logs。
2. **创建 `swarm_uav_interfaces` 独立仓库与 catkin 包**：原样迁移 3 个共享 msg 和 2 个 srv，编写契约 README，单独构建并用 `rosmsg/rossrv show` 校验。
3. **一次性迁移消费者类型**：更新 `tcp_to_ros` Action、CMake/package、Python imports、bridge YAML 和测试；构建所有消费者后删除旧共享定义，确保没有双类型残留。
4. **创建 `swarm_uav_executor` 独立仓库骨架**：完成 package 元数据、配置、内部 models、validation、TaskStore、StateReporter 和 `MotionDriver` 端口；此阶段不连接 MAVROS。
5. **实现并验证 mock driver**：实现非阻塞 Service、单活动任务、身份隔离、幂等/冲突重复、状态序号、timeout 和 HOLD 抢占；先完成纯单元与 executor rostest。
6. **替换旧机载 mock**：更新 GCS_A integration launch，复跑正常、重复、单机失败即停；结果等价后删除 `tcp_to_ros/uav_mock_node.py`。
7. **补齐跨 Master Phase 5 主闭环**：用新 executor 验证状态 Topic 聚合、4 秒确认边界、状态乱序/重复、cancel/TCP 断线 HOLD；问题按 GCS、bridge、executor 责任边界修复。
8. **实现 direct MAVROS driver**：先完成 fake callback 单测，再做 SITL；默认不自动 arm/mode，完成连续 setpoint、健康检查、稳定判据、HOLD 和 shutdown。
9. **执行不上桨安全联调**：验证 MAVROS 断连、pose stale、任务 timeout、HOLD failure 与 PX4 failsafe 交互；冻结现场阈值和职责后才评估上桨。
10. **完成文档与证据回写**：更新 CONTEXT、后续计划 Phase 4 状态、README、测试记录和部署步骤；明确 GCS_A 尚余/已完成的 Phase 5 项。
11. **独立提交两个新仓库及相关消费方变更**：提交内容同时覆盖用户冻结决策与实现产生的改动，使用规定的英文 Conventional Commit header；不配置 remote、不 push。
12. **进入后续能力设计（不包含在首版实现）**：以 `MotionDriver` 为稳定边界设计 EGO Planner driver、`FOLLOW_ROUTE`、leader-followers 编队和 Inter-UAV avoidance；先增加接口/仿真测试再接实机，不修改已冻结跨机服务语义。
