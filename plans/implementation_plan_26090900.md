# 实施计划：runtime/当前任务 UAV 集群 brake hold

- **计划文件**：`implementation_plan_26090900.md`
- **生成时间**：2026-09-09 00 时（文件名按 `plans/README.md` 的 `implementation_plan_yymmddhh.md` 规则）
- **目标包**：`tcp_to_ros`，联动 `swarm_uav_executor`；必要时仅补充现有状态字段传播
- **状态来源**：实施状态不写在本文件，统一见 `plans/README.md`「实施计划（现存）」与「状态-26090900」
- **基线**：当前整体回溯后的源码；不把 `implementation_plan_26090802.md` 中已回退的实现视为当前能力

## 0. 决策修订（2026-09-09 对话结论，实施前必读）

本版吸收评审结论，明确三件事：

1. **C++ 结构按计划原文实施**：统一所有安全出口到“直接 `BRAKE_HOLD`”，删除
   `EmergencyBrakeGenerator` / `EMERGENCY_BRAKE` 中间态与制动轨迹执行分支；即保留
   `implementation_plan_26090802.md` 变更 D 的结构方向（不恢复其变更 B/C/E 与旧组级广播实现）。
2. **触发判据（关键修订，反 26090802 变更 E）**：prepare 复测在含 26090802 时为 **1/5**、
   回滚后 **4/5**；复盘认定退化主因是“emergency 锁存判据改为实时位置唯一 + 预测不参与 +
   无 replan 余地 → 过易触发 brake”，而非“直接锁存结构”本身。故本计划锁存触发必须：
   - 以 **SafetyPredictor 预测**为主判据：`min_d ≤ protected_distance_m(1.0)` 或
     `ttc ≤ emergency_ttc_s(1.5)` 才进入直接锁存判定；
   - 保留 **replan/让路余地**：预测级 EMERGENCY 先置位并唤醒 replanOnce，先做常规
     replan/让路绕开；成功即清除紧急标志继续执行、不进锁存；仅当绕开失败且预测仍紧急
     （或 ttc 已不可避）才 `lockBrakeHoldLocked`；
   - **实时 3D 位置只作最后兜底**：executor `MIN_DISTANCE_BREACH` 保留，仅捕获预测失效/
     滞后的极端情况，仍属 group fan-out 白名单。
3. **目标集合（用户拍板）**：仅当前任务参与者——组级 fan-out 不自动包含 A13–A15 reserve；
   runtime 指定 HOLD 也按 JSON assignments 与活动任务参与者交集下发。

## 1. 目标与范围

本计划实现两类安全 HOLD：

1. **runtime 指定集群 HOLD**
   runtime 按实物协议第 7 节发送 `command=HOLD`，指定 `assignments[].uav_id`。`tcp_to_ros` 取消旧普通任务，并仅向指定 UAV 路由安全 HOLD。

2. **当前任务集群安全收口**
   当前活动任务中的任一参与 UAV 上报明确、结构化的安全终态后，GCS_A backend 识别该事件，向同一任务中仍需安全收口的其他参与 UAV 扇出 `UavHold`。目标集合以当前任务实际参与者为准，不自动包含 A13–A15 reserve，也不影响其他任务或 group。

本计划只做跨层安全命令和状态收口，不重构 planner 或轨迹算法。

### 明确不在本次范围

- 不修改 `ego_planner_driver` 的普通 replan、clearance、轨迹优化或 `t_off/schedule_tau`；将所有安全 HOLD 入口统一到复用现有当前位置捕获/锁存 setpoint 的直接 `BRAKE_HOLD` 路径。
- 不恢复 26090802 中已回退的整体安全策略（变更 B/C/E 与旧组级广播实现）；本计划保留其变更 D 的“安全状态直接锁存”结构方向，但触发判据按 §0 修订：以 SafetyPredictor 预测为主、保留 replan/让路余地，实时位置仅作兜底。
- 不新增 ROS msg/srv；优先复用 `EmergencyHold.srv`、`UavHold.srv`、现有 `UavTaskState` 和当前 C++ 状态/位置锁存能力。
- `HOVER` 保持常规、可由后续任务退出的悬停；runtime 的 TCP `HOLD` 在机载安全路径中解释为 brake hold，不得降级为普通 `HOVER` 或普通可退出 `HOLD`。
- 不新增 executor 全局安全锁存或解除锁存协议。这里的“锁存”是当前安全动作持续发布自身当前位置的 `BRAKE_HOLD` setpoint，不等同于永久禁止所有后续任务；永久锁存应另立带权限和复位语义的设计。

## 2. 当前源码事实

### 2.1 实物协议与 TCP server

实物协议：
`references/docs/实物无人机上下层TCP-IP接口对接方案.md` 第 7 节（约 482–524 行）。

HOLD JSON 必须包含公共命令字段，并使用如下结构：

```json
{
  "version": "1.0",
  "type": "COMMAND",
  "mission_id": "joint-mission-002",
  "group_id": "GroupA",
  "command_id": "hold-0123456789abcdef",
  "command": "HOLD",
  "timeout_s": 5.0,
  "assignments": [
    {"uav_id": "A01"},
    {"uav_id": "A02"}
  ],
  "reason": "runtime requested safe hold"
}
```

协议要求地面站停止旧任务并调用安全悬停；可以先反馈 `ACCEPTED`，稳定后反馈 `COMPLETED`。TCP 断线时，地面站也必须执行相同的本地安全收口。

当前实现：

- `src/tcp_to_ros/scripts/tcp_to_ros/protocol.py:66-168` 已支持 `HOLD`；HOLD assignment 只解析 `uav_id`。
- `src/tcp_to_ros/scripts/tcp_server_node.py:141-225` 先查 command cache，再让 HOLD 优先于普通活动命令冲突。
- `tcp_server_node.py:235-288` 会 suppress 当前任务、取消 Action、缓存原任务 `FAILED/COMMAND_HELD`，随后调用 `/group_a/emergency_hold`。
- 当前缺陷：`_handle_hold()` 忽略 `EmergencyHoldResponse.accepted`，即使 backend 返回 `accepted=false` 也会构造 TCP `COMPLETED`，并把请求全集固定写为 `completed_uavs`。
- `EmergencyHold.srv` 当前只有 `accepted/message`，无法准确返回部分 held/rejected UAV 列表。
- `tcp_server_node.py:384-414` 的断线路径已有 suppress/cancel/emergency_hold；服务异常目前主要记录日志，TCP 已断开时不能承诺远端反馈。

### 2.2 GCS_A backend

- `src/tcp_to_ros/scripts/gcs_a_backend_node.py:1218-1312` 的 `_safe_hold()` 已实现批量、并发、超时/重试和跨 master `/{exec_target}/uav_hold` 调用。
- 当前 `_safe_hold()` 目标是 `started_uavs` 中尚未 terminal、未处于 `hold_pending` 的活动参与者。
- `gcs_a_backend_node.py:1314-1349` 的 `_handle_emergency_hold()` 已校验活动任务和 `mission_id`，设置 `stop_event`，然后调用 `_safe_hold()`。
- 当前缺陷：`request.uav_ids` 没有传入 `_safe_hold()`，所以指定 HOLD 可能实际作用于活动任务的其他成员；成功判定却使用完整 `state.uav_ids`，与实际操作集合不一致。
- `GroupState` 已保存 `uav_ids`、`exec_by_uav`、`started_uavs`、`terminal`、`held`、`hold_rejected`、`stop_event` 等字段，可复用为冻结的任务参与快照。
- backend 已通过 `_on_uav_state()` 和状态序号逻辑吸收单机任务状态，但当前没有“单机安全终态触发同任务其他 UAV HOLD”的一次性传播机制。

### 2.3 executor 与机载 driver

- `src/swarm_uav_interfaces/srv/UavHold.srv` 已提供：`protocol_version`、`mission_id`、`command_id`、`uav_id`、`exec_target`、`reason`；响应为 `accepted/error_code/message`。
- `src/swarm_uav_executor/src/swarm_uav_executor/executor.py:97-106` 的 `handle_hold()` 已有瞬时 `_hold_in_progress` 互斥，并异步启动 `_run_hold()`。
- `executor.py:149-159` 的 `_run_hold()` 调用 `driver.hold()`；有活动任务时反馈 `FAILED/detail_stage=HOLD/COMMAND_HELD`，底层 HOLD 失败时反馈 `HOLD_FAILED`。
- `src/swarm_uav_executor/src/swarm_uav_executor/drivers/ego_swarm.py:561-583` 的现有 `hold()` 会发布 `/hold`，等待 C++ `exec_state` 为 `HOLD` 或 `BRAKE_HOLD`；安全 reason 需要在该调用链中选择直接 brake hold 入口，而不能沿用普通 `/hold` 语义。
- 当前 executor/driver 没有持久安全锁存。`_hold_in_progress` 仅覆盖 HOLD 正在执行的时间窗口；本计划新增的机载“锁存”特指在 C++ driver 内锁存本机当前位置并持续发布 `BRAKE_HOLD` setpoint，不是 executor 的永久任务锁。
- `src/ego_planner_driver/include/ego_planner_driver/ego_planner_driver_node.h` 与 `src/ego_planner_driver/src/ego_planner_driver_node.cpp` 当前存在 planner 安全监督、`EMERGENCY_BRAKE` 状态和制动轨迹生成流程。本计划把“进入锁存”的判定统一为：SafetyPredictor 预测紧急（`min_d ≤ protected_distance_m` 或 `ttc ≤ emergency_ttc_s`）且 replanOnce 绕开失败/ttc 不可避 → 直接捕获本机当前位置、设置 `steady_target_`、进入 `BRAKE_HOLD`；删除 `EMERGENCY_BRAKE` 轨迹生成/执行分支。实时 3D 门禁仅在 executor 侧作最后兜底（见 §4.4）。
- 用户已确认各机速度不大，安全 HOLD 时不额外设计速度平滑或设定值突变缓解；前提是捕获并发布无人机自身当前有效位置，由 PX4 位置控制器稳定悬停。

## 3. 身份和状态约束

### 3.1 身份边界

```text
任务层：A01..A15
    ↓ IdentityMap
执行层：UAV1..UAV15 / exec_target
    ↓ 跨 master UavHold
机载 executor/driver/C++/PX4
```

- TCP JSON 的 `assignments[].uav_id`、TCP `completed_uavs`、`failed_uavs` 使用任务层 Axx。
- backend 内部使用 `IdentityMap` 把 Axx 转为 `exec_target/UAVn`。
- `UavHold.srv` 请求的 `uav_id` 与 `exec_target` 使用执行层身份。
- backend 对 Action/TCP 的结果再映射回 Axx。
- `mission_id`、`command_id` 必须贯穿 HOLD 请求，但第一阶段保持现有 executor 校验范围，不凭空新增任务状态协议。

### 3.2 HOLD 与悬停状态

| 类型 | 入口 | 目标集合 | 机载动作 | 是否经过 PREPARE/START |
|---|---|---|---|---|
| runtime 指定安全 HOLD | TCP `command=HOLD` | JSON assignments 指定的 Axx | 捕获自身当前位置，直接锁存并进入 `BRAKE_HOLD` | 否 |
| 当前任务组安全 HOLD | 单机任务结构化安全终态 | 同一 active GroupState 中仍需收口的参与者 | 各机捕获自身当前位置，直接锁存并进入 `BRAKE_HOLD` | 否 |
| 普通任务 `HOVER` | TCP `command=HOVER` 或普通任务动作 | 普通任务目标 | 常规可退出悬停，不触发 group safety 传播 | 按普通任务流程 |
| 普通任务失败回退 HOLD | executor motion failure | 当前单机任务 | 继续复用现有 executor/driver 失败回退语义，是否进入 brake hold 不由本计划改变 | 否 |
| C++/SafetyPredictor 安全事件 | planner 监督/碰撞检查 | 本机 | 预测紧急（`min_d ≤ 1.0`/`ttc ≤ 1.5`）→ 先 replan/让路绕开；失败才直接锁存 `BRAKE_HOLD`（实时 3D 仅兜底） | 不适用 |

本次统一的是所有安全来源的“直接 `BRAKE_HOLD`”出口：runtime `HOLD`（指定目标）、当前任务组级安全传播、C++ 预测紧急 + replan 失败（planner 监督）都直接捕获本机当前位置并锁存 setpoint，不再经过 `EMERGENCY_BRAKE` 轨迹生成。**注意触发顺序**：planner 侧进入锁存前必须保留一轮 replan/让路绕开；实时位置只在 executor 兜底路径（`MIN_DISTANCE_BREACH`）触发，二者关系见 §0。普通 `HOVER` 仍是常规可退出悬停；普通任务失败回退按 executor 现有收口语义，不由本计划改变。

## 4. 最小改动方案

### 4.1 修正 runtime 指定 HOLD

**文件：**

- `src/tcp_to_ros/scripts/tcp_server_node.py`
- `src/tcp_to_ros/test/test_hold_interrupt.py`
- 必要时新增同目录单元测试，不新增接口

**步骤：**

1. 保持现有处理顺序：解析 → cache 去重 → HOLD 分支 → 普通命令活动冲突检查。
2. 保持 suppress/cancel 旧普通任务先于安全 HOLD 调用；安全 HOLD 不创建普通 Action 任务，不发送 PREPARE/START。
3. 处理 `EmergencyHoldResponse.accepted`：
   - `true`：TCP `status=COMPLETED`，表示指定 UAV 已接受并完成直接 brake hold；`completed_uavs` 为原 HOLD assignment 的 Axx 集合。
   - `false`：TCP `status=FAILED`，错误码固定为 `HOLD_REJECTED`，`failed_uavs` 使用原 HOLD assignment 的 Axx 集合，message/detail 保留 backend 原因。
   - `rospy.ServiceException`：继续使用 `HOLD_SERVICE_FAILED`。
4. `UavHold` 的 `reason` 采用内部安全语义（例如 `RUNTIME_BRAKE_HOLD`），但 TCP 对外仍保持 `command=HOLD`；不新增 `BRAKE_HOLD` TCP command。
5. 所有终态写入 command cache；同一 `command_id` 的重复请求返回第一次缓存结果，不被后续回调覆盖。
6. 因 `EmergencyHold.srv` 没有 held/rejected 列表，第一阶段不构造虚假的部分完成列表。backend 拒绝时采用保守全请求失败集合，并将具体未确认信息写入 message/detail。
7. 断线时保留本地 suppress/cancel/emergency_hold；对 `accepted=false` 和异常分别记录可搜索的 `HOLD_REJECTED`/`HOLD_SERVICE_FAILED` 日志或状态。TCP 已断时不声称 runtime 已收到结果。

### 4.2 修正 backend 的指定目标集合

**文件：**

- `src/tcp_to_ros/scripts/gcs_a_backend_node.py`
- `src/tcp_to_ros/test/test_gcs_a_aggregation.py`

**步骤：**

1. `_handle_emergency_hold()` 首先读取 active `GroupState`，校验 `mission_id`。
2. 对 `request.uav_ids` 去重，并与 `state.uav_ids` 求交集；请求外成员不下发 HOLD，并记录为不匹配。
3. 为 `_safe_hold()` 增加可选 `targets` 参数。保留既有筛选：必须是 started、未 terminal、未 pending；保留 batch size、confirm timeout、retry count 和 retry interval。
4. 成功判定只针对本次请求的有效目标集合：
   - 已在 `state.held` 的目标按幂等成功处理，不重复发服务；
   - 未 started、已 terminal、失联或不在 active group 的目标不得默认为成功；
   - 没有可执行目标时返回拒绝或未确认，message 明确原因。
5. 不在持有 `state.lock` 时同步调用跨 master ROS service。
6. 对指定 HOLD 使用任务层 Axx 作为 request 目标；构造 `UavHoldRequest` 时通过 `state.exec_by_uav` 转换到 UAVn/exec_target。
7. 当前任务组安全 HOLD 调用同一 `_safe_hold(targets=...)`，但 targets 来自冻结的活动任务集合，而不是 runtime 任意请求集合。

### 4.3 单机安全终态触发当前任务集群 HOLD

**文件：**

- `src/tcp_to_ros/scripts/gcs_a_backend_node.py`
- `src/tcp_to_ros/test/test_gcs_a_aggregation.py`
- 只有在确认状态字段未正确传播时，才最小修改 executor 终态上报代码

**事件识别原则：**

1. 只使用经过 `mission_id`、`command_id`、Axx/UAVn 身份和 `status_seq` 校验的 `UavTaskState`。
2. 只处理 `status=FAILED` 且 `error_code` 属于有限白名单的安全接管事件；禁止按 message 文本、进程日志或任意 FAILED 模糊触发。
3. `GroupState` 增加一次性防重字段，例如 `group_safety_hold_started`、触发 UAV、错误码和 reason；使用安全 reason `GROUP_BRAKE_HOLD:<error_code>:<trigger_uav>` 调用同一 `UavHold` 路径。
4. 该路径在 executor/driver 侧选择直接 brake hold：每架目标机捕获自身当前位置和 yaw，清除普通轨迹，锁存并发布自身当前位置 setpoint，直接进入 `BRAKE_HOLD`；不经过 `EMERGENCY_BRAKE`。
5. 状态回调只快速置位/排队，使用现有 backend 线程模型异步调用 `_safe_hold(targets=...)`，不能在 subscriber 回调持锁等待跨 master service。
6. 目标集合为当前活动任务中已 accepted/started、未 terminal、未失联的参与 UAV；排除已进入安全收口的触发者、已完成/失败/失联成员。A13–A15 reserve 不在集合内。
7. 触发后设置现有 `stop_event`，阻止普通 Action 继续推进；不新增全局永久安全锁存。
8. worker 完成后更新 `held/hold_rejected/message`，让现有 Action 等待/失败收口路径报告真实结果；不能把部分拒绝报告为全组成功。
9. 重复或乱序 `status_seq` 必须在现有状态吸收层被丢弃，不得再次发起 group HOLD。

### 4.4 executor/driver/C++ 的直接 brake hold 路由

**文件：**

- `src/swarm_uav_executor/src/swarm_uav_executor/executor.py`
- `src/swarm_uav_executor/src/swarm_uav_executor/drivers/ego_swarm.py`
- `src/ego_planner_driver/include/ego_planner_driver/ego_planner_driver_node.h`
- `src/ego_planner_driver/src/ego_planner_driver_node.cpp`
- 对应 executor/driver/C++ 单元测试

**步骤：**

1. 保留 `UavHold.srv` 和现有 `_hold_in_progress` 瞬时互斥；不新增消息或服务。
2. executor 将安全 `reason` 原样传给 driver；至少区分 `RUNTIME_BRAKE_HOLD`、`GROUP_BRAKE_HOLD` 与普通内部失败回退原因。
3. `EgoSwarmDriver.hold()` 对上述安全 reason 选择新的直接 brake hold 路径；普通 `HOVER` 不进入该路径。
4. C++ driver 的直接 brake hold 入口（driver 线程内）仅在满足 §0 判据时调用：SafetyPredictor 预测紧急（`min_d ≤ protected_distance_m` 或 `ttc ≤ emergency_ttc_s`）且 replanOnce 常规 replan/让路绕开失败、或 ttc 已不可避。入口动作：捕获本机当前有效位置和 yaw，清除普通 `current_`/待执行轨迹，设置 `steady_target_`，立即发布/锁存固定位置 setpoint，并进入 `BRAKE_HOLD`。
5. planner 监督路径保留“预测 → 重规划余地”：预测级 EMERGENCY 只置位并唤醒 replanOnce，先尝试常规 replan/让路绕开；成功清除紧急标志继续执行，不进入锁存。仅绕开失败才调用第 4 步入口。`EmergencyBrakeGenerator`、`EMERGENCY_BRAKE` 中间态与制动轨迹执行分支删除，不再作为兜底产出。
6. 直接 brake hold 的固定 setpoint 必须来自该 UAV 自身捕获的当前位置和 yaw，不使用 runtime 目标位置、邻机位置或伪造原点；按用户确认，不额外增加速度平滑或设定值突变处理。
7. 直接进入 `BRAKE_HOLD` 后，现有 `exec_state` 反馈用于 executor/backend 确认；确认失败返回 `HOLD_FAILED`，不能向上层报告已完成。
8. 本阶段不实现永久安全锁存：`BRAKE_HOLD` 的位置 setpoint 持续锁存仅表示当前安全动作；后续普通任务是否允许覆盖仍按现有生命周期，若需“不可转出”必须另行设计。
9. 若当前普通 `/hold` topic 无法携带安全模式，新增最小的机载内部模式路由（例如由 executor/driver 现有调用路径区分 reason 后调用一个最小 brake-hold 入口）；不得用字符串日志猜测安全模式，也不得把 TCP `HOLD` 改成新 command。
10. 只有发现某个白名单错误码没有进入 `UavTaskState.error_code` 时，才补该现有终态字段传播；不新增独立安全状态 topic。
11. executor 实时 3D 距离门禁（`MIN_DISTANCE_BREACH`，现有 EGO 巡航与 follower 两处内联判定）保留为**最后兜底**触发源：仅当预测/规划未能提前解危且实时中心距破门时产生该终态；其语义与 group fan-out 白名单保持不变（§0）。不新增第二套实时判据参数，也不把实时判据用作主锁存触发。


**并发边界：**

- 不在持有 executor/backend 状态锁时等待 ROS service 或 C++ 状态确认。
- 可单独评估 daemon 线程清理 `_cancel` 的 token ownership；无直接回归证据时不与本功能捆绑。
- 安全 HOLD 与普通任务 PREPARE/START 解耦；安全命令直接作用于已运行或指定的机载控制链路。

## 5. 并发、幂等与超时

- TCP HOLD 优先于普通命令；同一 `command_id` 终态缓存不可被覆盖。
- 单机 executor 同时只允许一个 `_hold_in_progress`；重复 UavHold 返回已有 HOLD 活动语义。
- 同一 `GroupState` 同时只允许一次自动 group safety fan-out；重复错误状态不重复发送。
- backend 复用现有 HOLD 批处理和重试参数，不新增第二套跨 master 通道。
- service exception、超时、`accepted=false` 都是未确认/失败语义；不得返回 TCP `COMPLETED`。
- 普通任务 PREPARE/START 与安全 HOLD 解耦；runtime 不需要再次确认 HOLD 才能触发安全动作。
- 组级安全 HOLD 触发后，不允许继续向未启动成员推进普通 START；具体 Action 终态必须沿现有 `stop_event`/失败聚合路径验证。
- 不把 reserve UAV 自动加入当前任务安全集合；若未来要求全 15 机收口，应另行定义 reserve 参与规则和协议。

## 6. 结果和可观测性

### 对外结果

- TCP `completed_uavs`、`failed_uavs` 只使用任务层 Axx，且只能来自该 HOLD 命令的原 assignments。
- `EmergencyHold.srv`/`UavHold.srv` 的 service response 使用现有字段；单机内部日志可使用 UAVn。
- 若 backend 无法表达部分 held/rejected 列表，采用保守 `FAILED`，并在 message/detail 中列出未确认目标。
- TCP 合法状态只使用 `ACCEPTED`、`COMPLETED`、`FAILED`；不要把 `RUNNING`、`SUCCESS`、`FAILURE` 作为 TCP 终态。

### 必须记录的字段

```text
mission_id
command_id
trigger_error_code
trigger_uav(Axx/UAVn)
requested_targets(Axx)
selected_targets(Axx)
excluded_targets(Axx + reason)
exec_targets(UAVn)
hold_reason
status_seq
每批目标、每次重试、service response、最终 held/rejected
```

## 7. 测试计划

### 7.1 protocol/TCP server 单元测试

- 标准 HOLD JSON 可解析。
- 缺失公共字段、空 assignments、重复 `uav_id` 被拒绝。
- HOLD assignment 只包含 Axx `uav_id` 时成功解析。
- `EmergencyHold.accepted=true` → TCP `COMPLETED`。
- `accepted=false` → TCP `FAILED/HOLD_REJECTED`，不得有 `completed_uavs` 假成功。
- ServiceException → TCP `FAILED/HOLD_SERVICE_FAILED`。
- 原活动命令先缓存为 `FAILED/COMMAND_HELD`。
- 同一 HOLD `command_id` 重复请求返回缓存终态。
- 若保持当前兼容策略，补充 reason 非字符串的现有行为测试；不要无关扩大 schema 变更。

### 7.2 backend 单元测试

- request UAV 子集只向子集下发 HOLD。
- 请求包含非 active UAV 时不向其下发，并返回明确拒绝/未确认信息。
- 未 started、已 terminal、失联成员不会被错误计为成功。
- 已 held 目标重复请求幂等，不重复调用 service。
- 批量上限、超时和重试保持现有行为。
- 两个并发安全终态只触发一次 group safety fan-out。
- 重复/乱序 `status_seq` 不重复触发。
- 触发者、已 terminal、失联成员和 A13–A15 reserve 被正确排除。
- 无 active group、mission mismatch、空可执行目标返回拒绝。

### 7.3 mock 多 Master/bridge 测试

- 模拟 `/{UAVn}/uav_hold` 成功、拒绝、超时、异常、迟到响应。
- 验证 Axx → UAVn/exec_target 转换和错误反向映射。
- 模拟一个参与 UAV 报告白名单安全终态，验证其余当前任务参与者收到 HOLD。
- 验证旧 Action 不继续推进，新普通任务不会绕过 active safety stop。
- 验证 bridge 只代理，不承担业务判定。

### 7.4 SITL 验收

1. 启动 15 个 SITL；普通任务实际只使用 A01–A12，A13–A15 保持 reserve。
2. 发起 MOVE_TO 或 FOLLOW_ROUTE，确认普通任务已获得合法 `ACCEPTED`。
3. 受控制造一个参与 UAV 的结构化安全终态，优先使用当前可稳定产生的 `MIN_DISTANCE_BREACH` 或等效白名单错误码。
4. 记录该 UAV 的 `UavTaskState`、`status_seq`、`error_code` 和 `exec_state`。
5. 在配置的 HOLD 确认/重试时间内，确认 backend 仅向同任务仍需收口的参与 UAV 发 `UavHold`。
6. 确认各目标机 `exec_state` 进入 `BRAKE_HOLD`，且其 `steady_target_`/固定位置 setpoint 来自该机自身捕获的当前位置和 yaw；旧轨迹不再继续。planner 安全监督/碰撞检查同样直接进入该状态，不应再出现 `EMERGENCY_BRAKE` 中间状态。
7. 单独发送协议第 7 节的指定 UAV HOLD，确认只影响指定 assignments；未指定 UAV 不被改变。
8. 重复同一 HOLD `command_id`，确认返回缓存结果且不重复产生副作用。
9. 注入 TCP 断线，确认本地取消旧任务并调用安全 HOLD；服务失败时产生本地可检索失败记录。
10. 保留完整日志：任务层 Axx、执行层 UAVn、exec_target、触发错误码、目标快照、重试和最终状态。
11. 完成定义前直接执行一次受保护的 15-UAV prepare：验证可解场景不过早进入 `BRAKE_HOLD`，安全事件时当前任务参与 UAV 均直接 `BRAKE_HOLD`，不含 reserve，不继续旧轨迹。每轮从 clean 开始，结束无条件清理；失败轮次不得单机修补或重启。

## 8. 影响、回滚和发布顺序

### 可能影响实际飞行表现

- 单机安全终态触发同任务其他 UAV HOLD。
- runtime 指定 HOLD 的目标过滤。
- `accepted=false`/超时情况下由假 `COMPLETED` 改为 `FAILED`。
- 安全错误码白名单及其触发时序。
- **测试门槛**：单元测试必须覆盖“预测可绕开不锁存”和“预测紧急且 replan 失败/不可避直接锁存”两条路径；具体用例、命令和结果由实施记录保存。完成构建和单元测试后，直接执行一次受保护的 15-UAV prepare；不要求额外的双机正交/相向回归。
- `stop_event` 与 Action 终止的竞态处理。

### 主要改善协议正确性和可观测性

- 消除 TCP HOLD 假成功。
- 记录实际选中/排除的任务层目标及其执行层映射。
- 记录批量、重试、重复状态和跨 master service 失败。
- 明确 TCP、Action、executor、driver 状态边界。

### 建议发布顺序

1. 先补 TCP `accepted=false` 的失败反馈测试和实现。
2. 再补 backend 指定目标集合过滤及成功判定测试。
3. 再补结构化安全终态的一次性 group fan-out。
4. 再实现 C++ 直接锁存入口与预测判据/去 `EMERGENCY_BRAKE`（§0 顺序：先置位→replan 绕开→失败才锁存），并跑 replan 余地与直锁单测。
5. 全量自动测试通过后执行 §7.4-11/12 的双机正交/相向与 15-UAV prepare 对照回归；未通过不得进入下一功能提交。
6. 最后才评估 executor token ownership 小修；若没有直接证据，延后。
7. 每一步独立提交，便于按功能回滚。

### 回滚边界

- 自动 group fan-out 误触发时，只关闭安全错误码白名单或对应功能开关，不回滚 runtime 指定 HOLD 的正确失败语义。
- 不删除现有 `_safe_hold()` 的批量重试。
- 不恢复 `EmergencyHold.accepted=false` 仍返回 TCP `COMPLETED` 的错误行为。
- 不修改 planner、安全距离、启动排程、intent 或 PX4 setpoint。

## 9. 成功标准

1. runtime 指定 HOLD 不经过 PREPARE/START；指定目标之外的 UAV 不被下发 HOLD。
2. `EmergencyHold` 拒绝、超时或异常时，TCP 绝不返回 `COMPLETED`。
3. 当前任务任一明确安全终态最多触发一次同任务 group safety fan-out。
4. fan-out 只作用于该 active task 的有效参与集合，不波及其他任务和 A13–A15 reserve。
5. 普通旧任务停止推进；安全命令目标 UAV 直接进入并持续保持 `BRAKE_HOLD`，其固定 setpoint 必须来自本机当前位置；失败则有明确未确认记录。锁存只在 SafetyPredictor 预测紧急（`min_d ≤ 1.0` 或 `ttc ≤ 1.5`）且 replan/让路绕开失败、或 ttc 已不可避时触发；实时 3D 门禁仅作兜底触发（§0/§4.4）。
6. 重复/乱序状态和重复 command_id 幂等。
7. 所有 Axx/UAVn/exec_target 映射、目标筛选、重试和结果均可由单测、mock 多 Master 测试和 SITL 日志复核。
8. 本阶段不新增 msg/srv，不改变 planner 算法、`t_off/schedule_tau`、clearance 或 PX4 setpoint 出口。
9. 双机正交/相向（预测可解 → 不锁存）与 15-UAV prepare 无头回归通过，且结果与 1/5 / 4/5 基线有对照记录。
