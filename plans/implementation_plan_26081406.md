# Implementation Plan

[Overview]
删除飞行安全租约（safety lease）机制，保留 backend 已有的 `link_health` + `_safe_hold` 作为“节点丢失 → 任务停止 → 全悬停 → 手动接管”的唯一机制。

依据 `docs/test_record_26081402.md`（2026-08-14 复测）：prepare 多次失败，根因集中在 lease 层：
1. backend `begin_session`（lease START）失败 → `LEASE_START_FAILED`，任务从未下发给 driver；runtime 表现为 TIMEOUT。
2. executor 残留 ACTIVE lease（armed 下 END 被拒 `LEASE_END_UNSAFE`）→ 后续 START 全 `LEASE_EPOCH_CONFLICT`。
3. renew 失败（deadline 0.9s 过紧 + 高负载）→ TTL 过期 → executor 本地 watchdog 锁存 `LEASE_EXPIRED` latched → 所有任务被拒，latch 无法解除（只能重启 executor）。

实机（自组网电台 200-800ms 抖动）不可行点：
- renew deadline = now+0.9s << 电台 200-800ms 延迟 → 每轮 renew 必超时；
- 失败 → TTL 过期 → 锁存 HOLD → 需现场重启机载电脑才能恢复；
- 与 backend 已有 `link_health`（节点丢失检测）+ `_safe_hold`（丢失即 HOLD）功能重叠，是重复实现。

本计划删除 lease 层后，链路健康与安全停止由 `link_health` 承担：节点丢失 → MAVROS state 停 → `link_lost` → `_safe_hold` → 参与机 `uav_hold` → 全悬停。

[Types]

- 无新增 ROS 类型。
- `swarm_uav_interfaces/UavSafetyLease.srv` 文件**保留**（死定义，避免消息重新生成与无关引用；如需彻底清理后续单独删）。
- `UavLinkHealth.msg`、`UavTask.srv`、`UavHold.srv`、`UavTaskControl.srv`、`UavTaskState.msg` 均保留不变。

[Files]

### GCS 侧（`src/tcp_to_ros`）

- `scripts/gcs_a_backend_node.py`：
  - 删除 `UavSafetyLease` import、`from tcp_to_ros.safety_lease import SafetyLeaseCoordinator`、`LEASE_SERVICE_SUFFIX`；
  - 删除 `self.lease_coordinator = SafetyLeaseCoordinator(...)` 初始化；
  - 删除 renew 定时器（`next_renew` 循环与 `_inject_lease_failures`）；
  - 删除 `_inject_lease_failures`、`_call_safety_lease`；
  - `_execute` 删除 `begin_session` 与 `lease_failures` 检查（含 `LEASE_START_FAILED` 分支）；
  - `_execute` finally 删除“safety lease spans all batches”注释；
  - `shutdown` 删除 lease session 检查；
  - **保留** `link_health`（`UavLinkMonitor`）、`_safe_hold`、失败即停。
- `scripts/tcp_to_ros/safety_lease.py`：删除文件（GCS coordinator）。
- `launch/gcs_a_backend.launch`：删除 `safety_lease_ttl_s` / `safety_lease_call_timeout_s` 参数。
- `launch/gcs_a_backend_fail.launch`、`launch/gcs_a_backend_test.launch`、`launch/topology_group_a_sim.launch`：删除 `safety_lease_required` 传参。
- `config/link_safety_defaults.yaml`：删除 `safety_lease_*`，保留 link_health 段。
- `config/topology_group_a_sim.yaml`：删除 `/UAVn/uav_safety_lease` 服务代理条目（15 条）。
- `test/test_safety_lease.py`：删除。

### 机载侧（`src/swarm_uav_executor`）

- `scripts/uav_executor_node.py`：删除 lease imports、`MonotonicWatchdogThread` 类、`lease_config`、`SafetyLeaseWatchdog` 构建、`executor.lease_guard = watchdog`、lease 服务注册、watchdog 线程与 shutdown 处理。
- `src/swarm_uav_executor/safety_lease.py`：删除文件（watchdog）。
- `src/swarm_uav_executor/executor.py`：删除 `lease_guard` 参数、`_local_safety_latched`、handle_task 中 `LEASE_EXPIRED` / `LEASE_NOT_ACTIVE` 检查、`release_local_safety_latch`。
- `src/swarm_uav_executor/models.py`：删除 `LEASE_*` 常量、`SafetyLeaseConfig`、`SafetyLeaseRecord`、`LeaseExpiryEvent`。
- `launch/uav_executor_ego.launch`：删除 `safety_lease/required_for_tasks`。
- `launch/uav_executor_mavros.launch`：删除 `safety_lease/required_for_tasks`。
- `launch/uav_executor_mock.launch`：删除 `safety_lease_required` arg 与参数。
- `config/executor_defaults.yaml`：删除 `safety_lease:` 段。
- `test/test_safety_lease.py`：删除；`test_executor.py` / `test_ego_swarm_driver.py` / `test_mavros_position_driver.py` / `test_uav_executor_ros.py` / `uav_executor_rostest.test`：删除 lease 相关用例。

### 验证脚本（`verification/`）

- `ego_smoke/combined_sitl`、`ego_smoke/combined_sitl_check.py`、`ego_smoke/smoke_commands.py`：删除 lease START/RENEW/END 逻辑。
- `acceptance/gcs_a_backend_12exec.launch`、`acceptance/gcs_a_backend_15exec.launch`：删除 lease 参数。

### 文档

- `src/tcp_to_ros/README.md`、`plans/README.md`、`CONTEXT.md`：删除/替换 lease 描述为 link_health 语义。
- `docs/test_record_26081402.md` 安全边界段：更新 lease → link_health。

[Functions]

- GCS：删除 `_call_safety_lease`、`_inject_lease_failures`、`begin_session`/`renew`/`end_session` 调用；`_execute` 直接进入 `_run_goal`（confirm → START → 等待终态）。
- 机载：删除 `handle_lease`、`check_expiry`、`trigger_local_safety_hold` 等 lease 专用函数；`handle_task` 不再检查 lease 门禁。
- 保留：`link_health` 状态机、`_safe_hold`、`uav_hold` 服务、失败即停。

[Classes]

- 删除：`SafetyLeaseCoordinator`（GCS）、`SafetyLeaseWatchdog`（机载）、`MonotonicWatchdogThread`、`LeaseValidationError`、`LeaseExpiryEvent`、`SafetyLeaseRecord`、`SafetyLeaseConfig`。
- 保留：`UavLinkMonitor`、`GcsABackendNode`、`UavTaskExecutor`、`MotionDriver` 家族、`StateReporter`、`TaskStore`。

[Dependencies]

- 无新增第三方依赖。
- `tcp_to_ros` 继续依赖 `mavros_msgs`（link_health 订阅 `/mavros/state`）与 `swarm_uav_interfaces`。
- `swarm_uav_interfaces` 保留 `UavSafetyLease.srv` 文件但不再被引用。

[Testing]

- 单元：删除 lease 相关用例后，`catkin run_tests swarm_uav_executor tcp_to_ros` 全绿。
- 多 Master/SITL 验证（方案 A 后）：
  1. 15 机机载层 + backend（disarm 下标定 ready）；
  2. 起飞 15/15；
  3. prepare 应 ACCEPTED + COMPLETED（不再有 lease 门禁/时延障碍）；
  4. 依次跑 coverage-segment-1 / fault-exit / recovery / coverage-segment-2 / hold-end；
  5. 断链验证：停某个 bridge → `link_lost`（10-15s）→ `_safe_hold` → 全悬停，符合初衷。
- 实机（电台 200-800ms）适配：`link_suspect_after_s` / `link_lost_after_s` 放宽到 30/45s（阈值远大于链路延迟；MAVROS state 1-5Hz 经电台仍可达）。

[Implementation Order]

1. GCS：`gcs_a_backend_node.py` 删除 lease 集成，删 `safety_lease.py`，更新 launch/config/yaml/test。
2. 机载：`uav_executor_node.py` / `executor.py` / `models.py` 删除 lease，删 `safety_lease.py`，更新 launch/config/test。
3. 验证脚本清理（`verification/`）。
4. 文档对齐（README / plans / CONTEXT / 测试记录）。
5. `catkin build` + `catkin run_tests` 全绿。
6. 重启机载层 + backend（disarm 下标定）→ 起飞 → 重跑 6 批次。
7. 各包独立提交（Conventional Commit，不 push）。
