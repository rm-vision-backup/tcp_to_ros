# 测试记录 26090106：MOVE_TO/FAULT_EXIT 基于 t_off 的 GCS 机间避碰

> 对应 `plans/implementation_plan_26090106.md`。时间后缀与计划文件相同（2026-09-01 06:00）。

## 1. 实施内容

| 文件 | 改动 |
|---|---|
| `scripts/tcp_to_ros/time_offset_scheduler.py`（新增） | 纯计算排程器：水平直线段、两两安全 delta 区间（解析区间边界分析）、UAV1 参考方差优化（差分约束 + 活动集 QP）、负 tau 平移、搜索上限 |
| `scripts/gcs_a_backend_node.py` | pose 订阅/缓存、PREPARE 前排程、失败返回 FAILED+fail id、START 按 t_off 独立 worker 调度、预算派生与 timeout 前置校验 |
| `config/backend_defaults.yaml` | 新增 12 项排程参数（计划 §7） |
| `test/test_time_offset_scheduler.py`（新增） | 排程器单元测试 + GCS 集成测试 |
| `plans/README.md` | 登记本计划与测试记录索引 |

## 2. 参数来源（canonical，`backend_defaults.yaml`）

| 参数 | 值 | 来源 |
|---|---|---|
| `horizontal_speed_mps` | 0.7 | 计划 §4.2 固定速度 |
| `schedule_geometric_distance_m` | 1.0 | 计划 §4.3；与 3D 距离门禁 1.0m 一致 |
| `schedule_safety_margin_m` | 0.5 | 终点偏差 0.2-0.7m（test_record_26082400），取 0.5m 裕度 |
| `schedule_pose_timeout_s` | 1.0 | 计划 §5.2 |
| `schedule_time_epsilon_s` | 1e-6 | 数值容差 |
| `schedule_max_start_delay_s` | 30.0 | prepare 历史执行约 81s（ticks=1621 @20Hz），timeout 150s，30s 为 START 等待上限 |
| `schedule_max_interval_combinations` | 2000 | 资源上限 |
| `schedule_max_compute_time_s` | 5.0 | 资源上限 |
| `start_service_budget_s` | 1.5 | 覆盖 confirm_wait_s=1.0 + confirm_retry_interval_s=0.2 |
| `start_dispatch_jitter_margin_s` | 0.5 | 调度线程唤醒/本地排队/通信抖动，参考 hold_retry_interval_s 量级 |

## 3. 自动测试结果

命令（`catkin build tcp_to_ros` 后）：

```bash
source /opt/ros/noetic/setup.bash && source devel/setup.bash
python3 src/tcp_to_ros/test/test_time_offset_scheduler.py
```

结果：**20/20 通过**（13 项排程器单元 + 7 项 GCS 集成，耗时约 2.9s）。

### 3.1 单元测试覆盖（计划 §8.1）

- 同向平行 → tau 同步（variance=0）
- 相向 → 非零偏移，模型下安全（dmin >= d_ij 或无同时运动）
- 正交交叉 → 安全边界
- 不同航程同向 → 同步
- 负 tau → 整体平移非负，相对偏移与方差不变
- 不连续安全集合 → 区间保留、窄间隙不跳过
- 两两可行但全局无解 → 差分约束负环机制验证 + DFS 穷尽无解
- 安全集合为空 → PAIR_SAFE_SET_EMPTY
- 搜索上限命中（无解/有解非全局最优两种）
- 零位移几何 → GEOMETRY_UNSUITABLE
- 单机 → 平凡安全
- 参数校验（非法值 fail-fast）

每个成功排程验证：所有两两预测最小距离 >= d_ij；UAV1 参与方差；平移前后相对偏移与方差一致。

### 3.2 集成测试覆盖（计划 §8.2）

- pose 缺失/过期 → 不 PREPARE，FAILED + 对应 fail id（SCHEDULE_POSE_UNAVAILABLE）
- MOVE_TO 正常排程 → PREPARE 并发，START 按 tau 发送（实测 UAV1 晚于 UAV2 约 tau 差）
- 排程无解（50m 相向）→ 不发送任务，FAILED（SCHEDULE_PAIR_SAFE_SET_EMPTY）
- START 调度 → 每条记录 plan/actual/deviation 日志
- 自动预算派生 → max(tau') + 执行时间 + 1.5 + 0.5 <= timeout 才放行
- 任务 timeout 不足 → 前置拒绝（SCHEDULE_TIMEOUT_INSUFFICIENT）
- 非 MOVE_TO/FAULT_EXIT（FOLLOW_ROUTE）→ 现有流程不回归

## 4. 回归

`tcp_to_ros` 无 ROS/TCP 环境测试均 OK：test_calibration、test_fixed_formations、
test_gcs_a_aggregation、test_link_health、test_parameter_wiring、test_tcp_disconnect_safety、
test_tcp_port_policy。test_duplicate_command / test_hold_interrupt / test_wrong_group
需 39001 TCP server（端到端协议测试），本次无该环境未跑，与本次改动无关。

## 5. 未验收项（如实记录）

- **FAULT_EXIT 排程**：计划 §8 明确本轮不测试 FAULT_EXIT，专门验收延后。
- **§8.4 五次干净重启 prepare 仿真验收**：需 15 机 Gazebo/PX4 SITL 完整环境（`run.sh sim/onboard/gcs/tcp/takeoff/task` + `cleanup.sh`），本次会话未执行。
- 上述未验收项**不得视为已验证**；排程器/backend 的实机行为以仿真验收后的记录为准。
