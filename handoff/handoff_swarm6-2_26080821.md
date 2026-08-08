# catkin_swarm6-2 Phase 4/5 收口交接（2026-08-08 21:00）

## 1. 当前结论

- 可以声明：**GCS_A 核心链路达到机载接入基线；Phase 4 `MOVE_TO + HOLD` 首版、direct MAVROS PX4 SITL 三机主动飞行验证已完成。**
- 不可声明：GCS_A 与机载链路已经完成最终验收、完整弱网验收已通过、真实硬件不上桨或实机上桨已通过。
- 冻结实施计划 `docs/implementation_plan_26080804.md` 的 12 个步骤，经文档收口后的最终审计为：**8 项完成、2 项部分完成、2 项未完成**。
- 下一主线不是继续扩展 GCS_A command 解析，而是先补齐 **Phase 5 跨 Master 弱网证据** 与 **Phase 6 实机不上桨安全门禁**；随后再正式设计 EGO Planner、航线、编队与机间避碰。

## 2. 已落地架构

### 包与仓库边界

- `src/tcp_to_ros`：GCS_A 地面站业务包，本地整组 Action、TCP 接入、逐机分发与 `ALL` 聚合。
- `src/swarm_uav_interfaces`：GCS 与机载共同依赖的纯 ROS msg/srv 包。
- `src/swarm_uav_executor`：每架 UAV 本机部署的执行器，拥有 validation、幂等 TaskStore、StateReporter、执行状态机和可替换 MotionDriver。
- `src/swarm_topology_bridge`：动态 Topic/Service 跨 Master 代理，不依赖业务包、不解释 command、不桥完整 Action。
- 工作空间根 `/home/ub20tg/catkin_swarm6-2` 不是 Git 仓库；两个新包没有登记为 submodule。

### 冻结语义

- 唯一任务键：`(mission_id, command_id, uav_id)`。
- 同键同内容返回已有记录，不重复动作；同键不同内容拒绝为冲突。
- `accepted=true` 只表示校验、登记并接管动作入口，终态由 `UavTaskState` Topic 发布。
- 每个 executor 同时最多一个正常任务；HOLD 高优先级抢占。
- 每个实例只接受启动时冻结的 `(uav_id, exec_target)`，且只连接本机 MAVROS namespace。
- 其他 UAV 信息未来只可作为感知/避碰输入，不能形成跨机控制权。
- GCS_A 并发确认仍使用单次 1.0 s、首次 + 2 次重试、间隔 0.2 s、整组绝对 deadline 4.0 s；`ALL` 任一失败即停。
- direct MAVROS driver 默认 `auto_arm=false`、`auto_offboard=false`。

### 共享接口

`swarm_uav_interfaces` 拥有：

- `Pose3DYaw.msg`
- `TaskAssignment.msg`
- `UavTaskState.msg`
- `UavTask.srv`
- `UavHold.srv`

`tcp_to_ros` 只保留本地业务接口：

- `ExecuteGroupTask.action`
- `GroupTaskState.msg`
- `EmergencyHold.srv`

## 3. 已验证能力

### 自动化与 ROS 回归

- `catkin build swarm_uav_interfaces swarm_uav_executor tcp_to_ros`：联合构建成功。
- `catkin run_tests swarm_uav_executor`：22 tests，0 error，0 failure。
- `catkin run_tests tcp_to_ros`：7 tests，0 error，0 failure。
- executor 单 Master rostest：Service 接收与终态 Topic 通过。
- GCS_A 正常整组、重复幂等、单机失败即停及活动 UAV HOLD 回归通过。
- 状态参与机/exec_target/status_seq 过滤、共享绝对 deadline、迟响应隔离、Action cancel 与 TCP 断线 HOLD 已有进程内自动化证据。

### PX4 SITL 三机主动飞行（2026-08-08）

- GCS_A 使用 ROS Master `11311`，UAV1..3 使用 `11312..11314`。
- `iris_0..2` 分别绑定 `A01/UAV1`、`A02/UAV2`、`A03/UAV3`。
- 三机 setpoint 约 30 Hz；操作员手动解锁、起飞并切换 OFFBOARD。
- 并发 `MOVE_TO` 均到达 `COMPLETED`：
  - A01 目标 `(1,0,10,0)`，实测约 `(1.02,-0.03,10.01)`；
  - A02 目标 `(0,1,10,0)`，实测约 `(0.03,0.98,10.03)`；
  - A03 目标 `(-1,0,10,0)`，实测约 `(-0.96,-0.02,10.03)`。
- HOLD 均返回 accepted；5 秒位移分别为 `0.099/0.105/0.121 m`，小于本轮 SITL 阈值 `0.5 m`。
- 权威成功终态是 `COMPLETED`，不是 `SUCCEEDED`。临时验收脚本曾因错误终态字符串和 YAML 引号处理误报失败，产品代码无对应故障。

完整证据见 `docs/tcp_to_ros_测试记录.md` 文末。

## 4. 冻结计划完成度

| Step | 状态 | 说明 |
|---|---|---|
| 1 基线 | 完成 | Phase 1-3 构建、单 Master 回归、跨 Master Service 冒烟已有记录。 |
| 2 接口仓库 | 完成 | 独立接口仓库、3 msg + 2 srv 与构建已完成。 |
| 3 类型迁移 | 完成 | 消费方切换完成，旧共享定义删除。 |
| 4 executor 骨架 | 完成 | 核心模型、校验、store、reporter、driver 边界已落地。 |
| 5 mock executor | 完成 | 幂等、冲突、busy、timeout、failure、HOLD 抢占已验证。 |
| 6 替换旧 mock | 完成 | launch 使用新 executor，旧 `uav_mock_node.py` 删除。 |
| 7 Phase 5 跨 Master | 部分完成 | 逻辑边界已测；缺可复现 bridge 弱网/重连/丢包端到端归档。 |
| 8 direct MAVROS | 完成 | fake callback 单测 + 三机 PX4 SITL 主动飞行通过。 |
| 9 不上桨安全联调 | 未完成 | SITL 不能替代真实硬件不上桨与 PX4 failsafe 现场验证。 |
| 10 文档与证据 | 完成 | CONTEXT、Phase 状态、过程、测试证据、未决问题与本交接已收口；弱网/不上桨本身仍分别属于 Step 7/9。 |
| 11 仓库与提交 | 完成 | 三个业务仓库已有独立提交；新仓库无 remote、未 push。 |
| 12 后续能力设计 | 未完成 | 只有扩展端口，尚无 EGO/航线/编队/避碰正式设计和测试产物。 |

## 5. Git 精确状态（最终文档提交前快照）

### `tcp_to_ros`

- 分支/HEAD：`main` / `578ac0fc1dbf761104da549a65cfa99ce5e153fd`
- upstream：`origin/main`
- 相对 upstream：ahead 4
- 工作树有文档变更，尚未提交：
  - `CONTEXT.md`
  - `docs/tcp_to_ros_后续开发计划.md`
  - `docs/tcp_to_ros_工程过程.md`
  - `docs/tcp_to_ros_提问记录.md`
  - `docs/tcp_to_ros_测试记录.md`
  - 本交接文件（新增）
- `docs/implementation_plan_26080804.md` 未修改，继续作为冻结基准。
- 这些工作树文档包含此前 SITL/计划审计更新与本轮 agent 的领域/过程/交接收口；提交信息应覆盖用户与 agent 的合并变更。

### `swarm_uav_interfaces`

- 分支/HEAD：`main` / `4213d8fced8d51acb777f0e1acc1f72c00cc3e43`
- 工作树 clean；无 upstream、无 remote。

### `swarm_uav_executor`

- 分支/HEAD：`main` / `aa4d0524d9e7157df2642bc3005ab44ffb04d0f8`
- 工作树 clean；无 upstream、无 remote。

### `swarm_topology_bridge`

- 分支/HEAD：`services` / `1e0d8d352e68ef5c5afad6a2732fabc2579385f9`
- upstream：`origin/services`
- 工作树 clean。

截至该快照尚未执行最终文档提交；全程未配置新 remote、未 push。最终提交状态应以 `git status` 与 `git log -1` 为准。

## 6. 下一步优先级

### P0：提交前复核当前文档工作树（已执行）

1. 阅读 `git -C src/tcp_to_ros diff --check` 和完整文档 diff。
2. 确认 `implementation_plan_26080804.md` 保持未修改。
3. 用户已授权提交；使用简洁 Conventional Commit，header 仅英文，body 先英文后 `Chinese:` 中文对应正文。

### P1：Phase 5 完整跨 Master 弱网验收

1. 冻结可复现故障模型：延迟、抖动、丢包、短时断网、重连、Service 迟响应。
2. 使用 GCS_A + 独立 UAV Masters + bridge + mock executor，避免在弱网测试中混入真实飞行风险。
3. 必须归档：
   - 状态 Topic 经 bridge 到 GCS_A 的完整聚合；
   - 4 秒整组确认边界；
   - Service timeout 后迟响应不得复活任务；
   - Action cancel 与 TCP 断线触发 HOLD；
   - HOLD 未确认 UAV 与错误码保留；
   - 原始日志、测试命令、环境参数，必要时 rosbag。
4. 将结果追加到 `docs/tcp_to_ros_测试记录.md`，将过程追加到 `docs/tcp_to_ros_工程过程.md`。

### P2：Phase 6 实机不上桨安全联调

在任何上桨前现场冻结并签字确认：

- A01-A15 到 hostname/IP/exec_target 的映射；
- 每机 MAVROS namespace；
- local frame、ENU/NED 转换与 yaw 约定；
- position/yaw tolerance、stable window、pose stale timeout；
- PX4 datalink/offboard/RC/battery failsafe；
- arm、mode、急停和现场安全职责；
- driver unhealthy、断联、executor 退出时的 HOLD/failsafe 行为。

仍必须保持默认不自动 arm、不自动切 OFFBOARD。SITL 结果不能作为上桨授权。

### P3：后续动作能力正式设计

- `EgoPlannerDriver`：局部规划/轨迹执行适配。
- `FollowRouteCoordinator`：航点与航线任务协调。
- leader-followers 编队：明确 leader 参考、follower 目标生成和成员失效策略。
- `InterUavAvoidanceProvider`：其他 UAV 信息仅作为感知约束，不授予跨机控制权。
- 为上述模块先产出 ADR/接口时序/故障语义/仿真验收方案，再编码。

## 7. 常用命令

```bash
cd /home/ub20tg/catkin_swarm6-2
source ~/.bashrc

# 联合构建
catkin build swarm_uav_interfaces swarm_uav_executor tcp_to_ros
source /home/ub20tg/catkin_swarm6-2/devel/setup.bash

# 自动化测试
catkin run_tests swarm_uav_executor
catkin run_tests tcp_to_ros
catkin_test_results build/swarm_uav_executor build/tcp_to_ros

# 仓库状态
git -C src/tcp_to_ros status --short --branch
git -C src/swarm_uav_interfaces status --short --branch
git -C src/swarm_uav_executor status --short --branch
git -C src/swarm_topology_bridge status --short --branch

# 文档提交前校验
git -C src/tcp_to_ros diff --check
git -C src/tcp_to_ros diff -- docs CONTEXT.md handoff
```

若 Agent 环境不能写默认 `~/.ros`，继续使用：

```bash
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
mkdir -p "$ROS_LOG_DIR"
```

## 8. 权威文档顺序

1. Phase 编号、接口、聚合与超时约束：`docs/tcp_to_ros_后续开发计划.md`
2. 本次冻结实施基准：`docs/implementation_plan_26080804.md`
3. 已落地过程：`docs/tcp_to_ros_工程过程.md`
4. 自动化、跨 Master 与 SITL 证据：`docs/tcp_to_ros_测试记录.md`
5. 领域词汇和包边界：`CONTEXT.md`
6. 未决问题：`docs/tcp_to_ros_提问记录.md`
7. 当前会话与 Git 状态：本交接文件
