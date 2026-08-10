# catkin_swarm6-2 P1 ego-swarm 集成中断交接（2026-08-10 16:00）

## 当前结论

- 本轮 P1 开发已完成一批代码和自动化验证，但因运行时验证、文档与许可证问题尚未收口，**不可声明 P1 完成**。
- 当前唯一完成动作是审计工作空间内 7 个独立 Git 仓库的未提交和未 push 状态并保全中断现场；本交接生成期间未继续开发、构建、测试、提交或 push。
- 已有自动化证据为 `72 tests, 0 errors, 0 failures, 0 skipped`；单 ROS Master ego 运行时冒烟、双/三 Master reciprocal、PX4 SITL/MAVROS 和 Phase 6 均未完成。
- 安全边界不变：默认不 arm、不切 OFFBOARD；软件 HOLD 不替代 PX4 datalink/offboard/RC failsafe。
- P1 冻结范围、接口和实施顺序见 `../plans/implementation_plan_26081003.md`；前一轮通信安全闭环见 `handoff_swarm6-2_26080902.md`。本文件只记录中断增量和 Git 现场，不复制上述文档。

## 已完成事项

- 新增 `ego_planner_driver` C++ 包并接入 B-spline 轨迹优化、终点代价、邻机轨迹避碰、持续 setpoint/HOLD 和执行终态。
- 新增 `swarm_uav_interfaces/UavTrajectoryIntent`，并补齐 `geometry_msgs` 的生成与导出依赖。
- 新增 Python `EgoSwarmDriver`、executor 装配、参数和 launch；P1 leader route 可进入 ego driver，follower 编队跟随明确返回 `NOT_IMPLEMENTED`。
- Group A 仿真 topology 增加 `trajectory_intent`，UAV1/UAV2/UAV3 使用 reciprocal full mesh。
- 修复开发期间发现的空 route 死循环、B-spline 端点不命中、clearance 测试高度失真、单点 goal 不移动、HOLD 不持续、邻机时间轴错位等问题。
- 已完成 7 个仓库的只读 Git 审计；所有 tracked/staged diff 的 `git diff --check` 均无报错。

## 自动化证据（此前已成功，本次交接未重跑）

```text
catkin build swarm_uav_interfaces ego_planner_driver swarm_uav_executor tcp_to_ros
  成功

catkin run_tests swarm_uav_interfaces ego_planner_driver swarm_uav_executor tcp_to_ros
  成功

catkin_test_results --all build
  72 tests, 0 errors, 0 failures, 0 skipped
```

- 其中 ego C++ 5 个测试、ego Python driver 7 个测试、validation 6 个测试。
- 以上仅证明已执行的自动化范围，不替代 ROS Master、SITL、MAVROS/PX4 或实机门禁证据。

## 未完成事项与运行时边界

1. 单 ROS Master ego runtime 冒烟未完成；`/tmp/ego_smoke.py` 是未收口临时脚本。
2. 双/三 ROS Master 的轨迹意图直连、reciprocal 交叉/换位和三机错位/换位未验证。
3. PX4 SITL、MAVROS 断链、SafetyLeaseWatchdog 与 ego driver 的组合回归未验证。
4. END gate 尚缺 MAVROS state 新鲜、connected、稳定 disarmed 条件；Phase 6 实机不上桨门禁未执行。
5. README、CONTEXT、测试记录、工程过程和 attribution/LICENSE 尚未收口。
6. `/tmp/run_ego.sh` 曾被 literal DSML/XML 行污染并产生 shell syntax error；下一 task 不应复用，应重建并先检查残留 ROS node/process。

## Git 审计摘要

- 审计时间：其余 6 个仓库为 2026-08-10 17:05（Asia/Shanghai），`references/ego-planner-swarm` 为 17:14；写入本文件后已按“最终复核”节再次检查。
- 工作区根 `/home/ub20tg/catkin_swarm6-2` 不是 Git 仓库；下列 7 个目录各自独立。
- “ahead/behind”顺序统一写为相对 upstream 的 `behind/ahead`。
- 无 upstream 的仓库不能自动判定哪些本地 commit 已 push；不能把“无 remote”误写成已 push 或未 push。

| 仓库 | 分支 / HEAD | upstream 与未 push | 工作树 |
|---|---|---|---|
| `src/ego_planner_driver` | `main` / `3f6c2ba607eeb31f92fb350c1b09c4cc813cdec8` | 无 remote、无 upstream；无法自动判断未 push。仓库共 2 个本地 commit | 5 modified，0 staged，0 untracked |
| `src/safe_valley_exp` | `modular` / `62a00f49252b568050c2ca0a8b5c44bcc415f5fe` | `origin/modular`，`0/0`，无未 push commit | clean |
| `src/swarm_topology_bridge` | `services` / `1e0d8d352e68ef5c5afad6a2732fabc2579385f9` | `origin/services`，`0/0`，无未 push commit | clean；另一本地非当前 `master` 显示落后 `origin/master` 1 commit |
| `src/swarm_uav_executor` | `main` / `a486a8dd6fdad0f7f63f2e0d9cc3d1530d6b5bba` | `origin/main`，`0/0`，无未 push commit | 6 modified，0 staged，3 untracked |
| `src/swarm_uav_interfaces` | `main` / `a1a4b14d5af39f2d82287bad40b5831bf9976569` | `origin/main`，`0/0`，无未 push commit | 2 modified，0 staged，1 untracked |
| `src/tcp_to_ros` | `main` / `261745ba4e96216756de3580bf8def67a1cdfc15` | `origin/main`，`0/1`；1 个 committed-but-unpushed commit | topology 1 modified；本交接文件新增后另有 1 untracked |
| `references/ego-planner-swarm` | `master` / `92fe9f7227b2da819133eb8e0e8c7fc000f6ae20` | `origin/master`，`0/0`，无未 push commit | tracked tree clean，0 staged，`docs/` 下 13 个 untracked |

## 各仓库精确现场与重要变更

### 1. `src/ego_planner_driver`

- 本地 commits（仓库无 remote/upstream）：
  - `3f6c2ba fix: qualify UniformBspline and add eigen include`
  - `9f2227b feat: add ego swarm trajectory driver core`
- unstaged modified：
  - `include/ego_planner_driver/ego_planner_core.h`
  - `launch/ego_planner_driver.launch`
  - `src/ego_planner_core.cpp`
  - `src/ego_planner_driver_node.cpp`
  - `test/test_trajectory.cpp`
- tracked stat：`5 files changed, 283 insertions(+), 138 deletions(-)`；无 staged/untracked。
- 核心增量：`lambda_terminal` 与终点 cost；空 waypoints 直接 HOLD；不足 5 点时细分；四阶端点导数；采样首尾锚定；新增 `COMPLETED/HOLD/POSE_STALE/EGO_PLAN_FAILED/EGO_EXEC_TIMEOUT` 等状态；到位稳定与执行超时；邻机绝对 stamp 转本机相对时轴；`traj_id` 单调去重。

### 2. `src/safe_valley_exp`

- 当前分支 `modular` 与 `origin/modular` 同步，工作树 clean，无 staged、untracked 或未 push commit。
- 本轮 P1 没有现场变更。

### 3. `src/swarm_topology_bridge`

- 当前分支 `services` 与 `origin/services` 同步，工作树 clean，无 staged、untracked 或未 push commit。
- 非当前本地 `master` 分支落后 `origin/master` 1 commit；这不影响当前 `services` 的 `0/0`，但下一 task 不应笼统声明所有本地分支均同步。

### 4. `src/swarm_uav_executor`

- unstaged modified：
  - `config/executor_defaults.yaml`
  - `scripts/uav_executor_node.py`
  - `src/swarm_uav_executor/executor.py`
  - `src/swarm_uav_executor/models.py`
  - `src/swarm_uav_executor/validation.py`
  - `test/test_validation.py`
- untracked：
  - `launch/uav_executor_ego.launch`
  - `src/swarm_uav_executor/drivers/ego_swarm.py`
  - `test/test_ego_swarm_driver.py`
- tracked stat：`6 files changed, 75 insertions(+), 6 deletions(-)`；无 staged。
- 核心增量：`MotionGoal` 增加 waypoints/leader/formation/command；`build_goal(request)`；FOLLOW_ROUTE leader route 校验与 follower `NOT_IMPLEMENTED`；`build_driver` 增加 `ego_swarm`；Safety Lease 默认覆盖 `mavros_position` 与 `ego_swarm`；新增 ego 参数、launch、topic relay 和 7 个 driver tests。

### 5. `src/swarm_uav_interfaces`

- unstaged modified：`CMakeLists.txt`、`package.xml`。
- untracked：`msg/UavTrajectoryIntent.msg`；无 staged。
- tracked stat：`2 files changed, 9 insertions(+), 2 deletions(-)`。
- 消息字段为 `protocol_version/uav_id/exec_target/traj_id/stamp/t/sampled_traj/clearance`；`sampled_traj` 使用 `geometry_msgs/Pose[]`。

### 6. `src/tcp_to_ros`

- committed-but-unpushed：`261745b docs(plan): add ego-swarm P1 trajectory integration plan`。
  - commit 内容：`CONTEXT.md` +33；`plans/implementation_plan_26081003.md` +159；合计 192 insertions。
- unstaged modified：`config/topology_group_a_sim.yaml`，tracked stat 为 `1 file changed, 4 insertions(+), 3 deletions(-)`。
- topology 增加 `swarm_uav_interfaces/UavTrajectoryIntent`，并把 UAV1/UAV2/UAV3 改为 reciprocal full mesh；`GCS_A` 仍连接三机但不进入机载避碰控制环。
- 本文件 `handoffs/handoff_swarm6-2_26081016.md` 创建后为新增 untracked 文件；无 staged。

### 7. `references/ego-planner-swarm`

- 当前 `master` 与 `origin/master` 同步；tracked tree clean，无 staged、unstaged 或未 push commit。
- untracked 共 13 个文件，总计 645,887 bytes：
  - 论文转录：`docs/3.Zhou-等-2021-EGO-Swarm-A-Fully-Autonomous-and-Decentralized-Qu.md`（63,091 bytes）。
  - 附图 12 张（合计 582,796 bytes）：
    - `docs/images/0_912_489_737_336_0.jpg`
    - `docs/images/0_913_874_737_248_0.jpg`
    - `docs/images/1_155_154_719_344_0.jpg`
    - `docs/images/2_179_155_705_664_0.jpg`
    - `docs/images/2_186_1058_663_232_0.jpg`
    - `docs/images/2_922_161_726_235_0.jpg`
    - `docs/images/3_917_148_732_282_0.jpg`
    - `docs/images/3_923_598_723_398_0.jpg`
    - `docs/images/4_152_149_731_550_0.jpg`
    - `docs/images/5_147_148_1499_228_0.jpg`
    - `docs/images/5_153_731_727_193_0.jpg`
    - `docs/images/5_916_452_733_862_0.jpg`
- 这些新增项是参考论文和附图，不是 planner 源码变更；仍应由下一 task 决定是否纳入该 GPLv3 仓库版本历史。

## 优先风险与待复核设计

### 红色：许可证与来源追踪

- `references/ego-planner-swarm` 的 `LICENSE` 和 README 均声明 GPLv3；`ego_planner_driver` 抽取/改编了其核心代码，但新包当前声明 MIT，存在明显许可证不一致风险。
- 下一 task 在继续分发、提交或 push `ego_planner_driver` 前，应优先做来源文件逐项审计，修正 package 许可证并补 attribution、版权通知和 GPLv3 对应材料；不能把自动化通过视为许可证合规。

### 运行与安全风险

- 任意 neighbor intent 可能触发 C++ replan；pending goal/waypoints 生命周期需要复核，避免过期邻机消息扰动当前任务。
- Python `EgoSwarmDriver.start_move_to` 当前以 `goal.leader_id` 是否非空决定发布 waypoints，FOLLOW_ROUTE leader/follower 语义可能反向或过度耦合，需结合 validation 和调用方复核。
- `can_end_safety_lease` 目前仅检查 fresh pose 与非 `EXECUTING`，缺 MAVROS connected 与稳定 disarmed 门禁。
- `FAULT_EXIT/HOVER` 沿用 MOVE_TO target validation，尚未强制 8 m/15 m 等 layer height 约束。
- `MotionGoal.waypoints: Tuple` 类型过泛，应收窄元素类型或引入明确 waypoint model。
- C++ HOLD 在无 local pose 时退回轨迹终点，可能导致不安全 setpoint；应改为 fail-safe 策略并补测试。
- plan start snapshot 在 optimizer 前获取，优化耗时会侵占执行时间轴；应决定以规划完成时刻还是输入快照时刻作为轨迹 epoch。

## 下一 task 建议顺序

1. 先复核本文件列出的 7 仓库状态，避免覆盖未提交工作；不要清理、reset 或 checkout 当前现场。
2. 优先完成 GPLv3 来源/许可证审计并冻结处理方案，再继续代码提交。
3. 静态复核 FOLLOW_ROUTE route publish 条件、neighbor-triggered replan、HOLD fallback 和 plan epoch；补针对性测试。
4. 重建单 Master 冒烟脚本，不复用 `/tmp/run_ego.sh`；先检查并清理残留 ROS node/process，再验证 goal→setpoint→terminal/HOLD 生命周期。
5. 按冻结计划逐级执行双 Master、三 Master、PX4 SITL/MAVROS 与 Safety Lease 回归；每级失败即停止升级。
6. 补齐 README、CONTEXT、测试记录、工程过程、attribution/LICENSE；只有证据和文档收口后才评估 P1 完成声明。
7. 如需提交，按独立仓库分别检查 diff 与 Conventional Commit；遵循用户要求，不擅自 push 或配置 remote。

## 常用只读验证命令

```bash
cd /home/ub20tg/catkin_swarm6-2

for repo in \
  src/ego_planner_driver \
  src/safe_valley_exp \
  src/swarm_topology_bridge \
  src/swarm_uav_executor \
  src/swarm_uav_interfaces \
  src/tcp_to_ros \
  references/ego-planner-swarm
do
  echo "===== $repo ====="
  git -C "$repo" status --short --branch
  git -C "$repo" rev-parse HEAD
  git -C "$repo" branch -vv
  git -C "$repo" diff --check
  git -C "$repo" diff --cached --check
done

# 对配置了 upstream 的仓库检查未 push commit
git -C src/tcp_to_ros rev-list --left-right --count '@{upstream}...HEAD'
git -C src/tcp_to_ros log --oneline '@{upstream}..HEAD'

# 无 remote/upstream 的 ego_planner_driver 只能列本地历史，不能自动断言 push 状态
git -C src/ego_planner_driver remote -v
git -C src/ego_planner_driver log --oneline --all
```

## Suggested skills

- `diagnose`：按单 Master→双/三 Master→SITL/MAVROS 的顺序定位 runtime、时轴、重规划和安全门禁问题。
- `grill-with-docs`：在继续实现或提交前核对 P1/P2 边界、FOLLOW_ROUTE 语义、HOLD/END 安全职责和领域术语。
- `improve-codebase-architecture`：在行为冻结后审查 C++ core/topic wrapper、Python driver/executor 的边界和可测试性。
- `handoff`：下一轮结束时更新 Git 现场、可声明能力、自动化与运行时证据。
