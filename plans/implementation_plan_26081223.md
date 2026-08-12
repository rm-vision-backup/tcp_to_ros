# Implementation Plan

状态：已实施并通过自动化质量门槛（2026-08-13）。15 机 PX4 SITL 全批次和实机不上桨/上桨验证仍需现场执行。

## 目标

将已确认的集群坐标标定、固定编队、按需起飞和分层运动语义落实到现有 ROS1/MAVROS 系统，保持
TCP 上游协议兼容且不修改 `references/`。最终要求工作空间内 `catkin build` 通过，并以单元测试、
节点级测试和可执行的仿真验收配置证明关键安全门禁。

## 已冻结决策

1. 任务坐标统一为右-前-上：`x` 向右、`y` 向前、`z` 向上；不使用 runtime 下发的 yaw。
2. GCS_A 启动后，使用 A01-A12 全部 12 架 arm 前全球位置逐轴中位数冻结全局原点；任何一架
   不合格都不允许进入可 arm 状态，不做子集匹配。
3. A01-A12 arm 前航向使用圆周平均冻结为集群参考航向，任一飞机与参考航向偏差不得超过 30°。
4. 冻结原点通过 MAVROS ROS 话题下发给 A01-A15，并通过 `gp_origin` 一次性回读确认；水平误差
   不超过 0.1 m、高度误差绝对值不超过 0.1 m。业务代码不直接处理 MAVLink 编码和高度基准转换。
5. 15 架全部确认原点后 GCS_A 才可接收任务；冻结值在 GCS_A 进程生命周期内不再变化和复验。
6. 全球位置中位点映射到任务坐标 `mission_enu=(10,5,0)`；任务航点均视为任务坐标系中的偏置航点。
7. A01/UAV1 固定为 leader。`coverage-segment-1` 只接受固定编队 1 的精确参与机集合，
   `coverage-segment-2` 只接受固定编队 2 的精确参与机集合；集合不匹配直接拒绝。
8. `MOVE_TO` 可按需启动此前未 arm 的参与机：整批 PREPARE 门禁通过后统一 START，完成 arm、
   OFFBOARD、原地上升 5 m、实测稳定保持 2 s；已 arm 的参与机跳过起飞阶段。
9. TCP 协议保持不变；PREPARE/START/ABORT 为 GCS_A 与机载 executor 内部两阶段语义。
   runtime 的 `timeout_s` 是整批共享期限，包含准备、起飞、检查点和后续动作。
10. 严格垂直阶段采用简单受约束 z 轨迹并冻结阶段起点 x/y/yaw；其他非 follower 航迹使用 EGO。
    垂直执行期间实时机间距离突破门槛，当前整批立即 HOLD。
11. `MOVE_TO`：5 m 检查点后先垂直到 15 m，再 EGO 水平移动，最后垂直到目标 z。
12. `FOLLOW_ROUTE`：leader 在 12 m 编队层执行 EGO 路线；followers 使用 PI 位置控制叠加编队偏置。
13. `FAULT_EXIT`：先垂直到 8 m 离队层，再 EGO 水平移动到目标位置。
14. 无人机物理半径 0.4 m；水平中心最小距离 1.0 m、垂直中心最小距离 2.0 m。EGO 路径和
    follower PI 控制均须具有运行时硬安全门禁。
15. 仿真出生位置改为 3 行 5 列：A01/A04/A07/A10/A13 位于 y=10，A02/A05/A08/A11/A14
    位于 y=5，A03/A06/A09/A12/A15 位于 y=0；各列 x=0/5/10/15/20。

## 固定编队

### 编队 1：`coverage-segment-1`

```text
y=0    A01  A04  A07  A10
y=-5   A02  A05  A08  A11
y=-10  A03  A06  A09  A12
        x=0  x=5  x=10 x=15
```

### 编队 2：`coverage-segment-2`

```text
y=0    A01  A04  A13
y=-5   A02  A05  A14
y=-10  A03  A06  A15
        x=0  x=5  x=10
```

## 实施阶段

### 阶段 1：代码基线与接口审计

- 审计六个现有 `src` 子仓库的未提交修改，保留用户已有工作并识别跨包依赖。
- 核对 runtime 协议字段、GCS_A action/service/topic、executor、EGO driver、仿真生成脚本和现有测试。
- 核对 `safe_valley_exp::sync_origin()` 与 `offboard_takeoff_15.py`，只提取适用机制，不照搬旧语义。
- 确认 `Testing/` 与 `verification/` 用途；保留有价值的验收资料，只将无价值缓存作为可清理项。

### 阶段 2：共享接口和配置

- 在合适的 ROS 接口包中增加机载准备/启动/中止、标定状态和安全状态所需的最小消息或服务字段；
  若现有接口足够则优先兼容扩展，避免破坏 TCP 上游协议。
- 将两个固定编队槽位、精确参与机集合、leader、任务锚点、层高、安全距离、标定门槛、重试与
  稳定时间写入 `tcp_to_ros/config/*.yaml`，允许 YAML 修改但默认值固定为本计划。
- 修正 EGO `swarm_clearance` 的半径语义，使 1.0 m 水平中心距对应每机 0.5 m clearance，并保留
  0.4 m 物理半径作为独立参数和校验下界。

### 阶段 3：GCS_A 原点与航向标定

- 新增可单元测试的纯 Python 标定模块：采样窗口、逐机数据新鲜度、全球位置逐轴中位数、圆周平均、
  30° 航向一致性、冻结状态和任务坐标变换。
- GCS_A 从 A01-A12 的 MAVROS 全球位置与 arm 前姿态读取样本，达到全部 12 架质量门槛后冻结。
- 先建立 A01-A15 的 `gp_origin` 回读订阅，再发布每机 `set_gp_origin`；只接受本轮发布后的回读，
  按 0.1 m 水平/0.1 m 高度容差确认，超时或超差重试。
- 全部 15 架确认后发布/维护 GCS 就绪状态；不等待 RTK 收敛、不在任务期间重新冻结或复验。
- 将任务坐标通过冻结航向和平移锚点转换为统一 local ENU，覆盖右-前-上轴定义。

### 阶段 4：GCS_A 分发、固定编队与整批期限

- 按 `coverage-segment-1/2` 的精确参与机集合选择固定编队，不读取 runtime 的编队形状或 yaw。
- runtime 集合与固定集合不一致时在接受阶段直接拒绝，不执行任务。
- 将 runtime `timeout_s` 转换为整批绝对 deadline，并随内部命令传递；移除会比整批 deadline 更早
  误杀任务的固定执行超时。
- 实现 PREPARE/START/ABORT 内部两阶段屏障：PREPARE 读取持续维护的机载健康快照，所有参与机
  注册成功后才 START；任一失败则 ABORT/HOLD。TCP 接受响应仍满足既有 5 s 约束。

### 阶段 5：机载 executor 的唯一 arm 权限与按需起飞

- executor 成为唯一允许调用 MAVROS arm/OFFBOARD 的组件；旧 takeoff 脚本降级为诊断工具。
- 为未 arm 的 `MOVE_TO` 参与机实现 arm 前门禁、START 后统一 arm/OFFBOARD、5 m 简单垂直轨迹、
  实测到达和连续稳定 2 s；已 arm 参与机跳过该阶段。
- A13-A15 可上电并运行 offboard/executor，但在首次参与 `MOVE_TO` 前保持 disarm。
- ABORT、期限耗尽或任一参与机失败时停止本批阶段推进并对活动参与机执行 HOLD。

### 阶段 6：分层轨迹与安全门禁

- 抽取/实现通用受约束垂直轨迹控制器：阶段开始冻结 x/y/yaw，平滑加速-巡航-减速，按实测位置
  判定到达，不让 EGO 负责严格垂直段。
- 重构 `MOVE_TO` 为检查点、15 m 垂直段、15 m 水平 EGO 段、目标 z 垂直段。
- 重构 `FOLLOW_ROUTE`：leader 垂直到 12 m 后执行 EGO；followers 在 12 m 使用 PI + 固定偏置。
- 重构 `FAULT_EXIT`：垂直到 8 m 后执行水平 EGO，再按既有任务终态要求收尾。
- 所有垂直段增加当前整批实时距离监控；突破水平/垂直门槛立即整批 HOLD。
- 为 EGO 规划结果增加硬 clearance 校验；为 follower PI 输出增加实时最小距离 HOLD 门禁。

### 阶段 7：仿真、launch 与兼容性

- 修改 `safe_valley_exp`/相关仿真 launch 或生成脚本，使 SITL 使用计划中的 15 机 Gazebo 世界坐标，
  但任务原点不依赖 Gazebo 原点。
- 将 GCS_A 标定节点、机载状态上报、executor 两阶段服务和安全监控装配进仿真与实机 launch；
  通过参数区分仿真/实机差异，不维护两套动作语义。
- 不修改 `references/`；对 runtime 的兼容性仅通过现有协议输入和本地适配实现。

### 阶段 8：测试和交付

- 为标定数学、坐标变换、固定编队集合、deadline、两阶段屏障、按需起飞、垂直轨迹和安全距离
  增加单元测试。
- 扩展 mock MAVROS/GCS 验收，覆盖 15/15 原点确认、某一架缺失/超差、航向超过 30°、集合不匹配、
  未 arm MOVE_TO、已 arm MOVE_TO、起飞检查点失败、垂直距离触发整批 HOLD。
- 运行各受影响包的定向构建和测试，再在工作空间根目录执行完整 `catkin build`。
- 运行 `catkin_test_results`，记录无法在当前环境执行的 SITL/实机项目及原因。
- 更新 `CONTEXT.md`、`plans/README.md`、README/launch 注释和测试记录，使旧的周期发布原点、
  手工 arm 安全基线等描述与新语义一致。

## 预期影响文件

实施前以代码审计结果为准，预计涉及：

- `tcp_to_ros`：GCS_A backend、标定/坐标模块、配置、launch、mock 与测试、文档。
- `swarm_uav_interfaces`：仅在现有 action/service 无法表达内部两阶段与 deadline 时最小扩展。
- `swarm_uav_executor`：准备屏障、唯一 arm 权限、按需起飞、动作阶段编排、HOLD 与测试。
- `ego_planner_driver`：受约束垂直轨迹接口、EGO 硬 clearance、执行 deadline 与参数。
- `safe_valley_exp`：仿真出生编队、兼容仿真/实机的 launch，以及旧 takeoff 脚本诊断化。
- `swarm_topology_bridge`：仅在新增 ROS 接口需要跨 Master 转发时修改配置或通用代理。

## Git 与工作区约束

- 不修改 `references/` 和工作空间外文件。
- 各现有 `src/<package>/.git` 独立提交；不在 `src/` 顶层重复创建仓库。
- 提交前逐包检查已有脏文件，避免覆盖或夹带无关修改。
- commit message 遵循 `.codex/rules/git-commit.md`：简短 Conventional Commit，英文 header；
  如需 body，先英文后 `Chinese:` 对应中文。
- 可按“接口/标定”“executor/轨迹”“仿真/测试/文档”分批提交，但仅提交已验证且逻辑完整的阶段。

## 完成门槛

- 所有冻结设计均有对应代码路径、配置和自动化测试，不以文档占位代替实现。
- TCP 上游协议与 `references/runtime` 文件保持不变。
- `src` 各受影响仓库无意外文件、缓存或用户修改被覆盖。
- 工作空间根目录完整 `catkin build` 成功；相关自动测试无失败。
- 对必须依赖真实 RTK、15 机 PX4 SITL 或实机的剩余验证给出明确操作步骤和风险，不虚报通过。
