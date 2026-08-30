# 参数配置规范化与 `safe_valley_exp` 拆包实施计划

**计划日期：** 2026-08-30 18:00（文件生成时间，24 小时制）  
**状态：** 计划已建立，未实施  
**来源：** `.tmp/notes/code_review_260829_parameter_standardization.md`

## 1. 实施目标

完成参数配置规范化审查中确认的修复：

```text
canonical YAML 是默认值唯一来源
→ roslaunch 命令行是唯一可选覆盖方式
→ launch 不再保存第二套业务默认
→ safe_valley_exp 拆包并删除
→ 依赖、文档、测试与实际启动链同步
```

## 2. 实施边界

### 保留

- 当前 planner、executor、Safety、relay、bridge、GPS bias、GCS/TCP 功能。
- `references/` 目录及其内容，不修改。
- 当前 1.0 m 三维距离门禁、Safety active、planner/relay 30 Hz、candidate timeout 0.2 s、TimedTrajectory 0.1 s。
- 当前外部 arm/OFFBOARD 责任边界。

### 删除或迁移

- `safe_valley_exp` 中已完成且与当前项目无关的 flock 示例及残留辅助。
- 迁移完成后删除 `safe_valley_exp` 包。

### 不做

- 不新增显式 `override` 参数。
- 不把 runtime JSON 改成 ROS tuning 文件。
- 不在本计划中实现未经验证的实机 takeoff agent、real launch 或新 takeoff service。
- 不通过关闭 Safety、降低门禁、减少 UAV、提高 limits 或启用 swap 绕过测试失败。

## 3. Canonical YAML 与 launch 覆盖修复

### 3.1 规则

```text
默认值：canonical YAML/参数表
命令行覆盖：roslaunch name:=value
launch <arg>：仅作为命令行接口，默认空值
同名 <param>：仅在命令行显式传入时条件写入
```

不新增显式 override 参数，不保留 launch 内业务默认值。

### 3.2 文件级修改

| 文件 | 动作 | 要求 |
|---|---|---|
| `src/tcp_to_ros/launch/gcs_a_backend.launch` | 修改 | backend 业务 arg 改为空默认；删除无条件同名 `<param>`；保留 YAML 加载 |
| `src/tcp_to_ros/launch/gcs_a_backend_test.launch` | 修改 | 同上；仅保留故障注入差异 |
| `src/tcp_to_ros/launch/gcs_a_backend_fail.launch` | 修改 | 同上；仅保留故障注入差异 |
| `src/tcp_to_ros/launch/tcp_to_ros_mock.launch` | 修改 | transport 默认来自 YAML；命令行显式传入时条件覆盖 |
| `src/ego_planner_driver/launch/ego_planner_driver.launch` | 修改 | `safety_supervisor_mode` 使用 YAML 默认，命令行可选覆盖 |
| `src/swarm_uav_executor/launch/uav_executor_ego.launch` | 修改 | planner/executor 同名业务参数改为空 arg 条件传参 |
| `src/swarm_uav_executor/launch/uav_executor_mavros.launch` | 修改 | legacy 默认来自 legacy YAML；删除固定同名业务 param |
| `src/swarm_uav_executor/launch/uav_executor_mock.launch` | 修改 | mock 默认来自 YAML；故障值由命令行传入 |
| 所有 canonical YAML | 修改 | 每条参数增加中文作用、单位、取值语义注释；必要处增加公式 |
| 各相关参数测试 | 新增/修改 | 覆盖 YAML 默认、命令行覆盖、最终 ROS 参数值 |

### 3.3 验证

对每个入口执行两组测试：

```text
不传 roslaunch 参数 → rosparam get == YAML 值
传入 name:=value → rosparam get == 命令行值
```

测试至少覆盖：

```text
gcs_a_backend*.launch
tcp_to_ros_mock.launch
ego_planner_driver.launch
uav_executor_ego.launch
uav_executor_mavros.launch
uav_executor_mock.launch
```

## 4. `safe_valley_exp` 拆包与删除

### 4.1 新建 `swarm_simulation`

新建 package：

```text
src/swarm_simulation/
```

迁移：

```text
safe_valley_exp/launch/multi_uav_ego_15sim.launch
safe_valley_exp/launch/multi_uav_ego_4sim.launch
safe_valley_exp/launch/multi_uav_sim.launch
safe_valley_exp/launch/multi_uav_sim_headless.launch
```

目标：

```text
swarm_simulation/launch/
```

同步新增：

```text
package.xml
CMakeLists.txt
README.md
launch 安装规则
```

仅放 Gazebo/PX4 SITL 环境，不放 planner、executor、GCS 或 bridge。

### 4.2 机载支持迁入 `swarm_uav_executor`

迁移：

```text
safe_valley_exp/launch/uav_offboard_ego.launch
safe_valley_exp/startup_offboard_ego.sh
safe_valley_exp/scripts/gps_bias_node.py
safe_valley_exp/config/gps_bias_defaults.yaml
```

目标：

```text
swarm_uav_executor/launch/uav_offboard_ego.launch
swarm_uav_executor/scripts/startup_offboard_ego.sh
swarm_uav_executor/scripts/gps_bias_node.py
swarm_uav_executor/config/gps_bias_defaults.yaml
```

同步修改：

```text
$(find ...) 路径
include 链路
CMake 安装项
package.xml 依赖
GPS bias 测试路径
README 启动说明
```

GPS bias 归类为 onboard time-sync utility：

```text
发布者：gps_bias_node
传输者：swarm_topology_bridge
消费者：ego_planner_driver
```

bridge 只转发，不加入 bias 估计逻辑。

### 4.3 SITL takeoff helper

迁移并重命名：

```text
safe_valley_exp/scripts/offboard_takeoff_15.py
→ swarm_uav_executor/scripts/offboard_takeoff_15_sitl.py
```

同步修改所有：

```text
launch
shell
README
CMake/install
测试
```

当前脚本只服务 SITL 仿真。实机 takeoff 后续采用独立 agent：

```text
GCS takeoff command
→ 每机 onboard takeoff agent
→ 本机 arm/OFFBOARD/takeoff
→ 状态回报
```

本计划不实现该实机 agent。

### 4.4 删除残留辅助

删除：

```text
safe_valley_exp/scripts/wait_mavros.py
safe_valley_exp/scripts/submode_publisher.py
safe_valley_exp/scripts/rosbag_record.py
```

同步清理：

```text
CMakeLists.txt 安装项
README.md / README-en.md 引用
TODO.md / TODO-en.md 引用
当前 launch 引用
flock 配置、launch、源码和专属文档引用
```

不得误删：

```text
gps_bias_node.py
uav_offboard_ego.launch
startup_offboard_ego.sh
offboard_takeoff_15_sitl.py
GPS bias tests
```

### 4.5 删除 `safe_valley_exp`

完成迁移、构建、测试和启动验证后：

```text
删除 src/safe_valley_exp
```

`build/`、`devel/`、`.ros_home/` 中的生成/缓存副本不作为源码迁移目标；通过全环境 clean 清理。

## 5. bridge 依赖修复

### 文件级修改

| 文件 | 动作 |
|---|---|
| `src/swarm_topology_bridge/package.xml` | 增加 `geographic_msgs` 的 build/build_export/exec 依赖 |
| `src/swarm_topology_bridge/CMakeLists.txt` | `find_package` 和 `catkin_package` 增加 `geographic_msgs` |
| `src/swarm_topology_bridge/test/*` | 增加动态消息类型解析/依赖检查 |

验证：

```text
rosdep check swarm_topology_bridge
catkin build swarm_topology_bridge
install-space 启动 topology_group_a_sim.launch
确认 geographic_msgs/GeoPointStamped 可解析
```

## 6. PyYAML 测试依赖修复

### 文件级修改

| 文件 | 动作 |
|---|---|
| `src/tcp_to_ros/package.xml` | 增加 `<test_depend>python3-yaml</test_depend>` |
| `src/tcp_to_ros/test/test_parameter_wiring.py` | 保持 PyYAML 解析逻辑，补充依赖失败提示 |
| catkin/CI 测试入口 | 纳入该测试 |

在只按 package manifest 安装依赖的干净环境执行测试收集；记录 PyYAML 版本和结果。

## 7. topology README 文档

### 文件级修改

| 文件 | 动作 |
|---|---|
| `src/swarm_topology_bridge/README-en.md` | 删除 |
| `src/swarm_topology_bridge/README.md` | 作为唯一文档；修正 topology 路径，补 launch、`uav_name`、owner、配置加载说明 |
| 其他源码文档/索引 | 删除 `README-en.md` 链接 |

不修改 `references/`。

## 8. 测试顺序

1. 文件路径、引用和 package manifest 静态检查。
2. YAML 解析、中文注释、参数 schema 检查。
3. canonical YAML 默认加载测试。
4. roslaunch 命令行覆盖测试。
5. planner、executor、backend、transport 相关包测试。
6. `rosdep check` 和 install-space 构建/启动测试。
7. `swarm_simulation` 独立 SITL 启动测试。
8. `swarm_uav_executor/uav_offboard_ego.launch` 机载组合启动测试。
9. GPS bias 测试、offboard SITL takeoff 测试。
10. 全工作区测试。
11. 全环境 clean 后 15-UAV prepare 重跑。

每次失败先诊断；不单机补丁、不跳过门禁。

## 9. 验收标准

### 配置

- canonical YAML 是每个参数唯一默认来源。
- launch 不再保存第二套业务默认。
- roslaunch 命令行传参可覆盖 YAML。
- supported 参数全部被实际节点读取。
- 每条 YAML 参数有中文作用/单位/语义注释。
- 依赖、路径、namespace、identity、端口检查通过。

### 拆包

- `swarm_simulation` 可独立启动当前 4/15 机 SITL。
- `swarm_uav_executor` 可启动机载 MAVROS/EGO/GPS bias 链路。
- `offboard_takeoff_15_sitl.py` 路径和调用正确。
- 源码不再依赖 `safe_valley_exp`。
- `safe_valley_exp` 删除后工作区可完整构建。
- flock、wait、submode、rosbag 残留引用清零。

### 依赖与文档

- bridge 可解析 `geographic_msgs/GeoPointStamped`，依赖声明完整。
- tcp_to_ros wiring 测试在干净环境可收集并运行。
- 只维护中文 `README.md`，无 `README-en.md` 残留链接。
- `references/` 无改动。

### 运行回归

- planner、executor、relay、Safety、GPS bias、bridge、GCS/TCP 关键链路无回归。
- 当前 15-UAV prepare 从全 clean 成功完成。
- 测试记录包含实际参数快照、完整命令、原始输出、时间、提交和未验证项。

## 10. 提交边界

- `ego_planner_driver`、`swarm_uav_executor`、`safe_valley_exp`、`swarm_topology_bridge`、`tcp_to_ros`、新 `swarm_simulation` 按嵌套仓库分批提交。
- 按需 commit，不 push。
- 删除 `safe_valley_exp` 前必须先完成迁移验证。
- 不覆盖或回滚用户已有修改。
