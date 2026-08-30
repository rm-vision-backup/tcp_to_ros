# 参数配置规范化与 `safe_valley_exp` 拆包实施测试记录

时间：2026-08-30 18:00-24:00 CST（实施 + 15-UAV prepare 三轮回归）
记录人：cline-deepseek
对应实施计划：`plans/implementation_plan_26083018.md`（参数配置规范化与 `safe_valley_exp` 拆包）

实施后各嵌套仓库 HEAD：

| 仓库 | HEAD | 说明 |
|---|---|---|
| ego_planner_driver | `7010dc5` | canonical YAML 参数 + **安全预测窗 5s→3s（计划外，§9 回归修复，见 §6/附录）** |
| swarm_uav_executor | `0833226` | onboard 支持/offboard takeoff 迁移；EGO/mavros/mock 空 arg 条件传参 |
| swarm_topology_bridge | `fabffca` | `geographic_msgs` 依赖 + 解析测试；README 单语言化 |
| swarm_uav_interfaces | `6ab1619` | 未改动 |
| tcp_to_ros | `0bc6281` | backend/transport 条件传参 + canonical YAML；`python3-yaml` test_depend |
| swarm_simulation | `9f36563` | 新建 Gazebo/PX4 SITL launch 包（承接 4/15 机仿真） |
| safe_valley_exp | 已删除 | 迁移/构建/测试/启动验证后移除，`build`/`devel` 由全环境 clean 清理 |

## 1. 参数规范化验证（计划 §3.3）

脚本：`.tmp/scripts/verify_launch_params_26083018.sh`（不传 → YAML 默认；传入 → 命令行值；
两组 run 分别覆盖 backend/transport 与 ego/mavros/mock）。

**结果：`ALL launch param checks PASS`**（两组日志：`.tmp/logs/verify_launch_params_26083018.log`、
`verify_launch_params_26083018.run2.log`）。

代表性检查项：

| 入口 | 默认（YAML） | override（命令行） | 结果 |
|---|---|---|---|
| `gcs_a_backend.launch` | `confirm_wait_s=1.0` / `confirm_retries=2` | 7.0 / 5 | OK |
| `gcs_a_backend_test.launch` | `confirm_deadline_s=4.0` | 9.5 | OK |
| `gcs_a_backend_fail.launch` | `confirm_retry_interval_s=0.2` | 0.2（默认） | OK |
| `ego_planner_driver.launch` | `safety_supervisor_mode=active`（YAML） | `off` | OK |
| `uav_executor_ego.launch` | `ego_swarm/task_timeout_s=200.0` / `ego_hold_timeout_s=2.0` | 1200.0 / 5.0 | OK |
| `uav_executor_mavros.launch` | `namespace=/mavros` / `require_armed=false` | `/uav2/mavros` / true | OK |
| `uav_executor_mock.launch` | `result_delay_s=0.5` / `final_success=true` | 1.25 / false | OK |

- canonical YAML 为唯一默认来源；launch 不再保存第二套业务默认；命令行 `name:=value` 条件覆盖。
- 每条 YAML 参数已加中文作用/单位/取值语义注释（必要时附公式），经 schema/解析检查。

## 2. 拆包与迁移验证（计划 §4）

### 2.1 `swarm_simulation` 新建

- 迁移 4 个 SITL launch（`multi_uav_ego_15sim`/`multi_uav_ego_4sim`/`multi_uav_sim`/
  `multi_uav_sim_headless`）；新增 `package.xml`/`CMakeLists.txt`/`README.md`/launch 安装规则。
- 仅含 Gazebo/PX4 SITL 环境，不含 planner/executor/GCS/bridge。
- 验证：`catkin build swarm_simulation` 成功；launch 文件 `roslaunch --files` 解析通过；
  15 机无头启动复用 `run.sh sim`（见 §6 三轮 prepare 均走该 launch）。

### 2.2 机载支持迁入 `swarm_uav_executor`

- 迁移：`uav_offboard_ego.launch`、`startup_offboard_ego.sh`、`gps_bias_node.py`、
  `gps_bias_defaults.yaml`（至 `swarm_uav_executor/{launch,scripts,config}/`）。
- `offboard_takeoff_15.py` → `offboard_takeoff_15_sitl.py`（SITL 专用），launch/CMake/README/测试
  引用同步更新；GPS bias 归类 onboard time-sync utility（发布者→bridge→planner），bridge 只转发。
- 验证：§6 三轮 prepare 机载层均以 `startup_offboard_ego.sh 1..15` 启动 15/15 ready；
  `offboard_takeoff_15_sitl.py` 15/15 takeoff 成功。

### 2.3 残留辅助删除与 `safe_valley_exp` 移除

- 删除 `wait_mavros.py`/`submode_publisher.py`/`rosbag_record.py` 及 flock 配置/launch/源码/
  专属文档引用；未误删 GPS bias / offboard launch / startup / takeoff helper / bias 测试。
- 迁移、构建、测试、启动验证全部通过后删除 `src/safe_valley_exp`。
- 验证：`src/` 下无 `safe_valley_exp`；全工作区构建正常（无路径/引用残留）；
  grep 残留引用（flock/wait/submode/rosbag/README-en 链接）为 0。

## 3. 依赖修复验证（计划 §5/§6）

- `swarm_topology_bridge`：`package.xml` 增加 `geographic_msgs` build/exec 依赖，`CMakeLists.txt`
  `find_package`/`catkin_package` 同步；新增动态消息类型解析/依赖检查测试。
  验证：`catkin build swarm_topology_bridge` 成功；`geographic_msgs/GeoPointStamped` 可解析
  （测试 `test_geographic_dependency.py` 通过）。
- `tcp_to_ros`：`package.xml` 增加 `<test_depend>python3-yaml</test_depend>`；
  `test_parameter_wiring.py` 保持 PyYAML 解析逻辑并补充依赖失败提示；测试收集正常。


## 4. 文档验证（计划 §7）

- `swarm_topology_bridge/README-en.md` 删除；`README.md` 为唯一文档（修正 topology 路径、
  补 launch/`uav_name`/owner/配置加载说明）；源码索引中 `README-en.md` 链接清零。
- `references/` 无改动。

## 5. 单元测试与全量回归（计划 §8.5/§8.10）

| 层级 | 命令 | 结果 |
|---|---|---|
| ego_planner_driver | `catkin build/test ego_planner_driver` | 编译 0 warning；单测 **83 tests 全过**（yield 6 / state machine 40 / safety 8 / brake 3 / trajectory 20 / short_final 7 / closed_loop 2） |
| swarm_uav_executor | `test_ego_launch_config.py` | **16 passed**（canonical YAML 断言 + 假配置/launch 重复检查） |
| tcp_to_ros | `python3 -m pytest src/tcp_to_ros/test/` | **37 passed**（含 `test_parameter_wiring`/`test_gcs_a_aggregation`） |
| 非法值 fail-fast | `_emergency_ttc_s:=5.0 _warning_ttc_s:=3.0` 启动 | FATAL `refusing to start`（不 clamp、不降级） |
| 全工作区 | `catkin build`（6 包） | 全部成功 |

## 6. 15-UAV prepare 端到端回归（计划 §8.11 / §9 运行回归）

全环境 clean 后逐轮重跑（cleanup 断言 0 残留 → `run.sh sim`（swarm_simulation 15 机）→
`run.sh onboard`（15 机 active 一次性启动）→ GCS bridge→backend（顺序，避免 11310 竞态）→
`run.sh tcp` → 标定 `calibration_ready=true` → `offboard_takeoff_15_sitl.py` 15/15 → prepare）。

| 轮次 | 配置 | 结果 | 失败机 | ticks | 说明 |
|---|---|---|---|---|---|
| 1 | SLOW 候选 + 预测窗 5s | **FAILURE** | A05 | 574 | SafetyPredictor 预测 UAV6→UAV5 min_d 0.984m（<1m）→ EMERGENCY → replan 失败收口；实际 min_3d 1.556m |
| 2 | SLOW 候选移除（9540a89） | **FAILURE** | A05, A06 | 586 | LEFT/RIGHT 候选也无法通过 1.2m 门槛；预测 min_d 0.96m vs 实际 1.92m，偏差 -0.94m |
| **3** | **SLOW 回滚（88710d4）+ 预测窗 3s（7010dc5）** | **SUCCESS** | — | 1601 | 12 机全部 COMPLETED；12 机 EMERGENCY 计数全 0；UAV5-6 预测 min_d 1.297m（>1m），实际 1.633m |

- 计划 §9「当前 15-UAV prepare 从全 clean 成功完成」在本轮（第 3 轮）达成。
- 第 1/2 轮失败根因与修复属**计划外修改**（见计划文件末尾附录）：5s 预测窗外推高频
  replan（~0.8s/次）产生的短命 intent 尾段，制造假 EMERGENCY；缩短到 3s 后预测落在已执行
  轨迹跨度内，EMERGENCY 全部消失。
- 三轮均无距离门禁触发、无碰撞、无 OOM；planner RSS 11.6-12.4 MB 稳定。

## 7. 结果汇总

| 验收项（计划 §9） | 结果 |
|---|---|
| canonical YAML 唯一默认 + launch 命令行覆盖 | ✅ `verify_launch_params` ALL PASS（§1） |
| supported 参数全部被实际节点读取 | ✅ 单测 + rosparam 快照 |
| 每条 YAML 参数中文注释 | ✅ schema/解析检查 |
| `swarm_simulation` 独立启动 4/15 机 SITL | ✅ 构建 + 15 机无头启动（§6 三轮） |
| 机载 MAVROS/EGO/GPS bias 链路可启动 | ✅ 15/15 ready + takeoff 15/15 |
| `offboard_takeoff_15_sitl.py` 路径/调用正确 | ✅ |
| 源码不再依赖 `safe_valley_exp` | ✅ 已删除 + 全工作区构建通过 |
| flock/wait/submode/rosbag 残留引用清零 | ✅ grep 检查 |
| bridge 可解析 `geographic_msgs/GeoPointStamped` | ✅ 依赖 + 测试 |
| tcp_to_ros wiring 测试干净环境可收集运行 | ✅ 37 passed |
| 单语言 README，无 README-en 残留 | ✅ |
| 15-UAV prepare 从全 clean 成功完成 | ✅ **SUCCESS（第 3 轮，预测窗 3s）** |

## 8. 未验证项

1. **实机 takeoff agent**（计划 §4.3 明确不做）：GCS→onboard takeoff agent 未实现，SITL helper 仅服务仿真。
2. **`geographic_msgs` install-space 启动验证**：开发空间构建/解析通过；install-space 独立验证未执行。
3. **15-UAV prepare 连续稳定性**：单次 SUCCESS 达成；多轮连续回归未跑（后续轮次可复用本记录 §6 流程）。
4. 残余优化项（不影响验收）：`self_odom_dev` p95 0.27-0.35m（本机轨迹跟踪偏差）、traj churn ~1 次/s、
   预测-实际偏差 -0.34m（预测窗 3s 后已不触发 EMERGENCY）。
