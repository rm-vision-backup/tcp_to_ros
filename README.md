# tcp-to-ros

`tcp_to_ros` 是 Group A 地面站的 ROS1 Noetic 适配包。第一版只做数据适配，不直接控制真实无人机。

当前范围：

- 作为 Group A TCP Server 接收 `references/runtime` 发来的 JSON Lines 命令；
- 校验 `MOVE_TO`、`FOLLOW_ROUTE`、`FAULT_EXIT`、`HOVER`、`HOLD`；
- 将整组命令转换为 `/group_a/execute_task` Action Goal；
- 通过 mock backend 自动生成 `ACCEPTED` / `COMPLETED` / `FAILED` / 超时场景；
- 发布 `/group_a/group_task_state`；
- 提供 `/group_a/emergency_hold` Service。

## 目录

- `action/ExecuteGroupTask.action`：整组任务 Action；
- `msg/`：任务参数与状态消息；
- `srv/EmergencyHold.srv`：紧急悬停服务；
- `scripts/tcp_server_node.py`：TCP Server 节点；
- `scripts/mock_backend_node.py`：参数化 mock backend；
- `scripts/task_event_injector.py`：手动注入 hold 的小工具；
- `config/mock_backend.yaml`：mock 策略配置；
- `launch/tcp_to_ros_mock.launch`：本机联调启动文件。

## 启动

默认 ROS 基础环境使用 `~/.bashrc` 中已有的：

- `source /opt/ros/noetic/setup.bash`
- `source ~/catkin_gazebo/devel/setup.bash`

在此基础上，编译 `tcp_to_ros` 后再额外 source 当前工作空间的 `devel/setup.bash`。

建议使用默认的 `build`、`devel`、`logs` 目录：

```bash
source ~/.bashrc
catkin build tcp_to_ros
source /home/ub20tg/catkin_swarm6-2/devel/setup.bash
roslaunch tcp_to_ros tcp_to_ros_mock.launch
```

如果默认 `build` / `devel` / `logs` 被旧的 `catkin_make` 产物占用，先删除后重新初始化源码空间，再编译：

```bash
source ~/.bashrc
rm -rf /home/ub20tg/catkin_swarm6-2/build \
       /home/ub20tg/catkin_swarm6-2/devel \
       /home/ub20tg/catkin_swarm6-2/logs
catkin_init_workspace /home/ub20tg/catkin_swarm6-2/src
catkin build tcp_to_ros
source /home/ub20tg/catkin_swarm6-2/devel/setup.bash
```

若当前环境无法写默认 `~/.ros`，可改用工作区内目录：

```bash
export ROS_HOME=/home/ub20tg/catkin_swarm6-2/.ros_home
export ROS_LOG_DIR=/home/ub20tg/catkin_swarm6-2/.ros_home/log
mkdir -p "$ROS_LOG_DIR"
```

## 单任务联调

以本机 `127.0.0.1:39001` 为例：

```bash
python3 references/runtime/task_test.py --backend tcp --group A --task prepare --tcp-host 127.0.0.1 --tcp-port 39001
python3 references/runtime/task_test.py --backend tcp --group A --task coverage-segment-1 --tcp-host 127.0.0.1 --tcp-port 39001
python3 references/runtime/task_test.py --backend tcp --group A --task fault-exit --tcp-host 127.0.0.1 --tcp-port 39001
python3 references/runtime/task_test.py --backend tcp --group A --task recovery --tcp-host 127.0.0.1 --tcp-port 39001
python3 references/runtime/task_test.py --backend tcp --group A --task coverage-segment-2 --tcp-host 127.0.0.1 --tcp-port 39001
python3 references/runtime/task_test.py --backend tcp --group A --task hold-end --tcp-host 127.0.0.1 --tcp-port 39001
```

## mock 配置

`config/mock_backend.yaml` 支持按命令类型覆写：

- `accept_delay_s`
- `result_delay_s`
- `send_accepted_feedback`
- `final_status`
- `failed_uavs`
- `error_code`
- `message`

例如：

- `send_accepted_feedback: false` 且 `final_status: TIMEOUT` 可验证 runtime 的 `ACCEPTED` 超时与 `HOLD` 路径；
- `final_status: FAILED` 且 `failed_uavs: [A10]` 可验证失败聚合路径。
