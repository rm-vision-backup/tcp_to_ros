# tools：实机运维工具（飞控 ulog 取件 / 离线分析 / 板钟对时）

> 位置：`src/tcp_to_ros/tools/`（地面站侧仓库，随 `tcp_to_ros` 一起克隆到 GCS_A / 开发机）。
> 这些脚本**不是 ROS 节点**：不参与 `catkin build` 安装（`CMakeLists.txt` 只装 `scripts/`），
> 由操作员直接 `python3` / `bash` 运行。机载电脑**不需要**它们（机载未安装 pymavlink / pyulog）。
> 证据与日志的归档位置、命名规则见 `src/swarm_uav_interfaces/real_setup.md` §8。

## 0. 依赖

```bash
pip3 install --user pymavlink pyulog     # 实测：pymavlink 2.4.41、pyulog 1.1.0（python3.8）
```

- 只用 `pymavlink`（与飞控通话）和 `pyulog`（读 `.ulg`），**不依赖 ROS**，无需 `source devel/setup.bash`。
- `pyulog` 另提供 `ulog_info` / `ulog2csv`（装在 `~/.local/bin`），可做交叉校验。

## 1. 目录

| 路径 | 用途 |
|---|---|
| `fcu/ulog_list_fcu.py` | 列飞控 SD 上的 ulog 清单（`LOG_REQUEST_LIST`），并把 `time_utc` 换算成本地时刻 |
| `fcu/ulog_fetch_fcu.py` | 下载 ulog（`LOG_REQUEST_DATA` 分块 + 断点续传 + 进度 + stall 重试） |
| `fcu/nsh_fcu.py` | 跑**只读** PX4 nsh 命令（`ls` / `mount` / `param show ...`），走 `SERIAL_CONTROL` |
| `ulog/ulog_zcheck.py` | ulog 快照：时长、local/global 位置、`actuator_armed`、`nav_state`、offboard 控制模式 |
| `ulog/ulog_cmds.py` | ulog 内 `vehicle_command` 汇总，解码 `SET_GPS_GLOBAL_ORIGIN(48)` |
| `real/board_time_sync.sh` | 从本机给机载电脑板钟对时（治「上电/重启回 1970」） |

## 2. `fcu/`：直连飞控取件

链路：开发机 / GCS_A ──UDP `:15500`── 机载 `mavlink-routerd` ──串口 921600── 飞控。

```bash
# 0) 先确认链路通（能收到飞控心跳；不通提前退出码 2）
MAV_TARGET=7:1 python3 src/tcp_to_ros/tools/fcu/ulog_list_fcu.py

# 1) 取最新 2 条（默认 LAST=2）
MAV_TARGET=7:1 ULOG_OUT=~/Downloads/flight_logs/UAV7_$(date +%y%m%d%H%M)/px4_ulogs \
  python3 src/tcp_to_ros/tools/fcu/ulog_fetch_fcu.py

# 2) 指定 id（可多条；已下满的会 skip，中断后可原地续传）
MAV_TARGET=7:1 ULOG_OUT=<同上> python3 src/tcp_to_ros/tools/fcu/ulog_fetch_fcu.py 62 63 64

# 3) 只读查板卡状态（不会写盘、不改参数）
MAV_TARGET=7:1 python3 src/tcp_to_ros/tools/fcu/nsh_fcu.py 'ls /fs/microsd' 'mount' 'param show SDLOG_MODE'
```

环境变量：

| 变量 | 默认 | 说明 |
|---|---|---|
| `MAV_SRC` | `udpin:0.0.0.0:15500` | MAVLink 源；对应机载 `mavlink-routerd -e <本机IP>:15500` |
| `MAV_TARGET` | 空（自动探测） | `sys:comp`，如 UAV7 = `7:1`；**建议显式指定**（见 §5 坑 1） |
| `ULOG_OUT` | 当前目录 | 落盘目录（`ulog_fetch_fcu.py`） |
| `LAST` | `2` / `12` | `ulog_fetch_fcu.py`：默认取最新 2 条；`ulog_list_fcu.py`：列表显示 12 行 |
| `CHUNK_TIMEOUT` / `STALL_LIMIT` | `1.0` / `6` | 静默超时（s）、连续静默上限（超限收工，已下载部分保留） |

退出码：`0` 正常；`2` 未收到 `LOG_ENTRY`（链路/目标不对，或飞控侧 SD 读取异常）。

## 3. `ulog/`：离线分析

```bash
python3 src/tcp_to_ros/tools/ulog/ulog_zcheck.py <ULOG_DIR>/log_64.ulg
python3 src/tcp_to_ros/tools/ulog/ulog_cmds.py  <ULOG_DIR>/log_6*.ulg
```

- `ulog_zcheck.py`：一行式快照，用来判「有没有 arm / 有没有进 OFFBOARD / 位置是相对原点还是绝对偏移」。
  例：`vehicle_local_position.z` 从 `-232.81` 到 `-227.14` = 相对 local 原点爬升 5.7 m，
  与规划侧 `takeoff_height_m: 5.0` 对得上，**不是乱飞**。
- `ulog_cmds.py`：看飞控实际收到的指令。注意 `SET_GPS_GLOBAL_ORIGIN(48)` 若在**解锁前**注入，
  日志尚未开始记录 → 查不到不等于没注入。
- 交叉校验（可选）：`ulog_info <log>.ulg` 看 `duration` 与 `No Dropouts`，用来判取件完整性。

## 4. `real/`：板钟对时

```bash
# 先看：不同时执行，-n 只打印命令
./src/tcp_to_ros/tools/real/board_time_sync.sh -n -u ubuntu UAV7

# 真对：本机取 epoch → 远端 sudo date -s → 回读复核（|Δ| ≤ 2 s 判 PASS）
BOARD_SUDO_PASS=<机载 sudo 密码> ./src/tcp_to_ros/tools/real/board_time_sync.sh -u ubuntu UAV7 UAV8
```

- **必须在启动 ROS 栈之前跑**：对时前启动的进程，其日志时间戳仍是 1970 基（`real_setup.md` §5.1.1 第 0 步）。
- 环境变量：`BOARD_SSH_PASS`（ssh 密码，不设则走密钥/agent）、`BOARD_SUDO_PASS`（远端 sudo 密码，不设则要求免密 `sudo -n`）。
- 目标写法：`UAVn`（自动解析为 `192.168.5.(10n+1)`）、或 `host`、或 `user@host`。
- 本机时钟先自检：`epoch < 1.6e9` 直接拒绝执行（避免把机载对成错的）。

## 5. 踩过的坑（先读再动手）

1. **飞控心跳要认准 `sys:comp`**：`mavlink-routerd` 自己会发一路 `HEARTBEAT`（`autopilot=8`），
   `wait_heartbeat()` 会把它当成目标 → 必须 `MAV_TARGET=<飞控 sys:comp>`（实测 UAV7 = `7:1`）。
2. **`LOG_ENTRY` 空列表 ≠ 卡里没日志**：实测同一天 18:37 返回空列表（`mavftp list` 还超时），
   19:06 同一条链路列出 65 条 → 先重启/重试再判卡坏。
3. **板钟不跨重启**：每次上电回 1970（`timedatectl`：`System clock synchronized: no`），
   本轮 ROS 日志时间戳不可用；此时 ulog 的 `LOG_ENTRY.time_utc`（GPS UTC）是唯一真实时刻锚点。
4. **下载慢**：实测 ≈1.8 MB/min（3 MB ≈ 2.5 min）；日志里出现 `stall` 重试属正常。
5. **同一文件可能跨时钟基**：现场 `~/.tmp/px4_multi.log` 同进程内从 `[8616.83]` 跳到
   `[1789296434.52]`（对时瞬间）→ **不能用文件 mtime 反推事件时刻**。
6. **脚本名保留 `_fcu` 后缀**：已归档的笔记/记录（`.tmp/notes/…`、`test_records/`）按这些名字写命令，
   改名会让旧记录失效。

## 6. 与文档/记录的关系

- 部署与启动流程：`src/swarm_uav_interfaces/real_setup.md`（§3.2 对时、§5.1.1 第 0 步核板钟、§8 日志与证据）。
- 一轮实测的完整取件记录（UAV7，2026-09-13）：`src/tcp_to_ros/test_records/` 对应记录、
  归档目录示例 `~/Downloads/flight_logs/UAV7_2026-09-13_1900/`（含 `md5sums.txt`）。
- 工程验证脚本（ego_smoke、acceptance、plot）另有落点：工作区 `verification/`（不入 git），与本目录职责不同。
