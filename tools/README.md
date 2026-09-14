# tools：实机运维工具（日志下载 / 离线分析 / 时钟修正）

> 位置：`src/tcp_to_ros/tools/`（地面站侧仓库，随 `tcp_to_ros` 一起克隆到 GCS_A / 开发机）。
> 这些脚本**不是 ROS 节点**：不参与 `catkin build` 安装（`CMakeLists.txt` 只装 `scripts/`），
> 由操作员直接 `python3` / `bash` 运行。机载电脑**不需要**它们（机载未安装 pymavlink / pyulog）。
> 证据与日志的归档位置、命名规则见 `src/swarm_uav_interfaces/real_setup.md` §8。

## 0. 依赖

```bash
pip3 install --user pymavlink pyulog     # 实测：pymavlink 2.4.41、pyulog 1.1.0（python3.8）
sudo apt install sshpass                 # 可选：机载 ssh 用密码（无密钥）时 `log_fetch.sh` 自动用它
```

- 只用 `pymavlink`（与飞控通话）和 `pyulog`（读 `.ulg`），**不依赖 ROS**，无需 `source devel/setup.bash`。
- `pyulog` 另提供 `ulog_info` / `ulog2csv`（装在 `~/.local/bin`），可做交叉校验。

## 1. 文件清单（平铺在 `tools/` 根下 + 前缀命名）

命名约定：**前缀说清「碰谁」**——`log_*` = 一轮取件总入口；`fcu_*` = 走 MAVLink **直连飞控**（在线）；
`onboard_*` = **ssh 到机载电脑**（在线；修时钟 / 起停栈）；`ulog_*` = 读 `.ulg` **文件**（离线，不碰链路）。
`.py` 用 `python3` 跑、`.sh` 用 `bash` 跑；除 `configs/` 外没有子目录。

| 文件 | 用途 |
|---|---|
| `log_fetch.sh` | **日志下载（推荐入口）**：一轮收全飞控 ulog + 机载 ROS 会话日志 + planner 诊断 + 终端转录 |
| `fcu_ulog_list.py` | 列飞控 SD 上的 ulog 清单（`LOG_REQUEST_LIST`），并把 `time_utc` 换算成本地时刻 |
| `fcu_ulog_fetch.py` | 下载 ulog（`LOG_REQUEST_DATA` 分块 + 断点续传 + 进度 + stall 重试） |
| `fcu_nsh.py` | 跑**只读** PX4 nsh 命令（`ls` / `mount` / `param show ...`），走 `SERIAL_CONTROL`；**不在下载链里**（见 §3.1） |
| `ulog_zcheck.py` | ulog 快照：时长、local/global 位置、`actuator_armed`、`nav_state`、offboard 控制模式 |
| `ulog_cmds.py` | ulog 内 `vehicle_command` 汇总，解码 `SET_GPS_GLOBAL_ORIGIN(48)` |
| `onboard_clock_fix.sh` | 时钟修正：本机系统时钟 →（多次指定）机载系统时钟 → 机载 RTC（治「上电/重启回 1970」） |
| `onboard_ctl.sh` | **机载栈远程启停**：`start/stop/status/logs` 多机扇出，实际动作调机载 `bashs/*_onboard.sh`（本机跑，见 §5.1） |
| `configs/onboard.env.example` | 机载账号/密码 + IP 规则模板（`onboard_clock_fix.sh` / `log_fetch.sh` / `onboard_ctl.sh` **共用同一份**） |
| `configs/onboard.env` | 上者的本地实参（含明文密码，已 gitignore；别提交、别 push） |


## 2. `log_fetch.sh`：日志下载（一轮收全，推荐入口）

一轮实机测试的日志散在三处，本脚本一把收齐。**本轮范围（最小档）**：

| 层 | 收什么 | 来源 |
|---|---|---|
| 飞控 | `fcu/ulog_list.txt`（全清单，含 GPS UTC）、`fcu/log_<id>.ulg`（最新 N 条）、`fcu/ulog_fetch.log` | MAVLink `LOG_REQUEST_LIST` / `LOG_REQUEST_DATA`（固定 UDP `:15500`） |
| 机载 | `onboard/ros_home/<session>/`（rosout、各节点、master、roslaunch） | ssh：`~/catkin_swarm6-2/.ros_home/log/<最新 session>/` |
| 机载 | `onboard/ego_planner/*.log`（planner 诊断，即 `diagnostic_log_dir`） | ssh：`~/catkin_swarm6-2/runtime_logs/ego_planner/` |
| 机载 | `onboard/term/*_offboard_ego.log`（终端全量转录，§5.1.2 方式 B） | ssh：`~/catkin_swarm6-2/.tmp/logs/` |

**本轮不收**（有意跳过）：nsh 快照（需要时按 §3 手动跑）、本机 GCS_A 侧 `.ros_home`、
**旧 launch 的旁路日志**（机上 `~/.tmp/*.log`，如 `px4_multi.log`、`offb_hover*.log`：
那是早期可用性测试用的 `px4_multi.launch` / 悬停脚本留下的，当前机载入口 `uav_offboard_ego_real.launch` 不产生）。
理由：机载终端转录已盖住启动链输出；GCS 侧与非当前入口的旁路日志按需另收。

```bash
# 收 UAV7：最新 1 条 ulog + 机载日志
LOG_OUT=~/Downloads/flight_logs ./src/tcp_to_ros/tools/log_fetch.sh 7

# 多台：每台取最新 3 条 ulog（脚本内部串行，见 §6 坑 6）
LOG_OUT=~/Downloads/flight_logs ./src/tcp_to_ros/tools/log_fetch.sh -n 3 1 2 6 14

# 机载 ssh 不通时只收飞控
./src/tcp_to_ros/tools/log_fetch.sh -s -n 1 7
```

| 选项 | 默认 | 说明 |
|---|---|---|
| `-n N` | `1` | 每台取最新 N 条 ulog；`0` = 只列清单不下载 |
| `-c FILE` | `configs/onboard.env` | 参数文件（与 `onboard_clock_fix.sh` 共用机载凭据/编址） |
| `-t SEC` / `-u USER` | 参数文件值 | ssh 连接超时 / ssh 登录用户 |
| `-s` | — | 跳过机载阶段（只收飞控 ulog） |

目标写法：裸数字（`7`）= UAVn，同时决定飞控 MAVLink（`sys=n, comp=1`）与本机推 IP 规则
（`IP_PREFIX.IP_STRIDE*n+IP_OFFSET`）；也接受 `UAV7`、`uav7`、区间 `1-6`、`host`、`user@host`（后两者收不到 ulog）。

输出（`LOG_OUT` 默认当前目录；`UAVn_<时间>` 及其子目录全由脚本生成）：

```
$LOG_OUT/UAV7_2026-09-14_1900/
├── README.md       本轮摘要（目标 / 机载事实 / 板钟告警 / 复现命令）
├── manifest.txt    逐项：OK|MISSING|FAIL|TRUNCATED|SIZE-MISMATCH + 来源 / 字节 / md5
├── md5sums.txt     `md5sum -c md5sums.txt` 校验用
├── fcu/            ulog_list.txt、log_*.ulg、ulog_fetch.log
└── onboard/
    ├── ros_home/<session>/…    机上整目录 tar 流回后解包
    ├── ego_planner/*.log       机上 >32 MB 时只留尾部（manifest 出 TRUNCATED 行）
    └── term/*_offboard_ego.log
```

- **stamp 取本机时钟**（`UAVn_YYYY-MM-DD_HHMM`，多目标共用同一个 stamp）；真实飞行时刻用 `fcu/ulog_list.txt`
  的 GPS UTC（`utc+8` 列）锚定，不用机载板钟（§6 坑 3）。
- 机载阶段**只开一趟 ssh**：远端脚本 base64 传入、`tar czf - | base64` 流回，**机上不落任何临时文件**。
- 凭据：参数文件里 `BOARD_SSH_PASS` 非空且装了 `sshpass` 时自动用它（`sshpass -e`，不走命令行参数）；
  否则退回密钥 / 交互输入。
- 退出码：`0` 全部成功；`1` 参数/配置错；`2` 至少一台有 `FAIL` 项。

## 3. `fcu_*.py`：MAVLink 直连飞控（底层工具，`log_fetch.sh` 内部调 list/fetch）

链路：开发机 / GCS_A ──UDP `:15500`── 机载 `mavlink-routerd` ──串口 921600── 飞控。
**所有机载 routerd 都往同一个 `:15500` 推**，所以必须用 `-t` 钉住目标：routerd 自己也会发一路
`HEARTBEAT`（`autopilot=8`），自动探测会认错（§6 坑 1）。

```bash
# 列清单（-t 必填：裸数字 = UAVn；也可写显式 7:1）
python3 src/tcp_to_ros/tools/fcu_ulog_list.py -t 7 -n 12

# 下载：LOG_OUT=<落盘目录> -t 7（无 id → 最新 -n 条；有 id → 按 id，半截文件原地续传）
LOG_OUT=~/Downloads/flight_logs/UAV7_2026-09-14_1900/fcu \
  python3 src/tcp_to_ros/tools/fcu_ulog_fetch.py -t 7 -n 2
LOG_OUT=<同上> python3 src/tcp_to_ros/tools/fcu_ulog_fetch.py -t 7 62 63 64

# 只读查板卡状态（不会写盘、不改参数）
python3 src/tcp_to_ros/tools/fcu_nsh.py -t 7 'ls /fs/microsd' 'mount' 'param show SDLOG_MODE'
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `-t N` | 无（**必填**） | 裸数字 = UAVn（`-t 7` 等价 `7:1`）；也接受显式 `sys:comp` |
| `-n LAST` | list `12` / fetch `2` | `fcu_ulog_list.py`：打印最新 N 行，`0` = 全表；`fcu_ulog_fetch.py`：无 id 时取最新 N 条 |
| `LOG_OUT` | 当前目录 | **唯一环境变量**（`fcu_ulog_fetch.py` 落盘目录） |
| id … | — | `fcu_ulog_fetch.py` 位置参数：显式 ulog id（已下满的 skip） |

- MAVLink 源（`udpin:0.0.0.0:15500`）、`CHUNK_TIMEOUT`（1.0 s）、`STALL_LIMIT`（6）、`CHUNK_COUNT`（65535）
  都是脚本内常量：**不传参、不设环境变量**。
- 退出码：`0` 正常；`2` 未收到 `LOG_ENTRY`（链路/目标不对，或飞控侧 SD 读取异常）；
  `3` 有日志没下完（SHORT，已下载部分保留，可原地续传）。

### 3.1 `fcu_nsh.py` 单独干什么（**不在**下载链上）

`log_fetch.sh` 只调 `fcu_ulog_list.py` / `fcu_ulog_fetch.py`，**从不调 nsh**；三个脚本之间也没有 import
（各自独立起 pymavlink 连接）。`fcu_nsh.py` 是飞控侧**独立取证**工具：当 ulog 列不出/下不动（怀疑卡坏、
分区没挂、参数被改），或要核固件与板载状态时，直接开一条 nsh 问飞控。

```bash
# 卡挂没挂、ulog 写不写得进（SDLOG_MODE=1 才是开机自动记录）
python3 src/tcp_to_ros/tools/fcu_nsh.py -t 7 'ls /fs/microsd' 'mount' 'param show SDLOG_MODE' 'ver'
# 启动异常看开机 dmesg（SD 卡报错常落在这里）
python3 src/tcp_to_ros/tools/fcu_nsh.py -t 7 'dmesg' 'free'
```

- nsh = **NuttShell**（PX4 跑在 NuttX 上，飞控控制台上那个 `nsh>` 就是它）；命令按 nsh 语法写，不是 Linux。
- 走 `SERIAL_CONTROL` 的 `DEV_SHELL`，flags 带 `EXCLUSIVE` → **独占会话**：同一时刻别的 MAVLink console
  （如 QGC 的 MAVLink Console）进不来；跑完工具会补发一帧释放 shell。
- 一次给多条命令会**依次**发，每条收满「链路静默 2 s」为止 → 输出混在一起时拆成单条跑。
- 心跳过滤要求 `-t` 指定的 sysid 且 `autopilot == 12`（PX4）→ 避开 routerd 自己发的 `autopilot=8`。
- **只准读**：nsh 写命令（`param set` / `mkfs` / `reboot`）没有二次确认，此工具不做。

## 4. `ulog_*.py`：离线分析（只读 `.ulg` 文件，不连任何链路）

```bash
python3 src/tcp_to_ros/tools/ulog_zcheck.py <ULOG_DIR>/log_64.ulg
python3 src/tcp_to_ros/tools/ulog_cmds.py  <ULOG_DIR>/log_6*.ulg
```

- `ulog_zcheck.py`：一行式快照，用来判「有没有 arm / 有没有进 OFFBOARD / 位置是相对原点还是绝对偏移」。
  例：`vehicle_local_position.z` 从 `-232.81` 到 `-227.14` = 相对 local 原点爬升 5.7 m，
  与规划侧 `takeoff_height_m: 5.0` 对得上，**不是乱飞**。
- `ulog_cmds.py`：看飞控实际收到的指令。注意 `SET_GPS_GLOBAL_ORIGIN(48)` 若在**解锁前**注入，
  日志尚未开始记录 → 查不到不等于没注入。
- 交叉校验（可选）：`ulog_info <log>.ulg` 看 `duration` 与 `No Dropouts`，用来判取件完整性。

## 5. `onboard_clock_fix.sh`：时钟修正（本机 → 机载系统时钟 → 机载 RTC）

机载无 NTP：上电/重启后系统时钟停在 `1970-01-01`；若 RTC 从未写入（或板子没有 RTC），重启又会回到 1970。
`onboard_clock_fix.sh` 一次做两步：**① 本机系统时钟 → 机载系统时钟**（`sudo date -s @<本机 epoch>`）；
**② 机载系统时钟 → 机载 RTC**（`sudo hwclock --systohc`，设备按 `RTC_CANDIDATES` 探测）。
两步都回读复核（`|Δ| <= TOLERANCE_S` 才 PASS），并可**一轮批量修多台**。

```bash
# 首次：把参数文件建好（含明文密码，已 gitignore，别提交/别 push）
cp src/tcp_to_ros/tools/configs/onboard.env.example \
   src/tcp_to_ros/tools/configs/onboard.env && chmod 600 src/tcp_to_ros/tools/configs/onboard.env
$EDITOR src/tcp_to_ros/tools/configs/onboard.env     # 填 BOARD_SSH_USER / BOARD_SSH_PASS / BOARD_SUDO_PASS

# 干跑：只打印，不改任何时钟
./src/tcp_to_ros/tools/onboard_clock_fix.sh -n 1 2 4 11 16

# 真修：一次修 1 2 4 11 16 五台（凭据从参数文件读，不在命令行里）
./src/tcp_to_ros/tools/onboard_clock_fix.sh 1 2 4 11 16
```

- **目标写法**：裸数字 = 无人机编号（`1 2 4 11 16` → UAV1/2/4/11/16）；也支持区间 `1-6`、`UAV7`、`uav7`、
  `host`、`user@host`（后者只覆盖该台的 ssh 用户）。
- 选项：`-n` 干跑、`-c FILE` 换参数文件、`-t SEC` ssh 超时、`-u USER` 覆盖 ssh 用户。
- 输出：`TARGET / SYS_DELTA / SYS_TIME / RTC_TIME / RTC_DEV / RTC_DELTA / RESULT`；
  `PASS` 全对齐；`WARN(no RTC device: reboot resets to 1970)` = 系统时钟已修但板子无 RTC；
  `FAIL(...)` 带 `sudo_rc/sys_rc/rtc_rc` 便于定位（本机时钟早于 2020 直接拒绝执行）。
- 退出码：`0` 全部成功；`1` 参数/配置/本机时钟错；`2` 至少一台失败。
- **必须在启动 ROS 栈之前跑**：修正前启动的进程，其日志时间戳仍是 1970 基（`real_setup.md` §5.1.1 第 0 步）。
- 参数文件（`configs/onboard.env`，机载账号/密码全机相同，**只记一份**；`log_fetch.sh` 也读它）：
  `BOARD_SSH_USER` / `BOARD_SSH_PASS` / `BOARD_SUDO_PASS` / `SSH_TIMEOUT` / `TOLERANCE_S` /
  `RTC_CANDIDATES` / `IP_PREFIX` / `IP_STRIDE` / `IP_OFFSET`（`UAVn = IP_PREFIX.(IP_STRIDE*n + IP_OFFSET)`）。
  环境变量 `BOARD_SSH_PASS` / `BOARD_SUDO_PASS` / `SSH_TIMEOUT` 优先级高于参数文件。
  `BOARD_SUDO_PASS` 留空 = 机载免密 sudo（脚本改用 `sudo -n`）。
  注：`BOARD_SSH_PASS` 只喂远端 sudo；ssh 本身走密钥/交互 —— `log_fetch.sh` 额外支持装了 `sshpass` 时用它。
- 密码经 stdin 交给 `sudo -S`，远端脚本整体 base64 传输，密码里的空格/引号/`$` 不会破坏命令（已实测）。

### 5.1 `onboard_ctl.sh`：机载栈远程启停（本机 → 机载 `bashs/*_onboard.sh`）

只在 **GCS 本机**跑；真正起停由**机载电脑**上既有的
`<ws>/src/swarm_uav_executor/bashs/{start,stop}_onboard.sh` 完成（**那两个脚本 0 改动**，机载仍可手工跑）。
`<ws>` 默认机载 `~/catkin_swarm6-2`；要改就在参数文件里设 `ONBOARD_WS`（机载上的绝对路径）。

```bash
./src/tcp_to_ros/tools/onboard_ctl.sh status 1-6                          # 谁在跑（一行一台）
./src/tcp_to_ros/tools/onboard_ctl.sh start 1 2 4                         # 起（分离式：ssh 立刻返回）
./src/tcp_to_ros/tools/onboard_ctl.sh start 7 -- interfaces_version:=v2   # `--` 之后原样交给 roslaunch
./src/tcp_to_ros/tools/onboard_ctl.sh stop 1-6                            # 停（机载脚本自己 INT→TERM→KILL）
./src/tcp_to_ros/tools/onboard_ctl.sh logs 7 [-f]                         # 看/跟机载终端转录
./src/tcp_to_ros/tools/onboard_ctl.sh -n start 1-6                        # 干跑：只打印将执行的远程动作
```

- 目标写法与 `onboard_clock_fix.sh` 完全一致（裸数字 / 区间 / `UAV7` / `host` / `user@host`）；
  选项 `-n` / `-c FILE` / `-t SEC` / `-u USER` / `-f`（logs 跟读）/ `-h`；参数文件同 §5 那份。
- 退出码：`0` 全部成功；`1` 参数/配置错；`2` 至少一台 FAIL（`status` 只在「连不上」时算 FAIL，`DOWN` 不算）。
- 输出：`start` 报 `PROCS/STARTED/TRANSCRIPT`；`stop` 报 `LEFT`（残留 roslaunch 数）；
  `status` 报 `PROCS/LOG_TS/BYTES` + 「本轮启动」行。FAIL 时追加机载详情块
  （`stop` 的命中进程列表 / `start` 的机载 stderr 尾部）并提示 `logs <目标>`。
- **机载脚本缺失 = MISSING**：该机代码没同步时 `start`/`stop` 直接 FAIL（不静默成功），`status` 提示缺哪个文件。
- **分离式启动**：`setsid` 分离 + stdin/stdout 丢弃，ssh 立即返回，断开 ssh / 关终端都不影响机载
  （持久性不靠 zellij，也不留后台会话）；拉起后默认 3 s（`SETTLE_S`）回查 `pgrep` 判成败。
- **为什么绕一道 python3**（而不是朴素的 `ssh host 'nohup ... &'`）：非交互 shell 的后台任务会被塞
  `SIGINT=SIG_IGN`，一路继承到 roslaunch → 机载 `stop_onboard.sh` 的 SIGINT 失效，只能等它 10 s 后退化到
  SIGTERM 硬杀（roslaunch 没机会收摊，节点可能变孤儿）。而 **shell 无权重置自己继承来的 ignore**（POSIX；
  `set -m` 只在「本 shell 自己没被 ignore」时才管用），所以启动改成
  `python3 -c '…SIG_DFL…' exec bash 机载脚本`（实测：python 能重置，`script`/roslaunch 都保得住默认处置）。
- 远端脚本整段 base64 传输（密码、`:=` 等特殊字符不经过 ssh 命令行）；`pgrep` 匹配串只出现在被 base64 的
  脚本里，不会自匹配 ssh 包装 shell。

## 6. 踩过的坑（先读再动手）

1. **飞控心跳要认准 `sys:comp`**：`mavlink-routerd` 自己会发一路 `HEARTBEAT`（`autopilot=8`），
   `wait_heartbeat()` 会把它当成目标 → 必须 `-t <飞控 sys>`（实测 UAV7 = `7`，即 `7:1`）。
2. **`LOG_ENTRY` 空列表 ≠ 卡里没日志**：实测同一天 18:37 返回空列表（`mavftp list` 还超时），
   19:06 同一条链路列出 65 条 → 先重启/重试再判卡坏。
3. **板钟不跨重启**：每次上电回 1970（`timedatectl`：`System clock synchronized: no`）。
   用 `onboard_clock_fix.sh` 修完系统时钟后还要**写进 RTC**（`hwclock --systohc`）才可能跨重启；
   若 `RESULT` 报 `WARN(no RTC device …)` 说明该板没有可用 RTC，只能每次上电重跑修正。
   本轮未修正时 ROS 日志时间戳不可用；此时 ulog 的 `LOG_ENTRY.time_utc`（GPS UTC）是唯一真实时刻锚点。
   `log_fetch.sh` 会把这个事实写进归档目录的 `README.md`（板钟告警行）。
4. **下载慢**：实测 ≈1.8 MB/min（3 MB ≈ 2.5 min）；日志里出现 `stall` 重试属正常。
5. **同一文件可能跨时钟基**：现场旧 `px4_multi.launch` 的旁路日志 `~/.tmp/px4_multi.log` 同进程内从
   `[8616.83]` 跳到 `[1789296434.52]`（对时瞬间）→ **不能用文件 mtime 反推事件时刻**。
   该 launch 与其日志属旧用法（当前机载入口不用、也不采集），这里只借它说明现象；
   本工作区自己的 ROS 日志同样以 1970 为基点，真时刻一律回到 ulog 的 `LOG_ENTRY.time_utc`。
6. **多台别同时下 ulog**：`:15500` 是所有机载 routerd 共用的一个端口，`LOG_REQUEST_DATA` 的分块会互相
   抢带宽、交错错序 → 必须一台一台串行（`log_fetch.sh` 已串行；手工跑时也要注意）。
7. **旧笔记里的命令写法已作废**（两层原因）：① 2026-09-14 起 `MAV_TARGET` / `MAV_SRC` / `LAST` / `ULOG_OUT`
   全部改成命令行参数（只保留 `LOG_OUT`）；② 同日起工具按 §1 的平铺 + 前缀改名（见 §7.1 对照表）。
   `.tmp/notes/…`、`test_records/…` 里 `MAV_TARGET=7:1 ULOG_OUT=<dir> python3 ulog_fetch_fcu.py` 这类命令
   照抄必然失败 → 换成 `LOG_OUT=<dir> python3 src/tcp_to_ros/tools/fcu_ulog_fetch.py -t 7`。
8. **改名余波：`log_fetch.sh` 的 `TOOLS_DIR` 曾指到上一层**（2026-09-14 修）：它算成 `tools/..`，
   于是 `DEFAULT_CONFIG` 找 `src/tcp_to_ros/configs/onboard.env`、`fcu_ulog_*.py` 也按 `src/tcp_to_ros/` 找 →
   不带 `-c` 直接 `ERROR: config file not found` 退 1。现改为 `TOOLS_DIR=$SCRIPT_DIR`（.py 与 configs/ 都在 `tools/` 下）。
   碰到「路径类」报错先怀疑改名的余波。
9. **后台起的机载栈会让机载 `stop_onboard.sh` 的 SIGINT 失效**（`onboard_ctl.sh` 已绕开）：
   非交互 shell 的 `cmd &` 会给子进程塞 `SIGINT=SIG_IGN`，并一路继承到 `script`/roslaunch；
   此时 SIGINT 无效果，stop 拖到 10 s 后 SIGTERM，roslaunch 没机会收摊 → 节点可能变孤儿。
   修法是让**程序**（`python3`）重置 `SIG_DFL` 后再 `exec` 机载脚本（见 §5.1）。

## 7. 与文档/记录的关系

- 部署与启动流程：`src/swarm_uav_interfaces/real_setup.md`（§3.2 时钟修正、§5.1.1 第 0 步核板钟、§8 日志与证据）。
- 一轮实测的完整取件记录（UAV7，2026-09-13）：`src/tcp_to_ros/test_records/` 对应记录、
  归档目录示例 `~/Downloads/flight_logs/UAV7_2026-09-13_1900/`（含 `md5sums.txt`）；
  现场笔记与一次性脚本已归档 `.tmp/archive_260913/`（`uav7_260913/` 部署/取件/实测三份 + `log_dl_stub_test/` 桩测试）；
  目录名规则 `UAVn_<YYYY-MM-DD_HHMM>` 与归档先例 `UAV6-260911-1930`、`UAV6_2026-09-13_1756` 不同，
  **新归档以 `log_fetch.sh` 生成的形状为准**（旧目录保留原样，不回改）。
- 工程验证脚本（ego_smoke、acceptance、plot）另有落点：工作区 `verification/`（不入 git），与本目录职责不同。

### 7.1 改名对照（2026-09-14，C 方案：平铺 + 前缀）

| 旧路径 | 现在 |
|---|---|
| `logs/fetch_logs.sh` | `log_fetch.sh` |
| `fcu/ulog_list_fcu.py` | `fcu_ulog_list.py` |
| `fcu/ulog_fetch_fcu.py` | `fcu_ulog_fetch.py` |
| `fcu/nsh_fcu.py` | `fcu_nsh.py` |
| `ulog/ulog_zcheck.py` | `ulog_zcheck.py` |
| `ulog/ulog_cmds.py` | `ulog_cmds.py` |
| `real/clock_fix.sh` | `onboard_clock_fix.sh` |
| `configs/clock_sync.env` / `.env.example` | `configs/onboard.env` / `configs/onboard.env.example` |

- 历史记录（`.tmp/notes/…`、`test_records/…`、`.tmp/archive_260913/`）**保留旧名不回改**；
  照抄前先对照本表。`tools/configs/onboard.env` 仍被 gitignore（含明文密码）。
- 行为不变：只改名 + 改默认配置路径（`-c` 可照旧覆盖）；两套离线桩测试（`.tmp/archive_260913/clock_fix_stub_test/`、
  `.tmp/archive_260913/log_dl_stub_test/`）都在新名下重跑通过。
