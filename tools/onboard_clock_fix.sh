#!/usr/bin/env bash
# Fix the clock of the onboard companion computers (UAVn) from this host.
#
# 动作两步：1) 本机系统时钟 → 机载系统时钟；2) 机载系统时钟 → 机载 RTC。
# 背景：机载无 NTP，上电后系统时钟停在 1970-01-01；RTC 未写入则重启又回 1970。
# 必须在本轮启动 ROS 栈之前执行：real_setup.md §3.2 第 1 步、§5.1.1 第 0 步。
# 验收：① 板钟与该台写入**前现取**的本机（GCS）基准之差 ≤ TOLERANCE_S（默认 5 s；比的是「落点」，
#          不是「修前/修后步长」）；
#       ② RTC 回读（timedatectl → hwclock -f 设备 → /proc/driver/rtc 依次兜底）与板钟之差 ≤ RTC_TOLERANCE_S（默认 5 s）。
# 容差是**故意放宽**的（不要求秒级）：依据见本文件 TOLERANCE_S 默认值处的注释与 tools/README.md §5。
#
# Usage:
#   onboard_clock_fix.sh [选项] <目标> [目标 ...]
#   onboard_clock_fix.sh 1 2 4 11 16                    # 裸数字 = UAV1/2/4/11/16
#   onboard_clock_fix.sh -n 1-6                         # 干跑 UAV1..UAV6
#   onboard_clock_fix.sh -c tools/configs/onboard.env UAV7
#
# 目标写法：裸数字（1 2 4 11 16）或区间（1-6）= 无人机编号；也接受 UAVn、host、user@host。
#
# Options:
#   -n          干跑：只打印将执行的远端脚本，不改任何时钟
#   -c FILE     参数文件（默认 <tools>/configs/onboard.env）
#   -t SEC      ssh 连接超时秒数（覆盖参数文件）
#   -u USER     ssh 登录用户（覆盖参数文件）
#   -h          本帮助
#
# 参数文件（机载账号/密码全机相同，模板见 configs/onboard.env.example）：
#   BOARD_SSH_USER / BOARD_SSH_PASS / BOARD_SUDO_PASS / SSH_TIMEOUT / TOLERANCE_S（默认 5）
#   RTC_TOLERANCE_S（默认 5）/ RTC_CANDIDATES
#   IP_PREFIX / IP_STRIDE / IP_OFFSET  →  UAVn = IP_PREFIX.(IP_STRIDE*n + IP_OFFSET)
#
# 退出码：0 全部成功；1 参数/配置/本机时钟错误；2 至少一台失败

set -uo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DEFAULT_CONFIG="$SCRIPT_DIR/configs/onboard.env"

# —— 优先级：命令行（-u/-t）> 环境变量 > 参数文件 > 默认值 ——
# 先在赋默认值**之前**抓环境变量（此刻抓到的才是「用户真的设了」），source 参数文件后再放回。
env_user="${BOARD_SSH_USER:-}"
env_pass="${BOARD_SSH_PASS:-}"
env_sudopass="${BOARD_SUDO_PASS:-}"
env_timeout="${SSH_TIMEOUT:-}"

# —— 默认值（会被参数文件、环境变量、命令行依次覆盖）——
BOARD_SSH_USER="${USER:-ubuntu}"
BOARD_SSH_PASS=""
BOARD_SUDO_PASS=""
SSH_TIMEOUT=8
# —— 容差为什么是 5 s 而不是秒级（本工具只需「时间轴/日志能对齐」）——
#   · 机间 intent 时间戳对齐由 gps_bias 修正：bias = header.stamp − GPS UTC
#     （swarm_uav_executor/scripts/gps_bias_node.py），接收端 T_own = T_other − bias_other + bias_own
#     → 两端系统钟偏差在差分里被**抵消**，driver 的 age 检查与绝对钟偏差无关；
#   · 参考 swarm 规划器要到 10 s 才判「swarm time not synchronized」并丢轨迹
#     （references/swarm-main/.../ego_replan_fsm.cpp:684）；
#   · 本机 driver 的 intent 接收窗是 ±30 s（neighbor_intent_stale_s，safety_defaults.yaml）。
#   2 s 会因 ssh 往返 + 回读耗时误判 FAIL；5 s 留余量，又远小于上述门限。
TOLERANCE_S=5        # 系统钟 vs 该台写入前现取的本机（GCS）基准的允许偏差
RTC_TOLERANCE_S=5    # RTC vs 板钟的允许偏差：放宽，含 hwclock 写 + 回读自身的秒级耗时
IP_PREFIX="192.168.5"
IP_STRIDE=10
IP_OFFSET=1
RTC_CANDIDATES="/dev/rtc /dev/rtc0 /dev/rtc1"

DRY_RUN=0
CONFIG="$DEFAULT_CONFIG"
cli_user=""       # 命令行值单独记，等参数文件与环境变量都落地后再盖上（最高优先级）
cli_timeout=""

usage() { sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'; }

TARGETS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -n) DRY_RUN=1 ;;
    -c) CONFIG="${2:-}"; shift ;;
    -t) cli_timeout="${2:-}"; shift ;;
    -u) cli_user="${2:-}"; shift ;;
    -h|--help) usage; exit 0 ;;
    -*) echo "unknown option: $1" >&2; usage >&2; exit 1 ;;
    *) TARGETS+=("$1") ;;
  esac
  shift
done

if [ "${#TARGETS[@]}" -eq 0 ]; then
  echo "ERROR: no target given (e.g. 1 2 4 11 16 or UAV7)" >&2
  usage >&2
  exit 1
fi

# —— 参数文件：不存在时明确报错（凭据不再散落在命令行/环境里）——
if [ ! -f "$CONFIG" ]; then
  echo "ERROR: config file not found: $CONFIG" >&2
  echo "       cp $DEFAULT_CONFIG.example $DEFAULT_CONFIG && chmod 600 $DEFAULT_CONFIG" >&2
  exit 1
fi
# shellcheck source=/dev/null
. "$CONFIG"

# 覆盖顺序：默认值（上面已落地）→ 参数文件（刚 source）→ 环境变量 → 命令行（-u/-t）
[ -n "$env_user" ] && BOARD_SSH_USER="$env_user"
[ -n "$env_pass" ] && BOARD_SSH_PASS="$env_pass"
[ -n "$env_sudopass" ] && BOARD_SUDO_PASS="$env_sudopass"
[ -n "$env_timeout" ] && SSH_TIMEOUT="$env_timeout"
[ -n "$cli_user" ] && BOARD_SSH_USER="$cli_user"
[ -n "$cli_timeout" ] && SSH_TIMEOUT="$cli_timeout"

if [ -z "${BOARD_SSH_USER:-}" ]; then
  echo "ERROR: BOARD_SSH_USER not set in $CONFIG" >&2
  exit 1
fi

# —— 本机时钟自检：把机载改成错的比不改更糟 ——
LOCAL_NOW=$(date +%s)
if [ "$LOCAL_NOW" -lt 1600000000 ]; then
  echo "ERROR: local clock looks wrong (epoch=$LOCAL_NOW, $(date '+%F %T %Z'))." >&2
  echo "       Fix this host clock first (e.g. sudo date -s '@<real epoch>')." >&2
  exit 1
fi
# —— 目标展开：输出 "label ip [ssh_user]"（label 只用于打印）——
# 裸数字 7 / 区间 1-6 / UAV7 / uav7 → UAVn，IP 由 IP_PREFIX/STRIDE/OFFSET 推出；
# host、user@host 透传（user@host 只覆盖该台的 ssh 用户）。
expand_targets() {
  local t a b n num
  for t in "$@"; do
    case "$t" in
      [0-9]*-[0-9]*)
        a="${t%%-*}"; b="${t##*-}"
        if ! [[ "$a" =~ ^[0-9]+$ && "$b" =~ ^[0-9]+$ ]]; then
          echo "ERROR: bad range: $t" >&2; return 1
        fi
        if [ "$a" -gt "$b" ]; then
          echo "ERROR: bad range (start > end): $t" >&2; return 1
        fi
        for ((n = a; n <= b; n++)); do
          printf 'UAV%s %s.%s\n' "$n" "$IP_PREFIX" "$((IP_STRIDE * n + IP_OFFSET))"
        done ;;
      [0-9]*)
        printf 'UAV%s %s.%s\n' "$t" "$IP_PREFIX" "$((IP_STRIDE * t + IP_OFFSET))" ;;
      [Uu][Aa][Vv][0-9]*)
        num="${t:3}"
        if ! [[ "$num" =~ ^[0-9]+$ ]]; then echo "ERROR: bad UAV id: $t" >&2; return 1; fi
        printf 'UAV%s %s.%s\n' "$num" "$IP_PREFIX" "$((IP_STRIDE * num + IP_OFFSET))" ;;
      *@*@*)
        echo "ERROR: bad target: $t" >&2; return 1 ;;
      *@*)
        printf '%s %s %s\n' "${t#*@}" "${t#*@}" "${t%%@*}" ;;
      *)
        printf '%s %s\n' "$t" "$t" ;;
    esac
  done
}

# 先展开一次：区间/编号写法在此处校验，错了直接退出，不进 ssh 流程
TARGET_LINES=$(expand_targets "${TARGETS[@]}") || exit 1

# —— 远端脚本（bash；经 base64 传输，密码等特殊字符不经过 ssh 命令行）——
remote_script() {
  local epoch="$1" pass="$2" rtc="$3" pass_b64 rtc_b64
  pass_b64=$(printf '%s' "$pass" | base64 -w0)
  rtc_b64=$(printf '%s' "$rtc" | base64 -w0)
  cat <<REMOTE
set -u
epoch=$epoch
sudo_pass_b64=$pass_b64
rtc_b64=$rtc_b64
pass=\$(printf '%s' "\$sudo_pass_b64" | base64 -d)
rtc_candidates=\$(printf '%s' "\$rtc_b64" | base64 -d)

rtc_devs=''
for d in \$rtc_candidates; do
  [ -e "\$d" ] || continue
  real=\$(readlink -f "\$d" 2>/dev/null || echo "\$d")
  case " \$rtc_devs " in *" \$real "*) continue ;; esac   # /dev/rtc 常是 symlink，去重避免同一路写两遍
  rtc_devs="\$rtc_devs \$real"
done
rtc_devs=\${rtc_devs# }
rtc_dev=\${rtc_devs%% *}

# 写入前先取 RTC 现状（作为「修之前是否停在 1970」的证据）
rtc_before=\$(timedatectl show -p RTCTime --value 2>/dev/null || echo n/a)
[ -z "\$rtc_before" ] && rtc_before=n/a

sudo_rc=0
if [ -n "\$pass" ]; then
  printf '%s\n' "\$pass" | sudo -S -v >/dev/null 2>&1 || sudo_rc=\$?
else
  sudo -n -v >/dev/null 2>&1 || sudo_rc=\$?
fi

sys_before=\$(date +%s)
sys_rc=90
if [ "\$sudo_rc" -eq 0 ]; then
  if sudo -n date -s "@\$epoch" >/dev/null 2>&1; then sys_rc=0; else sys_rc=\$?; fi
fi
sys_after=\$(date +%s)
sys_str=\$(date '+%F %T %Z')

# 拨钟后必须**重新预授权**：板钟从 1970 跳到 2026 时，sudo 时间戳里记的是 1970 的时刻
# （~1.9e3），而此刻已是 ~1.79e9 → sudo 认为时间戳「瞬间过期」→ 之后所有 sudo -n 都退化成
# "sudo: a password is required"（2026-09-14 实测：7 台未修过的板 date -s 成功、hwclock 全失败）。
if [ -n "\$pass" ]; then
  printf '%s\\n' "\$pass" | sudo -S -v >/dev/null 2>&1 || true
else
  sudo -n -v >/dev/null 2>&1 || true
fi

# 每路 RTC 都「写 + 回读」，且写和读用**同一个 -f 设备**：
#   hwclock --systohc（不带 -f）写的是内核默认 RTC，而 /dev/rtc 常是 symlink（本板 → rtc1）→
#   写 rtc0、读 /dev/rtc(=rtc1) 就会读到 1970 的**假 FAIL**（2026-09-14 UAV1 实测）；
#   而且只写默认那路时，另一路留在 1970，重启后从哪路恢复取决于内核/udev → 两路都要写。
# 写失败必须回显 stderr（rtc_wrerr）：这一路曾被 >/dev/null 2>&1 吞掉，导致「只报 rc=1 不知为何」。
rtc_rc=90
rtc_report=''; rtc_wrerr=''
if [ "\$sudo_rc" -ne 0 ]; then
  rtc_rc=90
elif [ -z "\$rtc_devs" ]; then
  rtc_rc=91
else
  rtc_rc=0
  for d in \$rtc_devs; do
    werr=''
    if [ -n "\$pass" ]; then
      werr=\$(printf '%s\\n' "\$pass" | sudo -S hwclock --systohc -f "\$d" 2>&1 >/dev/null) || rtc_rc=1
    else
      werr=\$(sudo -n hwclock --systohc -f "\$d" 2>&1 >/dev/null) || rtc_rc=1
    fi
    # 只留第一行**真错误**：sudo -S 会把 "[sudo] password for xxx:" 提示写进 stderr（不是错误）
    werr=\$(printf '%s' "\$werr" | grep -v -e '\[sudo\]' -e 'password for' | head -1)
    [ -n "\$werr" ] && rtc_wrerr="\$rtc_wrerr [\$d \$(printf '%s' "\$werr" | head -1)]"
    e=''
    if [ -n "\$pass" ]; then
      e=\$(printf '%s\\n' "\$pass" | sudo -S hwclock --show -f "\$d" 2>/dev/null | tail -1)
    else
      e=\$(sudo -n hwclock --show -f "\$d" 2>/dev/null | tail -1)
    fi
    ee=''
    [ -n "\$e" ] && ee=\$(date -d "\$e" +%s 2>/dev/null || true)   # 空串会被 date -d "" 解析成「今天 0 点」
    rtc_report="\$rtc_report \$d=\${ee:-n/a}"
  done
  rtc_report=\${rtc_report# }
  rtc_wrerr=\${rtc_wrerr# }
fi

rtc_after=''; rtc_src=''
# RTC 回读：多源兜底。本板实测 timedatectl show -p RTCTime --value 返回**空**且 rc=0
# （systemd degraded）→ 只靠它会让 RTC 段静默 n/a 并给出「没验 RTC 的 PASS」（2026-09-14 UAV13 实测）。
# 注：本 heredoc 是**不带引号**的，注释里绝不能用反引号或裸命令替换——它们会被当命令替换**在本机执行**
#   （2026-09-14 实测两次：注释里提到 hwclock/sudo 时带了反引号，结果每台目标前都在本机真跑了一次
#    hwclock / sudo。自检：awk 截出本函数体后 grep 未转义反引号与未转义 \$\( ，见桩 README 第 4 条）。
rtc_after=\$(timedatectl show -p RTCTime --value 2>/dev/null || true)
[ -n "\$rtc_after" ] && rtc_src=timedatectl
if [ -z "\$rtc_after" ]; then
  if [ -n "\$pass" ]; then
    rtc_after=\$(printf '%s\n' "\$pass" | sudo -S hwclock --show -f "\$rtc_dev" 2>/dev/null | tail -1)
  else
    rtc_after=\$(sudo -n hwclock --show -f "\$rtc_dev" 2>/dev/null | tail -1)
  fi
  [ -n "\$rtc_after" ] && rtc_src=hwclock
fi
if [ -z "\$rtc_after" ] && [ -r /proc/driver/rtc ]; then
  rtc_after=\$(awk '\$1=="rtc_time"{t=\$3} \$1=="rtc_date"{d=\$3} END{if (d!="" && t!="") print d" "t" UTC"}' /proc/driver/rtc 2>/dev/null || true)
  [ -n "\$rtc_after" ] && rtc_src=/proc/driver/rtc
fi
rtc_epoch=''
if [ -n "\$rtc_after" ]; then rtc_epoch=\$(date -d "\$rtc_after" +%s 2>/dev/null || true); fi
tze=\$(timedatectl show -p Timezone --value 2>/dev/null || echo n/a)
syncp=\$(timedatectl show -p NTPSynchronized --value 2>/dev/null || echo n/a)
adj=\$( [ -f /etc/adjtime ] && tail -1 /etc/adjtime 2>/dev/null || echo none )

echo "sudo_rc=\$sudo_rc"
echo "sys_rc=\$sys_rc"
echo "rtc_rc=\$rtc_rc"
echo "sys_before=\$sys_before"
echo "sys_after=\$sys_after"
echo "sys_str=\$sys_str"
echo "rtc_dev=\$rtc_dev"
echo "rtc_devs=\$rtc_devs"
echo "rtc_report=\$rtc_report"
echo "rtc_wrerr=\$rtc_wrerr"
echo "rtc_epoch=\$rtc_epoch"
echo "tz=\$tze"
echo "ntp_sync=\$syncp"
echo "adjtime=\$adj"
echo "rtc_before=\$rtc_before"
echo "rtc_after=\$rtc_after"
echo "rtc_src=\$rtc_src"
REMOTE
}

# —— ssh 执行：一次连接跑完全部动作，不做交互提示 ——
# 有密码（且本机装了 sshpass）就走 sshpass；否则退回裸 ssh（依赖密钥/ssh-agent）。
pass_ok() {
  [ -n "$BOARD_SSH_PASS" ] && [ "$BOARD_SSH_PASS" != CHANGE_ME ] \
    && command -v sshpass >/dev/null 2>&1
}

run_remote() {
  local ip="$1" b64="$2" user="${3:-$BOARD_SSH_USER}"
  local -a cmd=(ssh -o "StrictHostKeyChecking=no" -o "UserKnownHostsFile=/dev/null" \
      -o "ConnectTimeout=$SSH_TIMEOUT" -o "NumberOfPasswordPrompts=1" \
      -o "PreferredAuthentications=password,keyboard-interactive,publickey" \
      "${user}@${ip}" "echo $b64 | base64 -d | bash")
  if pass_ok; then SSHPASS="$BOARD_SSH_PASS" sshpass -e "${cmd[@]}" </dev/null 2>&1
  else "${cmd[@]}" </dev/null 2>&1; fi
}
# 注：`</dev/null` 不是可有可无——真 ssh 会把本地 stdin 转发给远端命令，若它继承了 while 循环的
# stdin（herestring / 管道），就会把剩余目标行吃光 → **只处理第一台且 rc=0**（静默漏修）。
# 桩 bin/ssh 已复现该行为；配合下面的 fd 3 双保险。

val() { sed -n "s/^$1=//p" <<<"$2" | head -1; }

# —— 主流程 ——
status=0
printf '%-14s %-10s %-21s %-29s %-8s %-10s %s\n' \
       TARGET SYS_OFF SYS_TIME RTC_TIME RTC_DEV RTC_DELTA RESULT
printf '%s\n' "------------------------------------------------------------------------------------------------------"

while read -r label ip tuser <&3; do
  [ -n "${label:-}" ] || continue
  # 基准**逐台现取**：串行修多台时若共用一趟开头的 epoch，靠后的板会被写成「旧 epoch」（板间
  # 差 = 本轮用时）。现取后每台都贴着本机实时，差异只剩 ssh 往返（~0.5 s），
  # 台间那一丁点差分由 gps_bias 修正抵消。
  NOW=$(date +%s)
  remote=$(remote_script "$NOW" "$BOARD_SUDO_PASS" "$RTC_CANDIDATES")
  b64=$(printf '%s' "$remote" | base64 -w0)

  if [ "$DRY_RUN" -eq 1 ]; then
    printf '%-14s %s\n' "$label" "DRY-RUN ${tuser:+$tuser@}$ip: set system clock to $NOW ($(date '+%F %T %Z')), then hwclock --systohc"
    continue
  fi

  out=$(run_remote "$ip" "$b64" "$tuser")

  sudo_rc=$(val sudo_rc "$out"); sys_rc=$(val sys_rc "$out"); rtc_rc=$(val rtc_rc "$out")
  sys_before=$(val sys_before "$out"); sys_after=$(val sys_after "$out")
  sys_str=$(val sys_str "$out")
  rtc_dev=$(val rtc_dev "$out"); rtc_epoch=$(val rtc_epoch "$out")
  rtc_before=$(val rtc_before "$out"); rtc_after=$(val rtc_after "$out")
  rtc_src=$(val rtc_src "$out")
  rtc_wrerr=$(val rtc_wrerr "$out")
  adjtime=$(val adjtime "$out")

  if [ -z "$sys_after" ]; then
    err=$(grep -v -e '^[a-z_]*=' -e '^Warning: Permanently added' <<<"$out" | head -1)
    printf '%-14s %-10s %-21s %-29s %-8s %-10s %s\n' \
           "$label" "-" "-" "-" "-" "-" "FAIL(unreachable: ${err:-empty reply})"
    status=2
    continue
  fi

  # 验收量 = 板钟相对 **该台写入前现取的基准 $NOW** 的偏差；修前/修后差值只说明「走了多少步」，
  # 不能当验收：板上只差 30 s 时老版本会误报 FAIL（2026-09-14 UAV13 实测，见 tools/README §5）。
  sys_off=$((sys_after - NOW))
  sys_off_abs=${sys_off#-}
  sys_step=$((sys_after - sys_before))
  # RTC：远端对**每一路**存在的设备都写过 + 读过（rtc_report="dev=epoch …"）；验收取偏差最大的那一路
  # （任一路不对都可能让重启回 1970），明细里列出全部。
  rtc_all=''; rtc_worst_abs=-1
  for pair in $(val rtc_report "$out"); do
    d=${pair%%=*}; e=${pair##*=}
    if [[ "$e" =~ ^[0-9]+$ ]]; then
      dd=$((e - sys_after)); ad=${dd#-}
      rtc_all+="${d##*/}:${dd}s "
      if [ "$ad" -gt "$rtc_worst_abs" ]; then rtc_worst_abs="$ad"; rtc_dev="$d"; rtc_epoch="$e"; fi
    else
      rtc_all+="${d##*/}:n/a "
    fi
  done
  rtc_delta="n/a"; rtc_disp="n/a"; rtc_time_disp="n/a"
  if [[ "$rtc_epoch" =~ ^[0-9]+$ ]]; then
    rtc_delta=$((rtc_epoch - sys_after))
    rtc_disp="${rtc_delta}s"
    rtc_time_disp=$(date -d "@$rtc_epoch" '+%F %T %Z')
  elif [ -n "$rtc_after" ]; then
    rtc_time_disp="$rtc_after"
  fi

  result="PASS"
  if [ "$sys_rc" != "0" ]; then
    result="FAIL(system clock: sudo_rc=$sudo_rc sys_rc=$sys_rc)"
  elif [ "$sys_off_abs" -gt "$TOLERANCE_S" ]; then
    result="FAIL(system clock off baseline by ${sys_off}s)"
  elif [ "$rtc_rc" -eq 91 ]; then
    result="WARN(no RTC device: reboot resets to 1970)"
  elif [ "$rtc_rc" != "0" ]; then
    result="FAIL(rtc write: rtc_rc=$rtc_rc)"
  elif [ "$rtc_delta" = "n/a" ]; then
    result="WARN(rtc readback unavailable: reboot persistence not confirmed)"
  elif [ "${rtc_delta#-}" -gt "$RTC_TOLERANCE_S" ]; then
    result="FAIL(rtc off by ${rtc_delta}s)"
  fi

  printf '%-14s %-10s %-21s %-29s %-8s %-10s %s\n' \
         "$label" "${sys_off}s" "$sys_str" "$rtc_time_disp" "${rtc_dev:-none}" "$rtc_disp" "$result"
  echo "    base=${NOW} sys_before=${sys_before} sys_step=${sys_step}s rtc_all=${rtc_all:-n/a} rtc_src=${rtc_src:-none} rtc_before=${rtc_before} adjtime=${adjtime}${rtc_wrerr:+ rtc_wrerr=$rtc_wrerr}"

  case "$result" in
    PASS|WARN*) ;;
    *) status=2 ;;
  esac
done 3<<<"$TARGET_LINES"

exit "$status"
