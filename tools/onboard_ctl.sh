#!/usr/bin/env bash
# 机载栈远程启停：在本机（地面站）一条命令拉起/停掉任意若干台 UAV 的机载 ROS 栈。
# 分工：本脚本只做「目标展开 + ssh 扇出 + 结果汇总」；真正的起停动作跑在**机载电脑**上的
#   <ws>/src/swarm_uav_executor/bashs/{start,stop}_onboard.sh（ws 默认 ~/catkin_swarm6-2）。
#   某台没同步代码（脚本不存在）→ 该台报 MISSING，不会静默成功。
#   启动是**分离式**的：ssh 立刻返回，断开 ssh / 关终端都不影响机载（持久性不靠 zellij/后台会话）。
# Usage:
#   onboard_ctl.sh <start|stop|status|logs> <目标> [目标 ...] [-- <roslaunch 参数>]
#   onboard_ctl.sh status 1-6                              # 谁在跑（一行一台）
#   onboard_ctl.sh start 1 2 4                             # 起 UAV1/2/4
#   onboard_ctl.sh start 7 -- interfaces_version:=v2       # `--` 之后的参数原样交给 roslaunch
#   onboard_ctl.sh stop 1-6                                # 停（SIGINT→TERM→KILL，见机载 stop 脚本）
#   onboard_ctl.sh logs 7                                  # 看机载终端转录尾部
#   onboard_ctl.sh logs 7 -f                               # 跟读（Ctrl-C 退出）
# 目标写法：裸数字（7）= UAVn / 区间（1-6）/ UAV7 / uav7 / host / user@host（与 onboard_clock_fix.sh 一致）
# Options:
#   -n        干跑：只打印将执行的远程动作，不 ssh
#   -c FILE   参数文件（默认 <tools>/configs/onboard.env）
#   -t SEC    ssh 连接超时秒数（覆盖参数文件）
#   -u USER   ssh 登录用户（覆盖参数文件）
#   -f        logs：跟在文件尾（tail -f）
#   -h        本帮助
# 参数文件（与 log_fetch.sh / onboard_clock_fix.sh **共用同一份**）：BOARD_SSH_USER / BOARD_SSH_PASS /
#   SSH_TIMEOUT / IP_PREFIX / IP_STRIDE / IP_OFFSET（UAVn = IP_PREFIX.(IP_STRIDE*n + IP_OFFSET)）；
#   可选 ONBOARD_WS = 机载工作空间**绝对路径**（默认空 = 交给机载用 $HOME 展开 ~/catkin_swarm6-2）。
#   覆盖顺序：命令行 `-u` / `-t` > 环境变量 > 参数文件 > 内置默认值。
#   另有环境变量 TAIL_N（logs 尾部行数，默认 30）/ SETTLE_S（start 后回查等待秒，默认 3）可调。
# 输出：每台一行结果（start: PROCS/STARTED/TRANSCRIPT；stop: LEFT；status: PROCS/LOG 时间），
#   FAIL 时附机载详情块（stop 的命中列表 / start 的机载 stderr）。
# 退出码：0 全部成功；1 参数/配置错；2 至少一台失败（status 只在「连不上」算失败，DOWN 不算）
# =============================================================================

set -uo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DEFAULT_CONFIG="$SCRIPT_DIR/configs/onboard.env"

# —— 优先级：环境变量 > 参数文件 > 默认值 ——
# 先在赋默认值**之前**抓环境变量（此刻抓到的才是「用户真的设了」），source 参数文件后再放回。
env_user="${BOARD_SSH_USER:-}"
env_pass="${BOARD_SSH_PASS:-}"
env_timeout="${SSH_TIMEOUT:-}"
env_ws="${ONBOARD_WS:-}"

SSH_TIMEOUT="${SSH_TIMEOUT:-8}"
BOARD_SSH_USER="${BOARD_SSH_USER:-${USER:-ubuntu}}"
BOARD_SSH_PASS="${BOARD_SSH_PASS:-}"
IP_PREFIX="${IP_PREFIX:-192.168.5}"
IP_STRIDE="${IP_STRIDE:-10}"
IP_OFFSET="${IP_OFFSET:-1}"
ONBOARD_WS="${ONBOARD_WS:-}"
TAIL_N="${TAIL_N:-30}"       # logs：默认看尾部多少行
SETTLE_S="${SETTLE_S:-3}"    # start：分离拉起后等几秒再回查进程

DRY_RUN=0
FOLLOW=0
CONFIG="$DEFAULT_CONFIG"
CMD=""
ARGS=()
TARGETS=()
cli_timeout=""     # 命令行 -t / -u 先记下来，等参数文件与环境变量都落地后再盖上去（CLI 最高优先级）
cli_user=""

# 与机载 stop_onboard.sh 同一个匹配串：只认 roslaunch 本体（python），不碰 ssh/包装 shell
LAUNCH_PAT='bin/roslaunch .*uav_offboard_ego_real\.launch'

usage() { sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
  case "$1" in
    -n) DRY_RUN=1 ;;
    -c) CONFIG="${2:-}"; shift ;;
    -t) cli_timeout="${2:-}"; shift ;;
    -u) cli_user="${2:-}"; shift ;;
    -f) FOLLOW=1 ;;
    -h|--help) usage; exit 0 ;;
    --) shift; ARGS=("$@"); break ;;
    -*) echo "unknown option: $1" >&2; usage >&2; exit 1 ;;
    *) if [ -z "$CMD" ]; then CMD="$1"; else TARGETS+=("$1"); fi ;;
  esac
  shift
done

case "$CMD" in
  start|stop|status|logs) ;;
  "") echo "ERROR: no command given (start|stop|status|logs)" >&2; usage >&2; exit 1 ;;
  *) echo "ERROR: unknown command '$CMD' (start|stop|status|logs)" >&2; usage >&2; exit 1 ;;
esac
if [ "${#TARGETS[@]}" -eq 0 ]; then
  echo "ERROR: no target given (e.g. 7 or 1 2 6 14)" >&2
  usage >&2
  exit 1
fi
if [ "${#ARGS[@]}" -gt 0 ] && [ "$CMD" != start ]; then
  echo "ERROR: '-- <roslaunch 参数>' 只对 start 有意义（当前命令：$CMD）" >&2
  exit 1
fi
if [ "$FOLLOW" -eq 1 ] && [ "$CMD" != logs ]; then
  echo "ERROR: -f 只对 logs 有意义（当前命令：$CMD）" >&2
  exit 1
fi
if [ -n "$cli_timeout" ] && { ! [[ "$cli_timeout" =~ ^[0-9]+$ ]] || [ "$cli_timeout" -lt 1 ]; }; then
  echo "ERROR: -t expects a positive integer, got '$cli_timeout'" >&2
  exit 1
fi

# —— 参数文件：与时钟修正/日志下载共用同一份机载凭据与编址 ——
if [ ! -f "$CONFIG" ]; then
  echo "ERROR: config file not found: $CONFIG" >&2
  echo "       cp $DEFAULT_CONFIG.example $DEFAULT_CONFIG && chmod 600 $DEFAULT_CONFIG" >&2
  exit 1
fi
# shellcheck source=/dev/null
. "$CONFIG"

# 覆盖顺序：默认值（上面已落地）→ 参数文件（刚 source）→ 环境变量 → 命令行（-t/-u）
[ -n "$env_user" ] && BOARD_SSH_USER="$env_user"
[ -n "$env_pass" ] && BOARD_SSH_PASS="$env_pass"
[ -n "$env_timeout" ] && SSH_TIMEOUT="$env_timeout"
[ -n "$env_ws" ] && ONBOARD_WS="$env_ws"
[ -n "$cli_user" ] && BOARD_SSH_USER="$cli_user"
[ -n "$cli_timeout" ] && SSH_TIMEOUT="$cli_timeout"

if [ -z "${BOARD_SSH_USER:-}" ]; then
  echo "ERROR: BOARD_SSH_USER not set in $CONFIG" >&2
  exit 1
fi
if ! [[ "$SSH_TIMEOUT" =~ ^[0-9]+$ ]] || [ "$SSH_TIMEOUT" -lt 1 ]; then
  echo "ERROR: SSH_TIMEOUT expects a positive integer, got '$SSH_TIMEOUT'（来源：-t / 环境变量 / $CONFIG）" >&2
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

# —— 远端脚本：整段经 base64 传输（密码/引号/`:=` 等特殊字符不经过 ssh 命令行）——
# 约定：远端只打印 "KEY=value" 事实行；多行详情用 START/END 包块（GCS 侧 block() 取）。
# 注意：匹配串 `bin/roslaunch ...` 只出现在被 base64 的脚本里，不出现在任何 shell 命令行里，
#   所以 pgrep 不会命中 ssh 包装 shell 自己（这是本工具唯一容易踩的自匹配坑）。
q() { printf '%q' "$1"; }        # 生成远端 shell 字面量
b64_of() { printf '%s' "$1" | base64 -w0; }

# 公共头：定位 ws / 机载脚本目录 / 转录文件，并回读机载 hostname
remote_prelude() {
  local ws
  if [ -n "$ONBOARD_WS" ]; then ws=$(q "$ONBOARD_WS"); else ws='$HOME/catkin_swarm6-2'; fi
  cat <<EOF
set -u
ws="$ws"
BASH_D="\$ws/src/swarm_uav_executor/bashs"
UAV=\$(hostname -s)
LOG="\$ws/.tmp/logs/\${UAV}_offboard_ego.log"
PAT='$LAUNCH_PAT'
echo "HOST=\$UAV"
EOF
}

# start：查脚本 → 记录起之前已有几个 → setsid 分离拉起 → 等 SETTLE_S 秒回查进程
remote_start_script() {
  local args_q="" a
  for a in "${ARGS[@]}"; do args_q+="$(q "$a") "; done
  cat <<EOF
$(remote_prelude)
for f in start_onboard.sh stop_onboard.sh; do
  [ -f "\$BASH_D/\$f" ] || { echo "MISSING=\$f"; echo "PRE_PROCS=-"; echo "STARTED=0"; exit 9; }
done
echo "MISSING=-"
pre=\$(pgrep -f "\$PAT" | wc -l); [ -n "\$pre" ] || pre=0
echo "PRE_PROCS=\$pre"
args=( $args_q )
errf="\$ws/.tmp/logs/onboard_ctl.err"       # 只接包装层 stderr（launch 输出走转录），每次新起先清空
mkdir -p "\$(dirname "\$errf")"; : > "\$errf"
# 分离拉起。**必须让 roslaunch 拿到默认 SIGINT 处置**，否则机载 stop_onboard.sh 的 SIGINT 无效，
#   只能等它 10 s 后升级 SIGTERM 硬杀（roslaunch 没机会收摊，节点可能变孤儿）。实测两个坑：
#   ① 非交互 shell 的 async 任务一律被塞 SIGINT=SIG_IGN，且**shell 无权重置自己继承来的 ignore**
#      （POSIX；bash 的 set -m 只在「本 shell 自己没被 ignore」时才恢复子进程默认处置）；
#   ② 程序可以重置（实测 python: getsignal=SIG_IGN → signal() 设 SIG_DFL 生效）。
#   所以用 python3 重置后 exec 机载脚本（python3 是 ROS 必备件；这里不依赖 set -m 的运气）。
if command -v python3 >/dev/null 2>&1; then
  setsid python3 -c 'import os,signal,sys; [signal.signal(s,signal.SIG_DFL) for s in (signal.SIGINT,signal.SIGQUIT)]; os.execv("/bin/bash",["bash"]+sys.argv[1:])' \
    "\$BASH_D/start_onboard.sh" \${args[@]+"\${args[@]}"} </dev/null >/dev/null 2>>"\$errf" &
else
  echo "# 机载没有 python3：退回 set -m + bash（只在调用方 SIGINT 未被忽略时有效）" >>"\$errf"
  set -m
  setsid bash "\$BASH_D/start_onboard.sh" \${args[@]+"\${args[@]}"} </dev/null >/dev/null 2>>"\$errf" &
  set +m
fi
sleep $SETTLE_S
n=\$(pgrep -f "\$PAT" | wc -l); [ -n "\$n" ] || n=0
echo "PROCS=\$n"
[ "\$n" -gt 0 ] && echo "STARTED=1" || echo "STARTED=0"
echo "LOG=\$LOG"
if [ -f "\$LOG" ]; then echo "LOG_TS=\$(date -r "\$LOG" '+%F %T')"; else echo "LOG_TS=-"; fi
echo "ERR_START"
tail -n 5 "\$errf" 2>/dev/null || true
echo "ERR_END"
EOF
}

# stop：直接调机载 stop_onboard.sh（它自己按 SIGINT→TERM→KILL 升级），再回查残留
remote_stop_script() {
  cat <<EOF
$(remote_prelude)
if [ ! -f "\$BASH_D/stop_onboard.sh" ]; then
  echo "MISSING=stop_onboard.sh"; echo "STOP_RC=9"; echo "LEFT=0"
  echo "DETAIL_START"; echo "机载没有 \$BASH_D/stop_onboard.sh（代码未同步）"; echo "DETAIL_END"
  exit 0
fi
echo "MISSING=-"
out=\$(bash "\$BASH_D/stop_onboard.sh" 2>&1); rc=\$?
n=\$(pgrep -f "\$PAT" | wc -l); [ -n "\$n" ] || n=0
echo "STOP_RC=\$rc"
echo "LEFT=\$n"
echo "DETAIL_START"
echo "\$out"
echo "DETAIL_END"
EOF
}

# status：只读（不需要机载脚本存在），报进程数 + 转录时间 + 本轮启动行
remote_status_script() {
  cat <<EOF
$(remote_prelude)
[ -f "\$BASH_D/start_onboard.sh" ] && echo "MISSING=-" || echo "MISSING=start_onboard.sh"
n=\$(pgrep -f "\$PAT" | wc -l); [ -n "\$n" ] || n=0
echo "PROCS=\$n"
if [ -f "\$LOG" ]; then
  echo "LOG_TS=\$(date -r "\$LOG" '+%F %T')"
  echo "LOG_BYTES=\$(stat -c %s "\$LOG")"
  echo "LOG_LAST=\$(grep -a '^===== ' "\$LOG" | tail -1)"
else
  echo "LOG_TS=-"; echo "LOG_BYTES=0"; echo "LOG_LAST=-"
fi
EOF
}

# logs：流式输出转录（不解析，原样透传；-f = tail -f）
remote_logs_script() {
  local follow=""; [ "$FOLLOW" -eq 1 ] && follow="-f"
  cat <<EOF
$(remote_prelude)
if [ ! -f "\$LOG" ]; then echo "# 机载没有转录文件 \$LOG（本机没起过机载栈？）"; exit 1; fi
echo "# host=\$UAV  log=\$LOG  bytes=\$(stat -c %s "\$LOG")  (\$BASH_D)"
tail -n $TAIL_N $follow "\$LOG"
EOF
}

# —— ssh 执行：一次连接跑完整段远端脚本，不做交互提示 ——
ssh_opts=(-o "StrictHostKeyChecking=no" -o "UserKnownHostsFile=/dev/null"
          -o "ConnectTimeout=$SSH_TIMEOUT" -o "NumberOfPasswordPrompts=1"
          -o "PreferredAuthentications=password,keyboard-interactive,publickey")

pass_ok() {
  [ -n "$BOARD_SSH_PASS" ] && [ "$BOARD_SSH_PASS" != CHANGE_ME ] \
    && command -v sshpass >/dev/null 2>&1
}

run_remote() {        # $1 ip  $2 base64 脚本  $3 ssh 用户 → 捕获 stdout+stderr
  local -a cmd=(ssh "${ssh_opts[@]}" "${3:-$BOARD_SSH_USER}@$1" "echo $2 | base64 -d | bash")
  if pass_ok; then SSHPASS="$BOARD_SSH_PASS" sshpass -e "${cmd[@]}" 2>&1; else "${cmd[@]}" 2>&1; fi
}

run_remote_stream() { # 同上，但不捕获（logs 用；Ctrl-C 直接断流）
  local -a tty=(); [ -t 0 ] && tty=(-t)
  local -a cmd=(ssh "${tty[@]}" "${ssh_opts[@]}" "${3:-$BOARD_SSH_USER}@$1" "echo $2 | base64 -d | bash")
  if pass_ok; then SSHPASS="$BOARD_SSH_PASS" sshpass -e "${cmd[@]}"; else "${cmd[@]}"; fi
}

val() { sed -n "s/^$1=//p" <<<"$2" | head -1; }
block() { sed -n "/^$2\$/,/^$3\$/p" <<<"$1" | sed '1d;$d'; }   # $1 输出 $2 起标记 $3 止标记
unreachable_err() { grep -v -e '^[A-Z_]*=' -e '^Warning: Permanently added' <<<"$1" | head -1 | tr -d '\r'; }

status=0      # 总退出码：0 全成，2 有 FAIL

# —— 表格头 ——
case "$CMD" in
  start)  printf '%-8s %-15s %-6s %-8s %-20s %s\n' TARGET IP PROCS STARTED TRANSCRIPT RESULT ;;
  stop)   printf '%-8s %-15s %-8s %-6s %s\n' TARGET IP HOST LEFT RESULT ;;
  status) printf '%-8s %-15s %-8s %-6s %-20s %-10s %s\n' TARGET IP HOST PROCS LOG_TS BYTES RESULT ;;
esac
[ "$CMD" = logs ] || printf '%s\n' \
  "--------------------------------------------------------------------------------------"

# 一台一趟 ssh：构造远端脚本 → （干跑则只打印）→ 执行 → 解析事实行 → 打印结果行
run_one() {   # $1 label  $2 ip  $3 ssh 用户
  local label="$1" ip="$2" tuser="${3:-$BOARD_SSH_USER}" script out host miss err det result rc

  case "$CMD" in
    start)  script=$(remote_start_script) ;;
    stop)   script=$(remote_stop_script) ;;
    status) script=$(remote_status_script) ;;
    logs)   script=$(remote_logs_script) ;;
  esac

  if [ "$DRY_RUN" -eq 1 ]; then
    case "$CMD" in
      start)  printf '%-8s DRY-RUN %s: setsid bash <ws>/src/swarm_uav_executor/bashs/start_onboard.sh %s（分离；%ss 后回查进程）\n' \
                     "$label" "$tuser@$ip" "${ARGS[*]-}" "$SETTLE_S" ;;
      stop)   printf '%-8s DRY-RUN %s: bash <ws>/src/swarm_uav_executor/bashs/stop_onboard.sh\n' \
                     "$label" "$tuser@$ip" ;;
      status) printf '%-8s DRY-RUN %s: pgrep -f "%s" + 读 <ws>/.tmp/logs/<host>_offboard_ego.log\n' \
                     "$label" "$tuser@$ip" "$LAUNCH_PAT" ;;
      logs)   printf '%-8s DRY-RUN %s: tail -n %s %s <ws>/.tmp/logs/<host>_offboard_ego.log\n' \
                     "$label" "$tuser@$ip" "$TAIL_N" "$([ "$FOLLOW" -eq 1 ] && echo -f)" ;;
    esac
    return 0
  fi

  if [ "$CMD" = logs ]; then
    echo "=== $label ($ip) ==="
    run_remote_stream "$ip" "$(b64_of "$script")" "$tuser"; rc=$?
    [ "$rc" -eq 0 ] || { echo "=== $label: logs 失败（ssh/tail rc=$rc）===" >&2; status=2; }
    return 0
  fi

  out=$(run_remote "$ip" "$(b64_of "$script")" "$tuser")
  host=$(val HOST "$out"); miss=$(val MISSING "$out")
  if [ -z "$host" ]; then
    printf '%-8s %-15s %s\n' "$label" "$ip" "FAIL(unreachable: $(unreachable_err "$out"))"
    status=2
    return 0
  fi

  case "$CMD" in
    start)
      result="PASS"
      if [ "$miss" != "-" ]; then result="FAIL(机载缺 $miss：代码未同步)"
      elif [ "$(val STARTED "$out")" != 1 ]; then result="FAIL(${SETTLE_S}s 后没有 roslaunch 进程)"
      elif [ "$(val PRE_PROCS "$out")" != 0 ]; then result="WARN(起之前已有 $(val PRE_PROCS "$out") 个 roslaunch：双开了？)"
      fi
      printf '%-8s %-15s %-6s %-8s %-20s %s\n' "$label" "$ip" "$(val PROCS "$out")" \
             "$(val STARTED "$out")" "$(val LOG_TS "$out")" "$result"
      if [ "$result" != PASS ]; then
        err=$(block "$out" ERR_START ERR_END); [ -z "$err" ] || sed 's/^/     | /' <<<"$err"
        echo "     看详情：$0 logs $label"
      fi
      [[ "$result" == FAIL* ]] && status=2 ;;
    stop)
      if [ "$miss" != "-" ]; then result="FAIL(机载缺 $miss：代码未同步)"
      elif [ "$(val STOP_RC "$out")" = 9 ]; then result="FAIL(机载缺 stop_onboard.sh)"
      elif [ "$(val LEFT "$out")" != 0 ]; then result="FAIL(残留 $(val LEFT "$out") 个 roslaunch，stop rc=$(val STOP_RC "$out"))"
      elif [ "$(val STOP_RC "$out")" != 0 ]; then result="FAIL(stop rc=$(val STOP_RC "$out"))"
      else result="PASS"
      fi
      printf '%-8s %-15s %-8s %-6s %s\n' "$label" "$ip" "$host" "$(val LEFT "$out")" "$result"
      if [ "$result" != PASS ]; then
        det=$(block "$out" DETAIL_START DETAIL_END); [ -z "$det" ] || sed 's/^/     | /' <<<"$det"
      fi
      [[ "$result" == FAIL* ]] && status=2 ;;
    status)
      if [ "$(val PROCS "$out")" -gt 0 ]; then result="UP($(val PROCS "$out"))"; else result="DOWN"; fi
      printf '%-8s %-15s %-8s %-6s %-20s %-10s %s\n' "$label" "$ip" "$host" "$(val PROCS "$out")" \
             "$(val LOG_TS "$out")" "$(val LOG_BYTES "$out")" "$result"
      [ "$(val LOG_LAST "$out")" = "-" ] || echo "     本轮启动：$(val LOG_LAST "$out")"
      [ "$miss" = "-" ] || echo "     注：机载缺 $miss → start 会 FAIL（先同步代码）" ;;
  esac
}

while read -r label ip tuser; do
  [ -n "${label:-}" ] || continue
  run_one "$label" "$ip" "${tuser:-$BOARD_SSH_USER}"
done <<<"$TARGET_LINES"

exit "$status"


