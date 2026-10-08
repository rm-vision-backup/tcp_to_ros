#!/usr/bin/env bash
# 日志下载：一轮把一台（或多台）UAV 的日志收到本机（飞控 ulog + 机载 ROS / 终端日志）。
# 采集范围（本轮定稿的最小档）：① 飞控 PX4 ulog（MAVLink，固定 UDP :15500）+ ulog 全清单；
# ② 机载 .ros_home/log/<最新 session>/、runtime_logs/ego_planner/*、.tmp/logs/*_offboard_ego.log。
# 不收：nsh 快照、本机 GCS 侧日志、旧 launch 旁路日志（机上 ~/.tmp/*.log，如 px4_multi.log）。
#   范围与理由见 tools/README.md §2「本轮不收」；时间基陷阱见 §6 坑 5。
# Usage:
#   log_fetch.sh [选项] <目标> [目标 ...]
#   log_fetch.sh 7                                  # 收 UAV7
#   log_fetch.sh -n 3 1 2 6 14                      # 每台取最新 3 条 ulog
#   LOG_OUT=~/Downloads/flight_logs log_fetch.sh 1 2 6 14
# 目标写法：裸数字（7）= UAVn，同时定飞控 MAVLink（sys=n, comp=1）与本机推 IP 规则
#   （IP_PREFIX.IP_STRIDE*n+IP_OFFSET）；也接受 UAV7、uav7、区间 1-6、host、user@host。
# Options:
#   -n N      每台取最新 N 条 ulog（默认 1；0 = 只列清单不下载）
#   -c FILE   参数文件（默认 <tools>/configs/onboard.env：机载账号/密码 + IP 规则）
#   -t SEC    ssh 连接超时秒数（覆盖参数文件）
#   -u USER   ssh 登录用户（覆盖参数文件）
#   -s        跳过机载阶段（只收飞控 ulog）
#   -h        本帮助
# 输出：$LOG_OUT/<UAVn_YYYY-MM-DD_HHMM>/（LOG_OUT 默认当前目录），内含 fcu/、onboard/{ros_home,
#   ego_planner,term}/，并生成 manifest.txt（逐项状态/来源/字节/md5）、README.md、md5sums.txt。
# 退出码：0 全部成功；1 参数/配置错；2 至少一台有失败项

set -uo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
TOOLS_DIR="$SCRIPT_DIR"           # .py 与 configs/onboard.env 都在 tools/ 下（曾写成 "$SCRIPT_DIR/.." → 路径全错，见 README §6 坑 8）
DEFAULT_CONFIG="$TOOLS_DIR/configs/onboard.env"
LOG_OUT="${LOG_OUT:-$PWD}"
CAP_BYTES=$((32 * 1024 * 1024))   # 大文件（planner / 终端转录）上限：超了取尾部（硬编码，不做开关）

ULOG_N=1
DO_SSH=1
CONFIG="$DEFAULT_CONFIG"
SSH_TIMEOUT=8
BOARD_SSH_USER="${USER:-ubuntu}"
BOARD_SSH_PASS=""
IP_PREFIX="192.168.5"
IP_STRIDE=10
IP_OFFSET=1

usage() { sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'; }

TARGETS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -n) ULOG_N="${2:-}" ; shift ;;
    -c) CONFIG="${2:-}" ; shift ;;
    -t) SSH_TIMEOUT="${2:-}" ; shift ;;
    -u) BOARD_SSH_USER="${2:-}" ; shift ;;
    -s) DO_SSH=0 ;;
    -h|--help) usage; exit 0 ;;
    -*) echo "unknown option: $1" >&2; usage >&2; exit 1 ;;
    *) TARGETS+=("$1") ;;
  esac
  shift
done

if [ "${#TARGETS[@]}" -eq 0 ]; then
  echo "ERROR: no target given (e.g. 7 or 1 2 6 14)" >&2
  usage >&2
  exit 1
fi
if ! [[ "$ULOG_N" =~ ^[0-9]+$ ]]; then
  echo "ERROR: -n expects a non-negative integer, got '$ULOG_N'" >&2
  exit 1
fi
if ! [[ "$SSH_TIMEOUT" =~ ^[0-9]+$ ]] || [ "$SSH_TIMEOUT" -lt 1 ]; then
  echo "ERROR: -t expects a positive integer, got '$SSH_TIMEOUT'" >&2
  exit 1
fi

# —— 参数文件：与时钟修正共用同一份机载凭据/编址 ——
if [ ! -f "$CONFIG" ]; then
  echo "ERROR: config file not found: $CONFIG" >&2
  echo "       cp $DEFAULT_CONFIG.example $DEFAULT_CONFIG && chmod 600 $DEFAULT_CONFIG" >&2
  exit 1
fi
# shellcheck source=/dev/null
. "$CONFIG"
[ -n "$BOARD_SSH_USER" ] || { echo "ERROR: BOARD_SSH_USER not set in $CONFIG" >&2; exit 1; }

# —— 目标展开：输出 "label ip ssh_user mav_target"（'-' = 该项不适用）——
uav_line() {
  local n="$1"
  printf 'UAV%s %s.%s - %s:1\n' "$n" "$IP_PREFIX" "$((IP_STRIDE * n + IP_OFFSET))" "$n"
}

expand_targets() {
  local t a b n num user host
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
        for ((n = a; n <= b; n++)); do uav_line "$n"; done ;;
      [0-9]*) uav_line "$t" ;;
      [Uu][Aa][Vv][0-9]*)
        num="${t:3}"
        if ! [[ "$num" =~ ^[0-9]+$ ]]; then echo "ERROR: bad UAV id: $t" >&2; return 1; fi
        uav_line "$num" ;;
      *@*@*) echo "ERROR: bad target: $t" >&2; return 1 ;;
      *@*) user="${t%%@*}"; host="${t#*@}"; printf '%s %s %s -\n' "$host" "$host" "$user" ;;
      *) printf '%s %s - -\n' "$t" "$t" ;;
    esac
  done
}

TARGET_LINES=$(expand_targets "${TARGETS[@]}") || exit 1

# —— 远端取件脚本：一趟 ssh 把机载日志流回来（不落机上临时文件）——
# 协议： "### key=value" 事实行； "@@@ <relpath> <源字节|-> <truncated|->" 后跟一行 base64 负载；
#        "!!! <说明>" = 机上没有该项。cap（第 1 个参数）为大文件上限字节。
remote_script() {
  cat <<'REMOTE'
set -u
ws=$HOME/catkin_swarm6-2
cap=${1:-0}
hn=$(cat /etc/hostname 2>/dev/null || hostname)
printf '### host=%s\n' "$hn"
printf '### epoch=%s\n' "$(date +%s)"
printf '### date=%s\n' "$(date '+%F %T %Z')"

emit_file() {   # $1 = 归档内相对路径；$2 = 机上绝对路径
  if [ ! -f "$2" ]; then printf '!!! %s missing\n' "$1"; return 0; fi
  sz=$(wc -c <"$2" 2>/dev/null || echo 0)
  if [ "$cap" -gt 0 ] && [ "$sz" -gt "$cap" ]; then
    printf '@@@ %s %s truncated\n' "$1" "$sz"
    tail -c "$cap" "$2" | base64 -w0
  else
    printf '@@@ %s %s -\n' "$1" "$sz"
    base64 -w0 <"$2"
  fi
  echo
}

# ① ROS 会话日志（最新 session 整份；latest 是符号链接，优先跟它）
sess=$(readlink -f "$ws/.ros_home/log/latest" 2>/dev/null || true)
if [ ! -d "$sess" ]; then
  sess=$(ls -1dt "$ws/.ros_home/log"/*/ 2>/dev/null | grep -v '/latest/$' | head -1 || true)
fi
sess=${sess%/}
if [ -n "$sess" ] && [ -d "$sess" ]; then
  name=${sess##*/}
  printf '@@@ onboard/ros_home/%s.tar.gz - -\n' "$name"
  tar czf - -C "$ws/.ros_home/log" "$name" | base64 -w0
  echo
else
  printf '!!! onboard/ros_home missing\n'
fi

# ② planner 诊断（优先 <hostname>-ego-planner.log，否则该目录最新一份）
plog="$ws/runtime_logs/ego_planner/$hn-ego-planner.log"
if [ ! -f "$plog" ]; then
  plog=$(ls -1t "$ws/runtime_logs/ego_planner"/*.log 2>/dev/null | head -1 || true)
fi
if [ -n "$plog" ] && [ -f "$plog" ]; then
  emit_file "onboard/ego_planner/${plog##*/}" "$plog"
else
  printf '!!! onboard/ego_planner missing\n'
fi

# ③ 终端全量转录（可能多份：漏设 UAV 时文件名是 _offboard_ego.log）
found=0
for f in "$ws/.tmp/logs/"*_offboard_ego.log; do
  [ -f "$f" ] || continue
  found=1
  emit_file "onboard/term/${f##*/}" "$f"
done
[ "$found" -eq 0 ] && printf '!!! onboard/term missing\n'

# ④ 观测/事件日志（2026-10-08 增）：bridge 数据面健康（per-topic 到达间隔/停顿/连接事件）
#    与执行器侧事件（跟随环首帧/超时、relay 选源与 HOLD）。固定文件名、追加写，
#    超 cap 时 emit_file 只留尾部——正是要的近期数据（首行可能是半行）。
found=0
# 宽匹配 *_events.jsonl：组件名（follower / relay / …）随实现变，采集不该与文件名耦合。
# 2026-10-08 四机 SITL 彩排实测：跟随环写的是 <UAV>_follower_events.jsonl，早前写的
# *_executor_events.jsonl 会漏收。
for f in "$ws/runtime_logs/health/"*_bridge_health.jsonl \
         "$ws/runtime_logs/health/"*_events.jsonl; do
  [ -f "$f" ] || continue
  found=1
  emit_file "onboard/health/${f##*/}" "$f"
done
[ "$found" -eq 0 ] && printf '!!! onboard/health missing\n'
printf '### done\n'
REMOTE
}

run_remote() {   # $1 = ip；$2 = base64 后的远端脚本；$3 = ssh 用户
  local -a cmd=(ssh -o "StrictHostKeyChecking=no" -o "UserKnownHostsFile=/dev/null"
                -o "ConnectTimeout=$SSH_TIMEOUT" -o "NumberOfPasswordPrompts=1"
                -o "PreferredAuthentications=password,keyboard-interactive,publickey"
                "${3}@${1}" "echo $2 | base64 -d | bash -s -- $CAP_BYTES")
  if [ -n "$BOARD_SSH_PASS" ] && [ "$BOARD_SSH_PASS" != CHANGE_ME ] \
     && command -v sshpass >/dev/null 2>&1; then
    SSHPASS="$BOARD_SSH_PASS" sshpass -e "${cmd[@]}"
  else
    "${cmd[@]}"
  fi
}

finish_target() {   # $1 = out dir；$2 = label；$3 = ip；$4 = mav；$5 = ssh 用户
  local out="$1" label="$2" ip="$3" mav="$4" tuser="$5"
  local md5 rel src host epoch tdate facts nok nfail nissue
  ( cd "$out" && find . -type f ! -name md5sums.txt ! -name manifest.txt \
      ! -name README.md ! -name '.facts' ! -name '.notes' -print0 \
      | LC_ALL=C sort -z | xargs -0 -r md5sum ) >"$out/md5sums.txt"

  {
    printf '# status|source|relpath|bytes|md5|note\n'
    facts="$out/.facts"
    if [ -f "$facts" ]; then sed 's/^/# /' "$facts"; fi
    printf '# target=%s ip=%s mav=%s ssh_user=%s stamp=%s\n' \
           "$label" "$ip" "$mav" "${tuser:--}" "$STAMP"
    [ -f "$out/.notes" ] && cat "$out/.notes"
    while read -r md5 rel; do
      rel="${rel#./}"
      case "$rel" in fcu/*) src=mavlink ;; *) src=ssh ;; esac
      printf 'OK|%s|%s|%s|%s|-\n' "$src" "$rel" "$(wc -c <"$out/$rel")" "$md5"
    done <"$out/md5sums.txt"
  } >"$out/manifest.txt"

  host='-'; epoch='-'; tdate='-'
  if [ -f "$facts" ]; then
    host=$(sed -n 's/^host=//p' "$facts" | head -1)
    epoch=$(sed -n 's/^epoch=//p' "$facts" | head -1)
    tdate=$(sed -n 's/^date=//p' "$facts" | head -1)
  fi
  nok=$(grep -c '^OK|' "$out/manifest.txt" || true)
  nfail=$(grep -c '^FAIL|' "$out/manifest.txt" || true)
  nissue=$(grep -c -E '^(MISSING|TRUNCATED|SIZE-MISMATCH)\|' "$out/manifest.txt" || true)

  emit_readme "$label" "$ip" "$mav" "$tuser" "$host" "$epoch" "$tdate" \
              "$nok" "$nfail" "$nissue" >"$out/README.md"
  rm -f "$out/.facts" "$out/.notes"

  if [ "$nfail" -gt 0 ]; then TARGET_FAILED=1; fi
  printf '    -> %s：落地 %s 项，FAIL %s 项，缺/截断 %s 项（%s）\n' \
         "$out" "$nok" "$nfail" "$nissue" "$out/README.md"
}

emit_readme() {   # $1 label $2 ip $3 mav $4 ssh_user $5 host $6 epoch $7 date
                  # $8 ok $9 fail $10 missing
  local label="$1" ip="$2" mav="$3" tuser="$4" host="$5" epoch="$6" tdate="$7"
  local nok="$8" nfail="$9" nissue="${10}" warn=""
  if [[ "$epoch" =~ ^[0-9]+$ ]] && [ "$epoch" -lt 1600000000 ]; then
    warn="
⚠ **机载板钟未修正**（epoch=$epoch）→ $label 上的 ROS 日志时间戳不可用；
   请以 \`fcu/ulog_list.txt\` 的 GPS UTC（utc+8 列）为准（先跑 tools/onboard_clock_fix.sh）。"
  fi
  cat <<EOF
# 日志归档 ${label}_${STAMP}

- 采集时刻：$(date '+%F %T %Z')（本机时钟；多目标共用同一 stamp）
- 目标：$label / $ip / 飞控 MAVLink ${mav}
- 机载：host=$host，板钟 epoch=$epoch（$tdate）
- 命令：$CMDLINE
- 结果：落地 $nok 项，FAIL $nfail 项，缺失/截断 $nissue 项$warn

## 内容

| 路径 | 来源 | 说明 |
|---|---|---|
| \`fcu/ulog_list.txt\` | MAVLink | PX4 \`LOG_ENTRY\` 全清单（含 GPS UTC 真时刻） |
| \`fcu/log_*.ulg\` | MAVLink | PX4 ulog（\`-n N\` 指定的最新 N 条；断点续传） |
| \`fcu/ulog_fetch.log\` | MAVLink | 下载过程日志（进度 / stall 重试痕迹） |
| \`onboard/ros_home/<session>/\` | ssh | ROS 会话日志（rosout、各节点、master、roslaunch） |
| \`onboard/ego_planner/*.log\` | ssh | planner 诊断（机上超 32 MB 时只留尾部，见 manifest） |
| \`onboard/term/*_offboard_ego.log\` | ssh | 终端全量转录（way B），漏设 \`UAV\` 时文件名为 \`_offboard_ego.log\` |
| \`onboard/health/*.jsonl\` | ssh | 数据面观测：bridge per-topic 到达间隔/停顿/连接事件、执行器跟随环与 relay 事件（超 32 MB 只留尾部，首行可能半行） |
| \`onboard/ssh.err\` | ssh | 仅在有 stderr 输出时存在 |
| \`manifest.txt\` | — | 逐项状态/来源/字节/md5（\`MISSING\`/\`FAIL\` 都在这里） |
| \`md5sums.txt\` | — | \`md5sum -c\` 校验用 |

## 复现

\`\`\`bash
cd "\$(dirname "\$0")" 2>/dev/null || cd .
md5sum -c md5sums.txt
ulog_info fcu/log_*.ulg          # duration / No Dropouts
less manifest.txt                # 缺什么、为什么缺
\`\`\`
EOF
}

parse_stream() {   # $1 = 抓到的流文件；$2 = 目标目录；解 base64 负载并落地
  local stream="$1" out="$2" line rel sz flag blob base real
  exec 3<"$stream"
  while IFS= read -r -u3 line; do
    case "$line" in
      '### '*)
        [ "${line#'### '}" = "done" ] && continue
        printf '%s\n' "${line#'### '}" >>"$out/.facts" ;;
      '!!! '*)
        # shellcheck disable=SC2086
        set -- ${line#'!!! '}
        printf 'MISSING|ssh|%s|-|-|%s\n' "$1" "${*:2}" >>"$out/.notes" ;;
      '@@@ '*)
        # shellcheck disable=SC2086
        set -- $line
        rel="$2"; sz="$3"; flag="$4"
        if ! IFS= read -r -u3 blob; then
          printf 'FAIL|ssh|%s|-|-|stream ended before payload\n' "$rel" >>"$out/.notes"
          continue
        fi
        base="$out/${rel%/*}"
        mkdir -p "$base"
        if ! printf '%s' "$blob" | base64 -d >"$base/${rel##*/}" 2>/dev/null; then
          printf 'FAIL|ssh|%s|-|-|base64 decode failed\n' "$rel" >>"$out/.notes"
          rm -f "$base/${rel##*/}"
          continue
        fi
        case "$rel" in
          *.tar.gz)
            if tar xzf "$base/${rel##*/}" -C "$base"; then rm -f "$base/${rel##*/}"
            else printf 'FAIL|ssh|%s|-|-|tar -xzf failed\n' "$rel" >>"$out/.notes"; fi ;;
          *)
            real=$(wc -c <"$base/${rel##*/}")
            if [ "$flag" = truncated ]; then
              printf 'TRUNCATED|ssh|%s|%s|-|board had %s bytes, kept the tail\n' \
                     "$rel" "$real" "$sz" >>"$out/.notes"
            elif [ "$sz" != "-" ] && [ "$real" != "$sz" ]; then
              printf 'SIZE-MISMATCH|ssh|%s|%s|-|board said %s bytes\n' \
                     "$rel" "$real" "$sz" >>"$out/.notes"
            fi ;;
        esac ;;
    esac
  done
  exec 3<&-
}

# —— 主流程：多目标串行（:15500 是所有机载 routerd 共用的一个端口，ulog 不能并行取）——
STAMP=$(date +%Y-%m-%d_%H%M)
CMDLINE="LOG_OUT=$LOG_OUT $(basename "$0") -n $ULOG_N$([ "$DO_SSH" -eq 0 ] && echo ' -s') ${TARGETS[*]}"
TARGET_FAILED=0
status=0

while read -r label ip tuser mavg; do
  [ -n "${label:-}" ] || continue
  out="$LOG_OUT/${label}_${STAMP}"
  mkdir -p "$out/fcu" "$out/onboard/ros_home" "$out/onboard/ego_planner" "$out/onboard/term"
  : >"$out/.facts"; : >"$out/.notes"
  [ "$tuser" = "-" ] && tuser=""     # '-' 占位符 = 用参数文件里的 BOARD_SSH_USER
  printf '== %s（%s，MAVLink %s）-> %s\n' "$label" "$ip" "$mavg" "$out"

  # ① 机载：一趟 ssh 把 ROS 会话日志 / planner 诊断 / 终端转录流回来
  if [ "$DO_SSH" -eq 1 ]; then
    if [ "$ip" = "-" ]; then
      printf 'FAIL|ssh|-|-|-|no host for target %s\n' "$label" >>"$out/.notes"
    else
      user="${tuser:-$BOARD_SSH_USER}"
      printf -- '-- 机载：%s@%s\n' "$user" "$ip"
      blob=$(remote_script | base64 -w0)
      tmp=$(mktemp); errf="$out/onboard/ssh.err"
      if run_remote "$ip" "$blob" "$user" >"$tmp" 2>"$errf"; then
        parse_stream "$tmp" "$out"
      else
        rc=$?
        printf 'FAIL|ssh|-|-|-|ssh rc=%d: %s\n' "$rc" \
               "$(head -1 "$errf" | tr -d '\r')" >>"$out/.notes"
      fi
      rm -f "$tmp"
      [ -s "$errf" ] || rm -f "$errf"
    fi
  fi

  # ② 飞控：ulog 全清单（F2）+ 最新 N 条 ulog（F1）
  if [ "$mavg" = "-" ]; then
    printf 'MISSING|mavlink|fcu|-|-|target %s has no UAV number (no FCU sysid)\n' "$label" \
           >>"$out/.notes"
  else
    printf -- '-- 飞控：列 ulog 清单（MAVLink %s）\n' "$mavg"
    lrc=0
    python3 "$TOOLS_DIR/fcu_ulog_list.py" -t "$mavg" -n 0 >"$out/fcu/ulog_list.txt" 2>&1 || lrc=$?
    if [ "$lrc" -ne 0 ]; then
      printf 'FAIL|mavlink|fcu/ulog_list.txt|-|-|rc=%d: no LOG_ENTRY (link/target wrong, or FCU cannot read SD)\n' \
             "$lrc" >>"$out/.notes"
    fi
    ids=()
    if [ "$lrc" -eq 0 ] && [ "$ULOG_N" -gt 0 ]; then
      mapfile -t ids < <(awk '/^[0-9]+[[:space:]]/{print $1}' "$out/fcu/ulog_list.txt" \
                         | tail -n "$ULOG_N")
    fi
    if [ "$lrc" -eq 0 ] && [ "$ULOG_N" -gt 0 ] && [ "${#ids[@]}" -eq 0 ]; then
      printf 'FAIL|mavlink|fcu/log_*.ulg|-|-|no id found in ulog_list.txt\n' >>"$out/.notes"
    fi
    if [ "${#ids[@]}" -gt 0 ]; then
      printf -- '-- 飞控：下 ulog %s\n' "${ids[*]}"
      frc=0
      LOG_OUT="$out/fcu" python3 "$TOOLS_DIR/fcu_ulog_fetch.py" -t "$mavg" "${ids[@]}" \
        >"$out/fcu/ulog_fetch.log" 2>&1 || frc=$?
      if [ "$frc" -ne 0 ]; then
        printf 'FAIL|mavlink|fcu/log_*.ulg|-|-|fcu_ulog_fetch.py rc=%d: see fcu/ulog_fetch.log\n' \
               "$frc" >>"$out/.notes"
      fi
      for i in "${ids[@]}"; do
        [ -s "$out/fcu/log_${i}.ulg" ] || \
          printf 'FAIL|mavlink|fcu/log_%s.ulg|-|-|not fetched: see fcu/ulog_fetch.log\n' "$i" \
                 >>"$out/.notes"
      done
    fi
  fi

  finish_target "$out" "$label" "$ip" "$mavg" "$tuser"
done <<<"$TARGET_LINES"

printf '\n== 完成：%s 个目标；归档根 %s\n' "$(wc -l <<<"$TARGET_LINES")" "$LOG_OUT"
[ "$TARGET_FAILED" -eq 1 ] && status=2
exit "$status"

