#!/usr/bin/env bash
# Sync the onboard companion-computer clock of UAVn from this host.
#
# 背景：机载电脑无 RTC/NTP，上电或重启后板钟回到 1970-01-01
#（`timedatectl` 显示 `System clock synchronized: no`），本轮 ROS 日志时间戳随之不可用。
# 对时**必须在启动 ROS 栈之前**完成：
#   real_setup.md §3.2 第 1 步（部署时）、§5.1.1 第 0 步（每次启动前核板钟）。
#
# Usage:
#   board_time_sync.sh [-n] [-u USER] [-t SEC] UAV7 [UAV8 ...]
#   board_time_sync.sh --help
#
# Options:
#   -n            dry run：只打印将执行的远端命令，不改任何时钟
#   -u USER       ssh 登录用户（默认 $USER）
#   -t SEC        ssh 连接超时秒数（默认 8）
#
# Env:
#   BOARD_SSH_PASS   ssh 密码；不设则用密钥/agent（本脚本不做交互提示）
#   BOARD_SUDO_PASS  远端 sudo 密码；不设则要求远端免密 sudo（`sudo -n`）
#
# 目标写法：UAVn（解析为 192.168.5.(10n+1)）、host、user@host
# 退出码：0 全部对齐；1 本机时钟异常/参数错误；2 至少一台失败

set -uo pipefail

SSH_USER="${USER:-ubuntu}"
SSH_TIMEOUT=8
DRY_RUN=0
TOLERANCE_S=2

usage() {
  sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'
}

TARGETS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -n) DRY_RUN=1 ;;
    -u) SSH_USER="${2:-}"; shift ;;
    -t) SSH_TIMEOUT="${2:-}"; shift ;;
    -h|--help) usage; exit 0 ;;
    -*) echo "unknown option: $1" >&2; usage >&2; exit 1 ;;
    *) TARGETS+=("$1") ;;
  esac
  shift
done

if [ "${#TARGETS[@]}" -eq 0 ]; then
  echo "ERROR: no target given (e.g. UAV7)" >&2
  usage >&2
  exit 1
fi

# 本机时钟自检：把机载对成错的比不对更糟
NOW=$(date +%s)
if [ "$NOW" -lt 1600000000 ]; then
  echo "ERROR: local clock looks wrong (epoch=$NOW, $(date '+%F %T %Z'))." >&2
  echo "       Fix this host clock first (e.g. sudo date -s '@<real epoch>')." >&2
  exit 1
fi

resolve_host() {
  # UAVn -> 192.168.5.(10n+1)（real_setup §1）；其余原样透传
  local t="$1" user_part name num
  name="${t##*@}"
  if [[ "$t" == *@* ]]; then user_part="${t%@*}@"; else user_part="${SSH_USER}@"; fi
  if [[ "$name" =~ ^UAV([0-9]{1,2})$ ]]; then
    num="${BASH_REMATCH[1]}"
    if [ "$num" -lt 1 ] || [ "$num" -gt 16 ]; then
      echo "ERROR: UAV number out of range 1..16: $num" >&2
      return 1
    fi
    echo "${user_part}192.168.5.$((10 * num + 1))"
  else
    echo "${user_part}${name}"
  fi
}

ssh_run() {
  # ssh_run <userhost> <remote-script>
  local uh="$1" script="$2"
  if [ -n "${BOARD_SSH_PASS:-}" ]; then
    sshpass -p "$BOARD_SSH_PASS" ssh -o ConnectTimeout="$SSH_TIMEOUT" \
      -o StrictHostKeyChecking=accept-new "$uh" "$script"
  else
    ssh -o ConnectTimeout="$SSH_TIMEOUT" -o BatchMode=yes \
      -o StrictHostKeyChecking=accept-new "$uh" "$script"
  fi
}

remote_script() {
  # 远端：回读 before → 设时 → 回读 after，全部以 key=value 回传
  local epoch="$1"
  if [ -n "${BOARD_SUDO_PASS:-}" ]; then
    cat <<EOF
before=\$(date +%s)
printf '%s\n' '${BOARD_SUDO_PASS}' | sudo -S date -s '@${epoch}' >/dev/null 2>&1; rc=\$?
after=\$(date +%s)
echo "rc=\$rc before=\$before after=\$after wall=\$(date '+%F %T') tz=\$(date +%Z)"
EOF
  else
    cat <<EOF
before=\$(date +%s)
sudo -n date -s '@${epoch}' >/dev/null 2>&1; rc=\$?
after=\$(date +%s)
echo "rc=\$rc before=\$before after=\$after wall=\$(date '+%F %T') tz=\$(date +%Z)"
EOF
  fi
}

fails=0
printf '%-28s %-12s %-21s %-21s %7s  %s\n' \
  host before_epoch before_wall after_wall delta_s status
for target in "${TARGETS[@]}"; do
  if ! uh=$(resolve_host "$target"); then
    fails=$((fails + 1))
    continue
  fi
  case "$uh" in *@*) ;; *) uh="${SSH_USER}@${uh}" ;; esac
  script=$(remote_script "$NOW")

  if [ "$DRY_RUN" -eq 1 ]; then
    printf '%-28s %s\n' "$uh" "DRY-RUN: ssh $uh '<set date -s @$NOW + verify>'"
    continue
  fi

  t0=$(date +%s)
  out=$(ssh_run "$uh" "$script" 2>&1)
  rc=$?
  if [ "$rc" -ne 0 ] || ! grep -q 'rc=' <<<"$out"; then
    printf '%-28s %-12s %-21s %-21s %7s  %s\n' \
      "$uh" '-' '-' '-' '-' "FAIL(ssh rc=$rc: $(tr -d '\n' <<<"$out" | cut -c1-60))"
    fails=$((fails + 1))
    continue
  fi

  rc_remote=$(sed -n 's/.*rc=\([0-9-]*\).*/\1/p' <<<"$out")
  before=$(sed -n 's/.*before=\([0-9]*\).*/\1/p' <<<"$out")
  after=$(sed -n 's/.*after=\([0-9]*\).*/\1/p' <<<"$out")
  wall=$(sed -n 's/.*wall=\([^ ]* [^ ]*\).*/\1/p' <<<"$out")
  delta=$((after - t0))
  adelta=${delta#-}
  status=OK
  if [ "$rc_remote" -ne 0 ]; then
    status="FAIL(set rc=$rc_remote: $( [ -n "${BOARD_SUDO_PASS:-}" ] && echo 'sudo 密码错?' || echo '需要 BOARD_SUDO_PASS')"
    fails=$((fails + 1))
  elif [ "$adelta" -gt "$TOLERANCE_S" ]; then
    status="FAIL(delta ${delta}s > ${TOLERANCE_S}s)"
    fails=$((fails + 1))
  fi
  printf '%-28s %-12s %-21s %-21s %7s  %s\n' \
    "$uh" "$before" "$(date -d "@$before" '+%F %T' 2>/dev/null || echo '-')" "$wall" "$delta" "$status"
done

if [ "$DRY_RUN" -eq 1 ]; then
  echo "DRY-RUN: nothing changed."
  exit 0
fi

echo
echo "NOTE: 只对之后启动的进程生效；启动 ROS 栈前应先执行本脚本（real_setup §5.1.1 第 0 步）。"
echo "NOTE: 机载无 NTP（实测 System clock synchronized: no）；若后续启用了 NTP，可能被回拨覆盖。"
[ "$fails" -eq 0 ] || echo "RESULT: $fails target(s) FAILED" >&2
exit $(( fails == 0 ? 0 : 2 ))
