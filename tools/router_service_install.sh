#!/usr/bin/env bash
# router_service_install.sh —— 把机载 mavlink-routerd 装成 systemd 服务（逐台；从 GCS 跑）
#
# 背景（为什么要改）：桌面自启方案靠 gnome-terminal 标签页常开，脚本一退标签页关闭、
# systemd 的 vte-spawn-*.scope 被撤销、cgroup 把 router 一起杀掉——2026-09-22 起反复
# 出现"开机 router 没起来"，2026-09-23 定位后改成"前台阻塞"形态，但依赖图形登录这点
# 始终是隐患（无人登录 / 标签页被关 = 又没起来）。systemd 服务没有这些依赖。
# 详见 tools/systemd/mavlink-routerd.service 头部与 quality_reports/2026-09-23 报告 §1。
#
# 用法（在 GCS 上跑，逐台 ssh 过去；用户/密码/编址取 tools/configs/onboard.env）：
#   bash src/tcp_to_ros/tools/router_service_install.sh 8 10 13
#   bash src/tcp_to_ros/tools/router_service_install.sh 10          # 单台
#
# 逐台做的事（幂等）：
#   0) 前置检查：机载栈必须**没在跑**（router 重启会打断 mavros→飞控的定向通道，见 README §九.2）
#   1) 校验二进制存在
#   2) 写 /etc/systemd/system/mavlink-routerd.service
#   3) 收掉旧的"非 systemd"实例 → daemon-reload → enable --now
#   4) 关掉桌面自启项（含 Mavlink-route/Mavlink_Routerd 的 .desktop 置 X-GNOME-Autostart-enabled=false）
#   5) 复验：is-active / 进程在 / journal 里四个端点 Open 行
# 退出码：0 = 全部通过；1 = 有台失败（逐台打印原因）。
# =============================================================================
set -u

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
UNIT_FILE="$SCRIPT_DIR/systemd/mavlink-routerd.service"
CONFIG="$SCRIPT_DIR/configs/onboard.env"

[ -f "$UNIT_FILE" ] || { echo "ERROR: 找不到 $UNIT_FILE" >&2; exit 1; }
[ -f "$CONFIG" ] || { echo "ERROR: 找不到 $CONFIG" >&2; exit 1; }
# shellcheck source=/dev/null
. "$CONFIG"

: "${BOARD_SSH_USER:=nvidia}"
: "${BOARD_SUDO_PASS:=nvidia}"
: "${IP_PREFIX:=192.168.5}"
: "${IP_STRIDE:=10}"
: "${IP_OFFSET:=1}"

[ $# -ge 1 ] || { sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }

UNIT_B64=$(base64 -w0 "$UNIT_FILE")
rc_all=0

for t in "$@"; do
  num="${t#UAV}"; num="${num#uav}"
  [[ "$num" =~ ^[0-9]+$ ]] || { echo "== $t：不是 UAV 号，跳过"; rc_all=1; continue; }
  ip="${IP_PREFIX}.$((IP_STRIDE * num + IP_OFFSET))"
  echo "== UAV$num（$ip）=="

  remote=$(cat <<REMOTE
set -u
BIN=/home/nvidia/Mavlink_Routerd_Sh/mavlink-routerd
UNIT=/etc/systemd/system/mavlink-routerd.service
fail() { echo "FAIL: \$1"; exit 1; }
# 0) 机载栈在跑就停手（router 重启会打断 mavros→飞控定向通道）
if pgrep -f 'uav_offboard_ego_real.launch' >/dev/null; then
  fail "机载栈在跑：先 stop_onboard.sh / onboard_ctl.sh stop $num，再装 router 服务"
fi
[ -x "\$BIN" ] || fail "找不到可执行文件 \$BIN"
# 1) 写 unit（先落到 /tmp 再 sudo cp：sudo -S 会吃掉 stdin 的第一行，不能直接管道进 tee）
echo "$UNIT_B64" | base64 -d > /tmp/mavlink-routerd.service || fail "unit 解码失败"
echo "$BOARD_SUDO_PASS" | sudo -S cp /tmp/mavlink-routerd.service "\$UNIT" || fail "写 unit 失败"
rm -f /tmp/mavlink-routerd.service
# 2) 收掉旧的非 systemd 实例 → enable --now
echo "$BOARD_SUDO_PASS" | sudo -S pkill -x mavlink-routerd 2>/dev/null
sleep 1
echo "$BOARD_SUDO_PASS" | sudo -S systemctl daemon-reload || fail "daemon-reload 失败"
echo "$BOARD_SUDO_PASS" | sudo -S systemctl enable mavlink-routerd >/dev/null 2>&1
echo "$BOARD_SUDO_PASS" | sudo -S systemctl restart mavlink-routerd || fail "systemctl restart 失败"
# 3) 关桌面自启项（存在才改）
for f in \$HOME/.config/autostart/*.desktop; do
  [ -f "\$f" ] || continue
  grep -qE 'Mavlink-route|Mavlink_Routerd' "\$f" || continue
  if grep -q '^X-GNOME-Autostart-enabled' "\$f"; then
    sed -i 's/^X-GNOME-Autostart-enabled=.*/X-GNOME-Autostart-enabled=false/' "\$f"
  else
    printf 'X-GNOME-Autostart-enabled=false\n' >> "\$f"
  fi
  echo "  已禁用桌面自启：\$f"
done
# 4) 复验
sleep 3
act=\$(echo "$BOARD_SUDO_PASS" | sudo -S systemctl is-active mavlink-routerd 2>/dev/null)
[ "\$act" = "active" ] || { echo "$BOARD_SUDO_PASS" | sudo -S journalctl -u mavlink-routerd -n 20 --no-pager; fail "服务不是 active（\$act）"; }
pgrep -x mavlink-routerd >/dev/null || fail "进程不在"
echo "  systemctl is-active: active；进程在"
echo "$BOARD_SUDO_PASS" | sudo -S journalctl -u mavlink-routerd -n 15 --no-pager | grep -E 'Opened|UART|UDP Client|TCP Server' | sed 's/^/  /'
echo "OK"
REMOTE
)
  out=$(timeout 120 sshpass -p "${BOARD_SSH_PASS:-nvidia}" ssh -n \
        -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=10 \
        "${BOARD_SSH_USER}@${ip}" "echo $(printf '%s' "$remote" | base64 -w0) | base64 -d | bash" 2>&1)
  echo "$out" | sed 's/^/  /'
  echo "$out" | grep -q '^OK$' || { echo "  ⇒ UAV$num 未通过"; rc_all=1; }
done

[ "$rc_all" -eq 0 ] && echo "== 全部通过：router 现在由 systemd 管理（开机自起；判活 systemctl is-active mavlink-routerd）==" \
                    || echo "== 有台未通过，逐条看上面 FAIL ==" >&2
exit "$rc_all"
