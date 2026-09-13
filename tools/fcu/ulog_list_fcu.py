#!/usr/bin/env python3
"""List PX4 ULog entries on the FCU over MAVLink (mavlink-router export)."""
import os
import sys
import time

from pymavlink import mavutil

SRC = os.environ.get('MAV_SRC', 'udpin:0.0.0.0:15500')
LAST = int(os.environ.get('LAST', '12'))
# Pin the autopilot explicitly (e.g. MAV_TARGET=7:1) because mavlink-routerd also
# emits its own heartbeat (autopilot=8), which wait_heartbeat would otherwise grab.
TARGET = os.environ.get('MAV_TARGET', '')

m = mavutil.mavlink_connection(SRC)
if TARGET:
    ts, tc = (int(x) for x in TARGET.split(':'))
    seen = False
    t_end = time.time() + 15.0
    while time.time() < t_end and not seen:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb is not None and hb.get_srcSystem() == ts and hb.get_srcComponent() == tc:
            seen = True
    print('HEARTBEAT sys=%d comp=%d seen=%s autopilot=%s'
          % (ts, tc, seen, hb.autopilot if seen else 'n/a'), flush=True)
else:
    hb = m.wait_heartbeat(timeout=15)
    ts, tc = m.target_system, m.target_component
    print('HEARTBEAT sys=%d comp=%d type=%s' % (ts, tc, hb.type if hb else 'none'),
          flush=True)

entries = {}
m.mav.log_request_list_send(ts, tc, 0, 0xffff)
t_win = time.time()
while time.time() - t_win < 6.0:
    msg = m.recv_match(type='LOG_ENTRY', blocking=True, timeout=1)
    if msg is None:
        continue
    if TARGET and (msg.get_srcSystem(), msg.get_srcComponent()) != (ts, tc):
        continue
    entries[int(msg.id)] = msg
    t_win = time.time()
if not entries:
    print('NO ENTRIES RECEIVED', flush=True)
    sys.exit(2)

ids = sorted(entries)
last_num = max(int(e.last_log_num) for e in entries.values())
print('TOTAL %d logs, ids %d..%d, last_log_num=%d'
      % (len(ids), ids[0], ids[-1], last_num), flush=True)
print('%-5s %10s %10s %14s %s' % ('id', 'size', 'num_logs', 'utc_epoch', 'utc+8'), flush=True)
for i in ids[-LAST:]:
    e = entries[i]
    stamp = time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(e.time_utc + 8 * 3600))
    print('%-5d %10d %10d %14d %s' % (i, e.size, e.num_logs, e.time_utc, stamp), flush=True)
