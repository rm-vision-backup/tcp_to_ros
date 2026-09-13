#!/usr/bin/env python3
"""List PX4 ULog entries on the FCU over MAVLink (mavlink-routerd export).

Usage:  ulog_list_fcu.py -t N [-n LAST]

  -t N      UAV number, or explicit "sys:comp":  -t 7 = UAV7 = 7:1   (required)
  -n LAST   print only the newest LAST entries; 0 = the whole table  (default 12)

The MAVLink source is fixed at udpin:0.0.0.0:15500 (every onboard mavlink-routerd
forwards its FCU link to that port on the ground station), so the autopilot must be
pinned with -t: mavlink-routerd emits its own HEARTBEAT (autopilot=8) on the same
port and auto-detect would latch onto it.

Exit: 0 ok; 2 no LOG_ENTRY received (link/target wrong, or the FCU cannot read the SD).
"""
import argparse
import sys
import time

from pymavlink import mavutil

# Fixed: onboard mavlink-routerd forwards the FCU link to this port (no env, no flag).
SRC = 'udpin:0.0.0.0:15500'
HB_TIMEOUT = 15.0     # s to wait for that autopilot's heartbeat
LIST_TIMEOUT = 6.0    # s of silence after the last LOG_ENTRY before giving up


def parse_target(text):
    """'7' -> (7, 1); '7:1' -> (7, 1). PX4 autopilot component id is 1."""
    parts = str(text).split(':')
    ts = int(parts[0])
    tc = int(parts[1]) if len(parts) > 1 else 1
    return ts, tc


def wait_heartbeat(m, ts, tc):
    hb = None
    t_end = time.time() + HB_TIMEOUT
    while time.time() < t_end:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb is not None and hb.get_srcSystem() == ts and hb.get_srcComponent() == tc:
            print('HEARTBEAT sys=%d comp=%d seen=True autopilot=%s'
                  % (ts, tc, hb.autopilot), flush=True)
            return True
    print('HEARTBEAT sys=%d comp=%d seen=False' % (ts, tc), flush=True)
    return False


def list_entries(m, ts, tc):
    entries = {}
    m.mav.log_request_list_send(ts, tc, 0, 0xffff)
    t_end = time.time() + LIST_TIMEOUT
    while time.time() < t_end:
        msg = m.recv_match(type='LOG_ENTRY', blocking=True, timeout=1)
        if msg is None:
            continue
        if (msg.get_srcSystem(), msg.get_srcComponent()) != (ts, tc):
            continue
        entries[int(msg.id)] = msg
        t_end = time.time() + LIST_TIMEOUT
    return entries


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-t', '--target', required=True,
                    help='UAV number (7 = UAV7) or explicit sys:comp (7:1)')
    ap.add_argument('-n', '--last', type=int, default=12,
                    help='print the newest N entries; 0 = all (default 12)')
    args = ap.parse_args()
    ts, tc = parse_target(args.target)

    m = mavutil.mavlink_connection(SRC)
    wait_heartbeat(m, ts, tc)
    entries = list_entries(m, ts, tc)
    if not entries:
        print('NO ENTRIES RECEIVED', flush=True)
        sys.exit(2)

    ids = sorted(entries)
    last_num = max(int(e.last_log_num) for e in entries.values())
    print('TOTAL %d logs, ids %d..%d, last_log_num=%d'
          % (len(ids), ids[0], ids[-1], last_num), flush=True)
    print('%-5s %10s %10s %14s %s'
          % ('id', 'size', 'num_logs', 'utc_epoch', 'utc+8'), flush=True)
    show = ids if args.last <= 0 else ids[-args.last:]
    for i in show:
        e = entries[i]
        stamp = time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(e.time_utc + 8 * 3600))
        print('%-5d %10d %10d %14d %s'
              % (i, e.size, e.num_logs, e.time_utc, stamp), flush=True)


if __name__ == '__main__':
    main()
