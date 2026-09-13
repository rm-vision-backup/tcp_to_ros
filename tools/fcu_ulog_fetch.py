#!/usr/bin/env python3
"""Fetch PX4 ULogs from the FCU over MAVLink into $LOG_OUT.

Usage:  LOG_OUT=<dir> fcu_ulog_fetch.py -t N [-n LAST] [log_id ...]

  -t N       UAV number, or explicit "sys:comp":  -t 7 = UAV7 = 7:1   (required)
  -n LAST    with no log_id: take the newest LAST entries            (default 2)
  log_id ... explicit ids; a file already complete is skipped, a partial file is
             resumed in place (LOG_REQUEST_DATA from its current byte offset)

$LOG_OUT is the only environment variable (default: current directory); the MAVLink
source (:15500) and the chunk timeout / stall limit are fixed constants below.

Exit: 0 all done; 2 no LOG_ENTRY received (link/target wrong, or the FCU cannot
      read the SD — retry/power-cycle the FCU before blaming the card).
"""
import argparse
import os
import sys
import time

from pymavlink import mavutil

# --- fixed link/chunking constants (deliberately not configurable) ---
SRC = 'udpin:0.0.0.0:15500'   # onboard mavlink-routerd forwards the FCU link here
CHUNK_TIMEOUT = 1.0           # s of silence before re-requesting the same chunk
STALL_LIMIT = 6               # consecutive silent rounds before giving up
COUNT = 65535                 # bytes asked per LOG_REQUEST_DATA
HB_TIMEOUT = 8.0              # s to watch heartbeats before listing
LIST_TIMEOUT = 6.0            # s of silence after the last LOG_ENTRY

OUT = os.environ.get('LOG_OUT') or os.getcwd()


def parse_target(text):
    """'7' -> (7, 1); '7:1' -> (7, 1). PX4 autopilot component id is 1."""
    parts = str(text).split(':')
    ts = int(parts[0])
    tc = int(parts[1]) if len(parts) > 1 else 1
    return ts, tc


def watch_heartbeats(m, seconds):
    """Report heartbeat sources on :15500 (all powered boards plus the routerd)."""
    seen = {}
    t_end = time.time() + seconds
    while time.time() < t_end:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb is None:
            continue
        sysid, compid = hb.get_srcSystem(), hb.get_srcComponent()
        if sysid not in seen:
            seen[sysid] = compid
            print('HEARTBEAT sys=%d comp=%d type=%d autopilot=%d'
                  % (sysid, compid, hb.type, hb.autopilot), flush=True)
    return seen


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


def fetch(m, ts, tc, lid, size):
    path = os.path.join(OUT, 'log_%d.ulg' % lid)
    offset = os.path.getsize(path) if os.path.exists(path) else 0
    if size and offset >= size:
        print('skip id=%d already %d bytes' % (lid, offset), flush=True)
        return True
    print('--- id=%d size=%d start@%d -> %s' % (lid, size, offset, path), flush=True)
    stall, t0, reported = 0, time.time(), 0
    with open(path, 'ab' if offset else 'wb') as fh:
        while (not size) or offset < size:
            m.mav.log_request_data_send(ts, tc, lid, offset, COUNT)
            idle_end, progressed = time.time() + CHUNK_TIMEOUT, False
            while time.time() < idle_end:
                msg = m.recv_match(type='LOG_DATA', blocking=True, timeout=0.2)
                if msg is None:
                    continue
                if int(msg.id) != lid or int(msg.ofs) != offset:
                    continue
                n = int(msg.count)
                if n <= 0:
                    continue
                fh.write(bytes(msg.data[:n]))
                offset += n
                progressed = True
                idle_end = time.time() + CHUNK_TIMEOUT
                if offset - reported >= 262144:
                    reported = offset
                    print('    ... %d/%s bytes (%.0fs)'
                          % (offset, size or '?', time.time() - t0), flush=True)
            if progressed:
                stall = 0
                continue
            stall += 1
            print('    stall @%d (try %d/%d) %.0fs'
                  % (offset, stall, STALL_LIMIT, time.time() - t0), flush=True)
            if stall >= STALL_LIMIT:
                break
    got = os.path.getsize(path)
    complete = (not size) or got >= size
    print('done id=%d -> %d bytes in %.0fs%s'
          % (lid, got, time.time() - t0,
             '' if complete else '  !! SHORT (expected %d)' % size), flush=True)
    return complete


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-t', '--target', required=True,
                    help='UAV number (7 = UAV7) or explicit sys:comp (7:1)')
    ap.add_argument('-n', '--last', type=int, default=2,
                    help='newest N entries to take when no log_id is given (default 2)')
    ap.add_argument('log_id', nargs='*', type=int, help='explicit ulog ids')
    args = ap.parse_args()
    ts, tc = parse_target(args.target)

    os.makedirs(OUT, exist_ok=True)
    m = mavutil.mavlink_connection(SRC)

    watch_heartbeats(m, HB_TIMEOUT)
    entries = list_entries(m, ts, tc)
    if not entries:
        print('NO ENTRIES RECEIVED', flush=True)
        sys.exit(2)

    ids = sorted(entries)
    print('target sys=%d comp=%d; %d logs id %d..%d'
          % (ts, tc, len(ids), ids[0], ids[-1]), flush=True)
    want = args.log_id or ids[-args.last:]
    short = 0
    for lid in want:
        if lid not in entries:
            print('id=%d not on FCU, skip' % lid, flush=True)
            continue
        if not fetch(m, ts, tc, lid, int(entries[lid].size)):
            short += 1
    m.mav.log_request_end_send(ts, tc)
    print('ALL DONE%s' % (' (SHORT: %d)' % short if short else ''), flush=True)
    if short:
        sys.exit(3)


if __name__ == '__main__':
    main()
