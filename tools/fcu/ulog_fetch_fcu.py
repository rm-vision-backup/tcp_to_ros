#!/usr/bin/env python3
"""Fetch PX4 ULogs from the FCU over MAVLink, selecting the autopilot target.

Usage:  ULOG_OUT=<dir> python3 ulog_fetch_fcu.py [log_id ...]   (default: newest 2)
Env:    MAV_SRC (default udpin:0.0.0.0:15500), CHUNK_TIMEOUT (idle s before re-request),
        STALL_LIMIT (consecutive idle rounds before giving up), LAST (default ids to take)
"""
import os
import sys
import time

from pymavlink import mavutil

SRC = os.environ.get('MAV_SRC', 'udpin:0.0.0.0:15500')
OUT = os.environ.get('ULOG_OUT', os.getcwd())
CHUNK_TIMEOUT = float(os.environ.get('CHUNK_TIMEOUT', '1.0'))
STALL_LIMIT = int(os.environ.get('STALL_LIMIT', '6'))
COUNT = int(os.environ.get('CHUNK_COUNT', '65535'))
LAST = int(os.environ.get('LAST', '2'))
# Pin the autopilot explicitly (e.g. MAV_TARGET=7:1); mavlink-routerd also emits its
# own heartbeat (autopilot=8), which the auto-detect loop can otherwise latch onto.
TARGET = os.environ.get('MAV_TARGET', '')

os.makedirs(OUT, exist_ok=True)
m = mavutil.mavlink_connection(SRC)


def watch_heartbeats(seconds=8.0):
    """Collect heartbeat sources; PX4 autopilot answers on its own system id."""
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


def list_entries(ts, tc, timeout=6.0):
    entries = {}
    m.mav.log_request_list_send(ts, tc, 0, 0xffff)
    t_end = time.time() + timeout
    while time.time() < t_end:
        msg = m.recv_match(type='LOG_ENTRY', blocking=True, timeout=1)
        if msg is None:
            continue
        if TARGET and (msg.get_srcSystem(), msg.get_srcComponent()) != (ts, tc):
            continue
        entries[int(msg.id)] = msg
        t_end = time.time() + timeout
    return entries


def fetch(ts, tc, lid, size):
    path = os.path.join(OUT, 'log_%d.ulg' % lid)
    offset = os.path.getsize(path) if os.path.exists(path) else 0
    if size and offset >= size:
        print('skip id=%d already %d bytes' % (lid, offset), flush=True)
        return
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
    print('done id=%d -> %d bytes in %.0fs%s'
          % (lid, got, time.time() - t0, '' if (not size or got >= size)
             else '  !! SHORT (expected %d)' % size), flush=True)


cands = watch_heartbeats()
entries, target = {}, None
if TARGET:
    ts, tc = (int(x) for x in TARGET.split(':'))
    entries = list_entries(ts, tc)
    if entries:
        target = (ts, tc)
else:
    for ts, tc in [(s, c) for s, c in sorted(cands.items())] + [(0, 0)]:
        entries = list_entries(ts, tc)
        if entries:
            target = (ts, tc)
            break
if not entries:
    print('NO ENTRIES RECEIVED', flush=True)
    sys.exit(2)

ids = sorted(entries)
ts, tc = target
print('target sys=%d comp=%d; %d logs id %d..%d' % (ts, tc, len(ids), ids[0], ids[-1]),
      flush=True)
want = [int(a) for a in sys.argv[1:]] or ids[-LAST:]
for lid in want:
    if lid not in entries:
        print('id=%d not on FCU, skip' % lid, flush=True)
        continue
    fetch(ts, tc, lid, int(entries[lid].size))
m.mav.log_request_end_send(ts, tc)
print('ALL DONE', flush=True)
