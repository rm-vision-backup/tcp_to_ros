#!/usr/bin/env python3
"""Run read-only PX4 nsh (NSH shell) commands over MAVLink SERIAL_CONTROL.

Usage: MAV_TARGET=7:1 python3 nsh_fcu.py "ls /fs/microsd" "mount" ...
"""
import os
import sys
import time

from pymavlink import mavutil

mavlink = mavutil.mavlink

SRC = os.environ.get('MAV_SRC', 'udpin:0.0.0.0:15500')
TARGET = os.environ.get('MAV_TARGET', '7:1')
TS, TC = (int(x) for x in TARGET.split(':'))

m = mavutil.mavlink_connection(SRC)

t_end = time.time() + 15
while time.time() < t_end:
    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if hb is not None and hb.get_srcSystem() == TS and hb.autopilot == 12:
        break
else:
    sys.exit('no PX4 heartbeat from %d:%d' % (TS, TC))

FLAGS = (mavlink.SERIAL_CONTROL_FLAG_RESPOND | mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
         | mavlink.SERIAL_CONTROL_FLAG_MULTI)


def shell(cmd, budget=8.0):
    """Send one nsh command, collect output until the link goes quiet."""
    data = (cmd + '\n').encode()
    buf = bytearray(70)
    buf[:len(data)] = data
    m.mav.serial_control_send(mavlink.SERIAL_CONTROL_DEV_SHELL, FLAGS, 0, 0,
                              len(data), buf)
    out = bytearray()
    t_end = time.time() + budget
    while time.time() < t_end:
        msg = m.recv_match(type='SERIAL_CONTROL', blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.device != mavlink.SERIAL_CONTROL_DEV_SHELL:
            continue
        out += bytes(msg.data[:msg.count])
        t_end = time.time() + 2.0
    return bytes(out).decode(errors='replace')


for cmd in sys.argv[1:]:
    print('=== nsh: %s ===' % cmd, flush=True)
    print(shell(cmd), flush=True)

# release the exclusive shell session
m.mav.serial_control_send(mavlink.SERIAL_CONTROL_DEV_SHELL,
                          mavlink.SERIAL_CONTROL_FLAG_RESPOND, 0, 0, 0,
                          bytearray(70))
print('=== shell released ===', flush=True)
