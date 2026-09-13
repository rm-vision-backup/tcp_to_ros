#!/usr/bin/env python3
"""Run read-only PX4 nsh (NSH shell) commands over MAVLink SERIAL_CONTROL.

Usage:  nsh_fcu.py -t N "ls /fs/microsd" "mount" ...

  -t N      UAV number, or explicit "sys:comp":  -t 7 = UAV7 = 7:1   (required)

The MAVLink source is fixed at udpin:0.0.0.0:15500 (onboard mavlink-routerd forwards
the FCU link there). Read-only commands only: this tool is for inspection, not repair.
"""
import argparse
import sys
import time

from pymavlink import mavutil

mavlink = mavutil.mavlink

# Fixed: onboard mavlink-routerd forwards the FCU link to this port (no env, no flag).
SRC = 'udpin:0.0.0.0:15500'
HB_TIMEOUT = 15.0


def parse_target(text):
    """'7' -> (7, 1); '7:1' -> (7, 1). PX4 autopilot component id is 1."""
    parts = str(text).split(':')
    ts = int(parts[0])
    tc = int(parts[1]) if len(parts) > 1 else 1
    return ts, tc


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-t', '--target', required=True,
                    help='UAV number (7 = UAV7) or explicit sys:comp (7:1)')
    ap.add_argument('commands', nargs='+', help='read-only nsh commands')
    args = ap.parse_args()
    ts, tc = parse_target(args.target)

    m = mavutil.mavlink_connection(SRC)

    t_end = time.time() + HB_TIMEOUT
    while time.time() < t_end:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb is not None and hb.get_srcSystem() == ts and hb.autopilot == 12:
            break
    else:
        sys.exit('no PX4 heartbeat from %d:%d' % (ts, tc))

    flags = (mavlink.SERIAL_CONTROL_FLAG_RESPOND | mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
             | mavlink.SERIAL_CONTROL_FLAG_MULTI)

    def shell(cmd, budget=8.0):
        """Send one nsh command, collect output until the link goes quiet."""
        data = (cmd + '\n').encode()
        buf = bytearray(70)
        buf[:len(data)] = data
        m.mav.serial_control_send(mavlink.SERIAL_CONTROL_DEV_SHELL, flags, 0, 0,
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

    for cmd in args.commands:
        print('=== nsh: %s ===' % cmd, flush=True)
        print(shell(cmd), flush=True)

    # release the exclusive shell session
    m.mav.serial_control_send(mavlink.SERIAL_CONTROL_DEV_SHELL,
                              mavlink.SERIAL_CONTROL_FLAG_RESPOND, 0, 0, 0,
                              bytearray(70))
    print('=== shell released ===', flush=True)


if __name__ == '__main__':
    main()
