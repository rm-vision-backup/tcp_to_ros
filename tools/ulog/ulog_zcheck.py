#!/usr/bin/env python3
"""Quick sanity check on PX4 ULogs: local/global position, armed, nav state.

Usage: python3 ulog_zcheck.py <log.ulg> [more.ulg ...]
"""
import sys

from pyulog import ULog


def rng(d, name, field):
    ds = d.get(name)
    if ds is None or field not in ds.data:
        return None
    x = ds.data[field]
    return 'min=%.2f max=%.2f n=%d' % (float(min(x)), float(max(x)), len(x))


def show(path):
    u = ULog(path)
    d = {x.name: x for x in u.data_list}
    print('== %s' % path)
    print('   duration_s=%.1f  start_timestamp_us=%d'
          % ((u.last_timestamp - u.start_timestamp) / 1e6, u.start_timestamp))
    for name, field in [('vehicle_local_position_setpoint', 'x'),
                        ('vehicle_local_position_setpoint', 'y'),
                        ('vehicle_local_position_setpoint', 'vx'),
                        ('vehicle_local_position_setpoint', 'vy'),
                        ('vehicle_local_position_setpoint', 'vz'),
                        ('trajectory_setpoint', 'position[2]'),
                        ('trajectory_setpoint', 'velocity[2]'),
                        ('offboard_control_mode', 'position'),
                        ('offboard_control_mode', 'velocity'),
                        ('vehicle_local_position', 'z'),
                        ('vehicle_local_position', 'vz'),
                        ('vehicle_global_position', 'alt'),
                        ('vehicle_global_position', 'lat'),
                        ('vehicle_global_position', 'lon'),
                        ('vehicle_local_position_setpoint', 'z'),
                        ('actuator_armed', 'armed'),
                        ('vehicle_status', 'nav_state'),
                        ('commander_state', 'main_state')]:
        print('   %-32s %-8s %s' % (name, field, rng(d, name, field)))
    print('   datasets=%d' % len(d))


for p in sys.argv[1:]:
    try:
        show(p)
    except Exception as e:  # noqa: BLE001 - report and continue
        print('!! %s: %s' % (p, e))
