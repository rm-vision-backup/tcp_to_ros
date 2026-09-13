#!/usr/bin/env python3
"""List MAVLink vehicle_command rows in PX4 ULogs; decode SET_GPS_GLOBAL_ORIGIN (48).

Usage: python3 ulog_cmds.py <log.ulg> [more.ulg ...]
"""
import sys

import numpy as np
from pyulog import ULog


def show(p):
    u = ULog(p)
    d = {x.name: x for x in u.data_list}
    vc = d.get('vehicle_command')
    print('== %s' % p)
    if vc is None:
        print('   no vehicle_command dataset')
        return
    cmd = vc.data['command']
    ts = vc.data['timestamp']
    print('   rows=%d  commands=%s'
          % (len(cmd), sorted(set(int(x) for x in cmd))))
    for c in sorted(set(int(x) for x in cmd)):
        m = cmd == c
        print('   cmd=%-6d n=%d  t+[%.1f..%.1f]s'
              % (c, int(m.sum()), float(np.min(ts[m])) / 1e6, float(np.max(ts[m])) / 1e6))
        for k in ('param1', 'param2', 'param3', 'param4', 'param5', 'param6', 'param7'):
            if k in vc.data:
                v = np.unique(vc.data[k][m])
                print('      %s = %s' % (k, v[:4]))
    m = cmd == 48
    if m.any():
        print('   SET_GPS_GLOBAL_ORIGIN (48): lat=%s lon=%s alt=%s @t+%ss'
              % (vc.data['param5'][m], vc.data['param6'][m], vc.data['param7'][m],
                 np.round(ts[m] / 1e6, 1)))


for path in sys.argv[1:]:
    show(path)
