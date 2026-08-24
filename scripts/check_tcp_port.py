#!/usr/bin/env python3
"""Check that the runtime TCP port cannot be taken as a Linux ephemeral port."""

import argparse
import sys

from tcp_to_ros.server import linux_port_is_ephemeral_and_unreserved


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=39001)
    args = parser.parse_args()

    try:
        unsafe = linux_port_is_ephemeral_and_unreserved(args.port)
    except (OSError, ValueError) as exc:
        print("ERROR: cannot inspect Linux port policy: %s" % exc, file=sys.stderr)
        return 2
    if unsafe:
        print(
            "ERROR: TCP port %d is available for ephemeral allocation. "
            "Reserve it on the GCS host with "
            "net.ipv4.ip_local_reserved_ports before starting ROS nodes."
            % args.port,
            file=sys.stderr,
        )
        return 1
    print("OK: TCP port %d is outside the ephemeral pool or reserved" % args.port)
    return 0


if __name__ == "__main__":
    sys.exit(main())