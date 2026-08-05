#!/usr/bin/env python3
"""Validate HOLD interruption behavior for an active FOLLOW_ROUTE command."""

from __future__ import annotations

import json
import socket
import sys


HOST = "127.0.0.1"
PORT = 39001


def recv_json_line(sock: socket.socket) -> dict:
    data = b""
    while b"\n" not in data:
        chunk = sock.recv(4096)
        if not chunk:
            raise RuntimeError("connection closed before full JSON line")
        data += chunk
    return json.loads(data.split(b"\n", 1)[0].decode("utf-8"))


def main() -> int:
    follow = {
        "version": "1.0",
        "type": "COMMAND",
        "mission_id": "hold-mission",
        "group_id": "GroupA",
        "command_id": "hold-route-001",
        "command": "FOLLOW_ROUTE",
        "timeout_s": 30.0,
        "leader_id": "A01",
        "assignments": [
            {
                "uav_id": "A01",
                "waypoints": [
                    {"x": 0.0, "y": 0.0, "z": 5.0, "yaw": 0.0},
                    {"x": 10.0, "y": 0.0, "z": 5.0, "yaw": 0.0},
                ],
            },
            {"uav_id": "A02", "formation_follow": True},
        ],
    }
    hold = {
        "version": "1.0",
        "type": "COMMAND",
        "mission_id": "hold-mission",
        "group_id": "GroupA",
        "command_id": "hold-cmd-001",
        "command": "HOLD",
        "timeout_s": 5.0,
        "assignments": [{"uav_id": "A01"}, {"uav_id": "A02"}],
        "reason": "manual hold test",
    }

    with socket.create_connection((HOST, PORT), timeout=3.0) as sock:
        sock.sendall((json.dumps(follow, separators=(",", ":")) + "\n").encode("utf-8"))
        accepted = recv_json_line(sock)
        sock.sendall((json.dumps(hold, separators=(",", ":")) + "\n").encode("utf-8"))
        hold_response = recv_json_line(sock)
        sock.sendall((json.dumps(follow, separators=(",", ":")) + "\n").encode("utf-8"))
        cached = recv_json_line(sock)

    print(
        json.dumps(
            {
                "accepted": accepted,
                "hold_response": hold_response,
                "cached": cached,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    if accepted.get("status") != "ACCEPTED":
        print("expected accepted status ACCEPTED", file=sys.stderr)
        return 1
    if hold_response.get("status") != "COMPLETED":
        print("expected HOLD status COMPLETED", file=sys.stderr)
        return 1
    if cached.get("status") != "FAILED":
        print("expected cached original status FAILED", file=sys.stderr)
        return 1
    if cached.get("error_code") != "COMMAND_HELD":
        print("expected cached error_code COMMAND_HELD", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
