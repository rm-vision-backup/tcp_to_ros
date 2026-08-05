#!/usr/bin/env python3
"""Validate that tcp_to_ros rejects commands for the wrong group."""

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
    command = {
        "version": "1.0",
        "type": "COMMAND",
        "mission_id": "wrong-group-test",
        "group_id": "GroupB",
        "command_id": "wrong-group-001",
        "command": "MOVE_TO",
        "timeout_s": 10.0,
        "assignments": [
            {"uav_id": "A01", "x": 1.0, "y": 2.0, "z": 3.0, "yaw": 0.0}
        ],
    }

    with socket.create_connection((HOST, PORT), timeout=3.0) as sock:
        sock.sendall(
            (json.dumps(command, separators=(",", ":")) + "\n").encode("utf-8")
        )
        response = recv_json_line(sock)

    print(json.dumps(response, ensure_ascii=False, indent=2))
    if response.get("status") != "FAILED":
        print("expected status FAILED", file=sys.stderr)
        return 1
    if response.get("error_code") != "WRONG_GROUP":
        print("expected error_code WRONG_GROUP", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
