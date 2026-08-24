"""TCP server utilities for newline-delimited JSON ground-station messages."""

from __future__ import annotations

import json
import socket
import threading
from typing import Any, Dict, Optional, Set, Tuple

from .protocol import MAX_MESSAGE_BYTES


IP_LOCAL_PORT_RANGE = "/proc/sys/net/ipv4/ip_local_port_range"
IP_LOCAL_RESERVED_PORTS = "/proc/sys/net/ipv4/ip_local_reserved_ports"


def parse_port_range(value: str) -> Tuple[int, int]:
    parts = value.split()
    if len(parts) != 2:
        raise ValueError("ip_local_port_range must contain two integers")
    lower, upper = (int(part) for part in parts)
    if not 0 <= lower <= upper <= 65535:
        raise ValueError("invalid ip_local_port_range")
    return lower, upper


def parse_reserved_ports(value: str) -> Set[int]:
    ports: Set[int] = set()
    for item in value.strip().split(","):
        item = item.strip()
        if not item:
            continue
        bounds = item.split("-", 1)
        lower = int(bounds[0])
        upper = int(bounds[-1])
        if not 0 <= lower <= upper <= 65535:
            raise ValueError("invalid ip_local_reserved_ports entry: %s" % item)
        ports.update(range(lower, upper + 1))
    return ports


def linux_port_is_ephemeral_and_unreserved(
    port: int,
    range_path: str = IP_LOCAL_PORT_RANGE,
    reserved_path: str = IP_LOCAL_RESERVED_PORTS,
) -> bool:
    """Return whether Linux may allocate ``port`` to an unrelated socket."""
    with open(range_path, "r", encoding="ascii") as stream:
        lower, upper = parse_port_range(stream.read())
    with open(reserved_path, "r", encoding="ascii") as stream:
        reserved = parse_reserved_ports(stream.read())
    return lower <= int(port) <= upper and int(port) not in reserved


class JsonLineConnection:
    def __init__(self, connection: socket.socket, address) -> None:
        self.connection = connection
        self.address = address
        self._send_lock = threading.Lock()
        self._closed = False

    def send_json(self, payload: Dict[str, Any]) -> None:
        raw = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8") + b"\n"
        if len(raw) > MAX_MESSAGE_BYTES:
            raise ValueError("outgoing status message exceeds 64 KiB")
        with self._send_lock:
            if self._closed:
                raise RuntimeError("connection already closed")
            self.connection.sendall(raw)

    def close(self) -> None:
        with self._send_lock:
            if self._closed:
                return
            self._closed = True
            try:
                self.connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.connection.close()


def create_listener(host: str, port: int, backlog: int = 1) -> socket.socket:
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((host, int(port)))
    listener.listen(backlog)
    listener.settimeout(0.5)
    return listener
