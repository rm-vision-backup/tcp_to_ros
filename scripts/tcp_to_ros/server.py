"""TCP server utilities for newline-delimited JSON ground-station messages."""

from __future__ import annotations

import json
import socket
import threading
from typing import Any, Dict, Optional

from .protocol import MAX_MESSAGE_BYTES


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
