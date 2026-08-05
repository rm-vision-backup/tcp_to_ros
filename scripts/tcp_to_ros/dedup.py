"""Command deduplication and status caching helpers."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Dict, Optional


@dataclass
class CommandCacheEntry:
    command_id: str
    final_response: Dict[str, Any]


class CommandCache:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._entries: Dict[str, CommandCacheEntry] = {}

    def get(self, command_id: str) -> Optional[CommandCacheEntry]:
        with self._lock:
            return self._entries.get(command_id)

    def set_final(self, command_id: str, final_response: Dict[str, Any]) -> None:
        with self._lock:
            self._entries[command_id] = CommandCacheEntry(
                command_id=command_id,
                final_response=dict(final_response),
            )
