"""Active-task state tracking for the TCP server node."""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Dict, Optional

from .protocol import ParsedCommand


@dataclass
class ActiveTask:
    command: ParsedCommand
    goal_serial: int
    accepted_sent: bool = False
    suppressed: bool = False
    metadata: Dict[str, str] = field(default_factory=dict)


class ActiveTaskRegistry:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._current: Optional[ActiveTask] = None
        self._next_serial = 1

    def current(self) -> Optional[ActiveTask]:
        with self._lock:
            return self._current

    def start(self, command: ParsedCommand) -> ActiveTask:
        with self._lock:
            task = ActiveTask(command=command, goal_serial=self._next_serial)
            self._next_serial += 1
            self._current = task
            return task

    def mark_accepted(self, command_id: str) -> Optional[ActiveTask]:
        with self._lock:
            if self._current is None or self._current.command.command_id != command_id:
                return None
            self._current.accepted_sent = True
            return self._current

    def suppress_current(self) -> Optional[ActiveTask]:
        with self._lock:
            if self._current is None:
                return None
            task = self._current
            task.suppressed = True
            self._current = None
            return task

    def finish(self, goal_serial: int) -> Optional[ActiveTask]:
        with self._lock:
            if self._current is None or self._current.goal_serial != goal_serial:
                return None
            task = self._current
            self._current = None
            return task
