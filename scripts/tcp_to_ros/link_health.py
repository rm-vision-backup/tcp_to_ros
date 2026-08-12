"""Monotonic per-UAV link-health state machine without ROS side effects."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, replace
from typing import Dict, Iterable, List, Optional, Tuple


PROTOCOL_VERSION = "1.0"
UNDISCOVERED = "UNDISCOVERED"
DISCOVERED = "DISCOVERED"
HEALTHY = "HEALTHY"
SUSPECT = "SUSPECT"
LOST = "LOST"
RECOVERED = "RECOVERED"


@dataclass
class UavLinkObservation:
    uav_id: str
    exec_target: str
    last_message_monotonic_s: Optional[float] = None
    last_valid_monotonic_s: Optional[float] = None
    last_connected: bool = False
    valid_sample_count: int = 0
    healthy_since_monotonic_s: Optional[float] = None
    state: str = UNDISCOVERED
    reason: str = "no MAVROS state received"
    recovery_pending: bool = False


@dataclass(frozen=True)
class LinkHealthTransition:
    uav_id: str
    exec_target: str
    previous: str
    current: str
    age_s: float
    connected: bool
    reason: str


class UavLinkMonitor:
    """Track configured UAVs while never treating undiscovered members as lost."""

    def __init__(
        self,
        identities: Iterable[Tuple[str, str]],
        suspect_after_s: float = 3.0,
        lost_after_s: float = 5.0,
        healthy_window_s: float = 3.0,
        clock=time.monotonic,
    ) -> None:
        if suspect_after_s <= 0.0:
            raise ValueError("suspect_after_s must be positive")
        if lost_after_s <= suspect_after_s:
            raise ValueError("lost_after_s must be greater than suspect_after_s")
        if healthy_window_s < 0.0:
            raise ValueError("healthy_window_s must not be negative")
        self.suspect_after_s = float(suspect_after_s)
        self.lost_after_s = float(lost_after_s)
        self.healthy_window_s = float(healthy_window_s)
        self._clock = clock
        self._lock = threading.RLock()
        self._observations: Dict[str, UavLinkObservation] = {}
        self._pending: List[LinkHealthTransition] = []
        for uav_id, exec_target in identities:
            if not uav_id or not exec_target:
                raise ValueError("uav_id and exec_target must be non-empty")
            if uav_id in self._observations:
                raise ValueError("duplicate uav_id: %s" % uav_id)
            self._observations[uav_id] = UavLinkObservation(uav_id, exec_target)

    def record_state(
        self,
        uav_id: str,
        exec_target: str,
        connected: bool,
        received_monotonic_s: Optional[float] = None,
    ) -> UavLinkObservation:
        now = self._clock() if received_monotonic_s is None else float(received_monotonic_s)
        with self._lock:
            observation = self._get_matching(uav_id, exec_target)
            previous = observation.state
            observation.last_message_monotonic_s = now
            observation.last_connected = bool(connected)
            if previous == UNDISCOVERED:
                observation.state = DISCOVERED
                observation.reason = "first MAVROS state received"
                self._append_transition(observation, previous, now)
            if connected:
                observation.last_valid_monotonic_s = now
                observation.valid_sample_count += 1
                if previous == LOST:
                    observation.healthy_since_monotonic_s = now
                if observation.healthy_since_monotonic_s is None:
                    observation.healthy_since_monotonic_s = now
                if previous == LOST or observation.state == LOST:
                    observation.recovery_pending = True
            else:
                observation.healthy_since_monotonic_s = None
                observation.recovery_pending = False
                observation.reason = "MAVROS reported connected=false"
            return replace(observation)

    def evaluate(self, now_monotonic_s: Optional[float] = None) -> List[LinkHealthTransition]:
        now = self._clock() if now_monotonic_s is None else float(now_monotonic_s)
        with self._lock:
            transitions = list(self._pending)
            self._pending.clear()
            for observation in self._observations.values():
                if observation.state == UNDISCOVERED:
                    continue
                previous = observation.state
                age = self._valid_age(observation, now)
                if observation.last_valid_monotonic_s is not None and age >= self.lost_after_s:
                    current = LOST
                    reason = "valid MAVROS heartbeat timed out"
                elif not observation.last_connected:
                    current = SUSPECT
                    reason = "MAVROS reported connected=false"
                elif age >= self.suspect_after_s:
                    current = SUSPECT
                    reason = "valid MAVROS heartbeat is stale"
                elif observation.recovery_pending:
                    current = RECOVERED
                    reason = "valid MAVROS heartbeat recovered"
                    observation.recovery_pending = False
                elif self._stable_for_window(observation, now):
                    current = HEALTHY
                    reason = "valid MAVROS heartbeat is stable"
                else:
                    current = previous if previous in (DISCOVERED, RECOVERED) else DISCOVERED
                    reason = "waiting for healthy stability window"
                if current != previous:
                    observation.state = current
                    observation.reason = reason
                    if current in (SUSPECT, LOST):
                        observation.healthy_since_monotonic_s = None
                    transitions.append(self._make_transition(observation, previous, now))
                else:
                    observation.reason = reason
            return transitions

    def is_healthy_or_undiscovered(self, uav_id: str, now_monotonic_s: Optional[float] = None) -> bool:
        """Return True for UNDISCOVERED UAVs (never powered on), or HEALTHY ones.

        UNDISCOVERED UAVs are considered "healthy enough" to avoid fail-fast
        pre-checks on the group task. Only genuine link problems (stale/lost
        heartbeat on a UAV that actually transmitted MAVROS state) are blocked.
        """
        now = self._clock() if now_monotonic_s is None else float(now_monotonic_s)
        with self._lock:
            observation = self._observations[uav_id]
            if observation.state == UNDISCOVERED:
                return True
            return (
                observation.state == HEALTHY
                and observation.last_connected
                and self._valid_age(observation, now) < self.suspect_after_s
                and self._stable_for_window(observation, now)
            )

    def is_stably_healthy(self, uav_id: str, now_monotonic_s: Optional[float] = None) -> bool:
        now = self._clock() if now_monotonic_s is None else float(now_monotonic_s)
        with self._lock:
            observation = self._observations[uav_id]
            return (
                observation.state == HEALTHY
                and observation.last_connected
                and self._valid_age(observation, now) < self.suspect_after_s
                and self._stable_for_window(observation, now)
            )

    def observation(self, uav_id: str) -> UavLinkObservation:
        with self._lock:
            return replace(self._observations[uav_id])

    def snapshot(
        self,
        uav_id: str,
        ros_time_s: float,
        mission_id: str = "",
        session_epoch: str = "",
        now_monotonic_s: Optional[float] = None,
    ):
        from swarm_uav_interfaces.msg import UavLinkHealth

        now = self._clock() if now_monotonic_s is None else float(now_monotonic_s)
        with self._lock:
            observation = self._observations[uav_id]
            return UavLinkHealth(
                protocol_version=PROTOCOL_VERSION,
                uav_id=observation.uav_id,
                exec_target=observation.exec_target,
                state=observation.state,
                connected=observation.last_connected,
                age_s=self._valid_age(observation, now),
                mission_id=mission_id,
                session_epoch=session_epoch,
                reason=observation.reason,
                reported_at=float(ros_time_s),
            )

    def _get_matching(self, uav_id: str, exec_target: str) -> UavLinkObservation:
        if uav_id not in self._observations:
            raise KeyError("unknown uav_id: %s" % uav_id)
        observation = self._observations[uav_id]
        if observation.exec_target != exec_target:
            raise ValueError("exec_target mismatch for %s" % uav_id)
        return observation

    def _stable_for_window(self, observation: UavLinkObservation, now: float) -> bool:
        started = observation.healthy_since_monotonic_s
        return started is not None and now - started >= self.healthy_window_s

    @staticmethod
    def _valid_age(observation: UavLinkObservation, now: float) -> float:
        if observation.last_valid_monotonic_s is None:
            return -1.0
        return max(0.0, now - observation.last_valid_monotonic_s)

    def _append_transition(self, observation: UavLinkObservation, previous: str, now: float) -> None:
        self._pending.append(self._make_transition(observation, previous, now))

    def _make_transition(
        self, observation: UavLinkObservation, previous: str, now: float
    ) -> LinkHealthTransition:
        return LinkHealthTransition(
            uav_id=observation.uav_id,
            exec_target=observation.exec_target,
            previous=previous,
            current=observation.state,
            age_s=self._valid_age(observation, now),
            connected=observation.last_connected,
            reason=observation.reason,
        )