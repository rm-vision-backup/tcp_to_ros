"""TCP protocol parsing and status-message helpers for Group A ground station."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence


PROTOCOL_VERSION = "1.0"
GROUP_ID = "GroupA"
MAX_MESSAGE_BYTES = 64 * 1024
SUPPORTED_COMMANDS = {"MOVE_TO", "FOLLOW_ROUTE", "FAULT_EXIT", "HOVER", "HOLD"}


class CloseConnectionError(ValueError):
    """Raised when the peer sent a line that requires closing the TCP connection."""


class CommandValidationError(ValueError):
    """Raised when a parsed command is invalid but still has enough IDs to fail closed."""

    def __init__(
        self,
        message: str,
        error_code: str = "INVALID_COMMAND",
        failed_uavs: Optional[Sequence[str]] = None,
    ) -> None:
        super().__init__(message)
        self.error_code = error_code
        self.failed_uavs = list(failed_uavs or [])


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    yaw: float = 0.0


@dataclass(frozen=True)
class Assignment:
    uav_id: str
    formation_follow: bool = False
    target_id: str = ""
    target_pose: Optional[Pose] = None
    waypoints: Sequence[Pose] = ()
    formation_offset: Optional[Pose] = None


@dataclass(frozen=True)
class ParsedCommand:
    mission_id: str
    group_id: str
    command_id: str
    command: str
    timeout_s: float
    assignments: Sequence[Assignment]
    leader_id: str = ""
    reason: str = ""

    @property
    def uav_ids(self) -> List[str]:
        return [item.uav_id for item in self.assignments]


def parse_json_line(decoded: Any) -> ParsedCommand:
    """Validate one decoded JSON message and normalize field types."""

    if not isinstance(decoded, dict):
        raise CloseConnectionError("TCP message must be a JSON object")

    required_fields = [
        "version",
        "type",
        "mission_id",
        "group_id",
        "command_id",
        "command",
        "timeout_s",
        "assignments",
    ]
    missing = [field for field in required_fields if field not in decoded]
    if missing:
        raise CloseConnectionError(
            "TCP message missing required fields: {}".format(",".join(missing))
        )

    if decoded.get("version") != PROTOCOL_VERSION:
        raise _semantic_error(decoded, "unsupported protocol version", "BAD_VERSION")
    if decoded.get("type") != "COMMAND":
        raise _semantic_error(decoded, "unsupported message type", "BAD_TYPE")

    mission_id = _required_str(decoded, "mission_id")
    group_id = _required_str(decoded, "group_id")
    command_id = _required_str(decoded, "command_id")
    command = _required_str(decoded, "command").upper()

    if group_id != GROUP_ID:
        raise _semantic_error(
            decoded,
            "command sent to the wrong ground station",
            "WRONG_GROUP",
        )
    if command not in SUPPORTED_COMMANDS:
        raise _semantic_error(decoded, "unsupported command", "BAD_COMMAND")

    try:
        timeout_s = float(decoded["timeout_s"])
    except (TypeError, ValueError) as exc:
        raise _semantic_error(decoded, "timeout_s must be numeric", "BAD_TIMEOUT") from exc
    if timeout_s <= 0.0:
        raise _semantic_error(decoded, "timeout_s must be positive", "BAD_TIMEOUT")

    assignments_value = decoded.get("assignments")
    if not isinstance(assignments_value, list) or not assignments_value:
        raise _semantic_error(
            decoded,
            "assignments must be a non-empty array",
            "BAD_ASSIGNMENTS",
        )

    assignments = [_parse_assignment(command, item) for item in assignments_value]
    uav_ids = [item.uav_id for item in assignments]
    if len(uav_ids) != len(set(uav_ids)):
        raise _semantic_error(decoded, "assignments contain duplicate uav_id", "DUPLICATE_UAV")

    leader_id = ""
    if command == "FOLLOW_ROUTE":
        leader_id = _required_str(decoded, "leader_id")
        if leader_id not in uav_ids:
            raise _semantic_error(
                decoded,
                "leader_id must appear in assignments",
                "BAD_LEADER",
            )
        leader_routes = [
            item for item in assignments if item.uav_id == leader_id and item.waypoints
        ]
        if len(leader_routes) != 1:
            raise _semantic_error(
                decoded,
                "FOLLOW_ROUTE leader must provide exactly one non-empty waypoint list",
                "BAD_LEADER_ROUTE",
            )
        for item in assignments:
            if item.uav_id == leader_id:
                continue
            if not item.formation_follow:
                raise _semantic_error(
                    decoded,
                    "FOLLOW_ROUTE followers must set formation_follow=true",
                    "BAD_FOLLOWER",
                )

    reason = ""
    if command == "HOLD":
        reason = str(decoded.get("reason") or "")

    return ParsedCommand(
        mission_id=mission_id,
        group_id=group_id,
        command_id=command_id,
        command=command,
        timeout_s=timeout_s,
        assignments=assignments,
        leader_id=leader_id,
        reason=reason,
    )


def build_accepted(command: ParsedCommand, message: str = "") -> Dict[str, Any]:
    payload = _status_envelope(command, "ACCEPTED")
    if message:
        payload["message"] = message
    return payload


def build_completed(
    command: ParsedCommand,
    completed_uavs: Sequence[str],
    message: str = "",
) -> Dict[str, Any]:
    payload = _status_envelope(command, "COMPLETED")
    payload["completed_uavs"] = list(completed_uavs)
    if message:
        payload["message"] = message
    return payload


def build_failed(
    command: ParsedCommand,
    failed_uavs: Sequence[str],
    error_code: str = "",
    message: str = "",
) -> Dict[str, Any]:
    payload = _status_envelope(command, "FAILED")
    payload["failed_uavs"] = list(failed_uavs)
    if error_code:
        payload["error_code"] = error_code
    if message:
        payload["message"] = message
    return payload


def pose_to_dict(pose: Optional[Pose]) -> Dict[str, float]:
    if pose is None:
        return {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
    return {"x": pose.x, "y": pose.y, "z": pose.z, "yaw": pose.yaw}


def _status_envelope(command: ParsedCommand, status: str) -> Dict[str, Any]:
    return {
        "version": PROTOCOL_VERSION,
        "type": "STATUS",
        "mission_id": command.mission_id,
        "group_id": command.group_id,
        "command_id": command.command_id,
        "status": status,
    }


def _parse_assignment(command: str, value: Any) -> Assignment:
    if not isinstance(value, dict):
        raise CommandValidationError("assignment must be an object")

    uav_id = _required_str(value, "uav_id")
    if command == "MOVE_TO":
        return Assignment(uav_id=uav_id, target_pose=_parse_pose(value))
    if command == "FOLLOW_ROUTE":
        if value.get("formation_follow") is True:
            offset = _parse_optional_offset(value)
            return Assignment(uav_id=uav_id, formation_follow=True, formation_offset=offset)
        return Assignment(uav_id=uav_id, waypoints=_parse_waypoints(value))
    if command == "FAULT_EXIT":
        return Assignment(uav_id=uav_id, waypoints=_parse_waypoints(value))
    if command in {"HOVER", "HOLD"}:
        return Assignment(uav_id=uav_id)
    raise CommandValidationError(f"unsupported command {command!r}")


def _parse_waypoints(value: Mapping[str, Any]) -> Sequence[Pose]:
    points = value.get("waypoints")
    if not isinstance(points, list) or not points:
        raise CommandValidationError("waypoints must be a non-empty array")
    return [_parse_pose(item) for item in points]


def _parse_optional_offset(value: Mapping[str, Any]) -> Optional[Pose]:
    """Parse an optional formation_offset {x,y,z[,yaw]} block from an assignment."""
    offset = value.get("formation_offset")
    if offset is None:
        return None
    if not isinstance(offset, dict):
        raise CommandValidationError("formation_offset must be an object")
    return _parse_pose(offset)


def _parse_pose(value: Mapping[str, Any]) -> Pose:
    try:
        return Pose(
            x=float(value["x"]),
            y=float(value["y"]),
            z=float(value["z"]),
            yaw=float(value.get("yaw", 0.0)),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise CommandValidationError("pose requires numeric x, y, z, yaw") from exc


def _required_str(value: Mapping[str, Any], field: str) -> str:
    raw = value.get(field)
    if not isinstance(raw, str) or not raw:
        raise CommandValidationError(f"{field} must be a non-empty string")
    return raw


def _semantic_error(
    decoded: Mapping[str, Any],
    message: str,
    error_code: str,
) -> CommandValidationError:
    return CommandValidationError(
        message,
        error_code=error_code,
        failed_uavs=_extract_uav_ids(decoded.get("assignments")),
    )


def _extract_uav_ids(value: Any) -> List[str]:
    if not isinstance(value, list):
        return []
    ids: List[str] = []
    for item in value:
        if isinstance(item, dict):
            uav_id = item.get("uav_id")
            if isinstance(uav_id, str) and uav_id:
                ids.append(uav_id)
    return ids
