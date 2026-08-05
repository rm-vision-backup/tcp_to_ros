"""Conversions between TCP commands and ROS action messages."""

from __future__ import annotations

from typing import Iterable, List, Sequence, Tuple

from tcp_to_ros.msg import Pose3DYaw, TaskAssignment
from tcp_to_ros.msg import ExecuteGroupTaskGoal

from .protocol import Assignment, ParsedCommand


def goal_from_command(command: ParsedCommand) -> ExecuteGroupTaskGoal:
    goal = ExecuteGroupTaskGoal()
    goal.mission_id = command.mission_id
    goal.group_id = command.group_id
    goal.command_id = command.command_id
    goal.command = command.command
    goal.timeout_s = float(command.timeout_s)
    goal.leader_id = command.leader_id
    goal.assignments = [_assignment_to_msg(item) for item in command.assignments]
    return goal


def completed_uavs_from_result(
    expected_uavs: Sequence[str],
    completed_uavs: Sequence[str],
) -> List[str]:
    completed = [item for item in completed_uavs if item in expected_uavs]
    return completed if completed else list(expected_uavs)


def failed_uavs_from_result(
    expected_uavs: Sequence[str],
    failed_uavs: Sequence[str],
) -> List[str]:
    failed = [item for item in failed_uavs if item in expected_uavs]
    return failed if failed else list(expected_uavs)


def _assignment_to_msg(assignment: Assignment) -> TaskAssignment:
    msg = TaskAssignment()
    msg.uav_id = assignment.uav_id
    msg.formation_follow = bool(assignment.formation_follow)
    msg.target_id = assignment.target_id
    msg.target_pose = _pose_to_msg(assignment.target_pose)
    msg.waypoints = [_pose_to_msg(item) for item in assignment.waypoints]
    return msg


def _pose_to_msg(pose) -> Pose3DYaw:
    msg = Pose3DYaw()
    if pose is None:
        return msg
    msg.x = float(pose.x)
    msg.y = float(pose.y)
    msg.z = float(pose.z)
    msg.yaw = float(pose.yaw)
    return msg
