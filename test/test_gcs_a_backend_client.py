#!/usr/bin/env python3
"""Integration test client for GCS_A backend (ExecuteGroupTask Action).

Usage:
  python3 test_gcs_a_backend_client.py --uavs A01,A02,A03 \
      --command MOVE_TO --timeout 15 --expect COMPLETED
"""

from __future__ import annotations

import argparse
import sys
import time

import actionlib
import rospy

from tcp_to_ros.msg import (
    ExecuteGroupTaskAction,
    ExecuteGroupTaskGoal,
    Pose3DYaw,
    TaskAssignment,
)


def build_goal(
    mission_id: str,
    command_id: str,
    command: str,
    timeout_s: float,
    uav_ids,
    leader_id: str = "",
) -> ExecuteGroupTaskGoal:
    goal = ExecuteGroupTaskGoal()
    goal.mission_id = mission_id
    goal.group_id = "GroupA"
    goal.command_id = command_id
    goal.command = command
    goal.timeout_s = float(timeout_s)
    goal.leader_id = leader_id
    for idx, uav_id in enumerate(uav_ids):
        item = TaskAssignment()
        item.uav_id = uav_id
        if command == "MOVE_TO":
            item.target_pose = Pose3DYaw(
                x=float(idx + 1), y=10.0, z=12.0, yaw=0.0
            )
        elif command in {"FOLLOW_ROUTE", "FAULT_EXIT"}:
            if command == "FOLLOW_ROUTE" and uav_id == leader_id:
                item.waypoints = [
                    Pose3DYaw(x=10.0, y=10.0, z=12.0, yaw=0.0),
                    Pose3DYaw(x=50.0, y=10.0, z=12.0, yaw=0.0),
                ]
            elif command == "FAULT_EXIT":
                item.waypoints = [
                    Pose3DYaw(x=80.0, y=float(idx + 1), z=8.0, yaw=0.0),
                    Pose3DYaw(x=5.0, y=float(idx + 1), z=8.0, yaw=0.0),
                ]
            else:
                item.formation_follow = True
        goal.assignments.append(item)
    return goal


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--action", default="/group_a/execute_task")
    parser.add_argument("--mission", default="it-mission-001")
    parser.add_argument("--command", default="MOVE_TO")
    parser.add_argument("--command-id", default="")
    parser.add_argument("--uavs", default="A01,A02,A03")
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument(
        "--expect", default="COMPLETED", choices=["COMPLETED", "FAILED"]
    )
    parser.add_argument("--leader", default="")
    args = parser.parse_args()

    rospy.init_node("gcs_a_backend_test_client", anonymous=True)
    command_id = args.command_id or "it-cmd-{}".format(int(time.time()))
    uav_ids = [item.strip() for item in args.uavs.split(",") if item.strip()]

    client = actionlib.SimpleActionClient(args.action, ExecuteGroupTaskAction)
    rospy.loginfo("waiting for action %s", args.action)
    if not client.wait_for_server(rospy.Duration(10.0)):
        print("RESULT: FAILED_TO_CONNECT")
        return 2
    rospy.loginfo("connected to action server")

    goal = build_goal(
        mission_id=args.mission,
        command_id=command_id,
        command=args.command,
        timeout_s=args.timeout,
        uav_ids=uav_ids,
        leader_id=args.leader,
    )

    result_box = {}

    def on_done(state, result):
        result_box["state"] = state
        result_box["result"] = result

    client.send_goal(goal, done_cb=on_done)
    finished = client.wait_for_result(rospy.Duration(args.timeout + 8.0))
    if not finished:
        print("RESULT: NO_RESULT timeout")
        return 2

    result = result_box.get("result")
    final_status = str(getattr(result, "final_status", "") or "")
    completed = list(getattr(result, "completed_uavs", []))
    failed = list(getattr(result, "failed_uavs", []))
    print("RESULT_STATE=%d" % result_box.get("state", -1))
    print("RESULT_FINAL=%s" % final_status)
    print("RESULT_COMPLETED=%s" % ",".join(completed))
    print("RESULT_FAILED=%s" % ",".join(failed))
    print("RESULT_MESSAGE=%s" % getattr(result, "message", ""))
    print("RESULT_ERROR=%s" % getattr(result, "error_code", ""))

    if final_status == args.expect:
        print("ASSERT_OK expect=%s" % args.expect)
        return 0
    print("ASSERT_FAIL expect=%s got=%s" % (args.expect, final_status))
    return 1


if __name__ == "__main__":
    sys.exit(main())