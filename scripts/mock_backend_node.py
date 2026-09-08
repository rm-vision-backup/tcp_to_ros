#!/usr/bin/env python3
"""Mock ROS backend that simulates Group A task execution outcomes."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Dict, List

import actionlib
import rospy

from tcp_to_ros.msg import (
    ExecuteGroupTaskAction,
    ExecuteGroupTaskFeedback,
    ExecuteGroupTaskResult,
    GroupTaskState,
)
from tcp_to_ros.srv import EmergencyHold, EmergencyHoldResponse


@dataclass
class MockPolicy:
    accept_delay_s: float
    result_delay_s: float
    send_accepted_feedback: bool
    final_status: str
    failed_uavs: List[str]
    error_code: str
    message: str


class MockBackendNode:
    def __init__(self) -> None:
        self.action_name = rospy.get_param("~action_name", "/group_a/execute_task")
        self.state_topic = rospy.get_param("~state_topic", "/group_a/group_task_state")
        self.hold_service_name = rospy.get_param(
            "~emergency_hold_service",
            "/group_a/emergency_hold",
        )
        self.defaults = rospy.get_param("~defaults", {})
        self.command_overrides = rospy.get_param("~command_overrides", {})

        self.state_pub = rospy.Publisher(
            self.state_topic, GroupTaskState, queue_size=20
        )
        self.server = actionlib.SimpleActionServer(
            self.action_name,
            ExecuteGroupTaskAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self.hold_service = rospy.Service(
            self.hold_service_name, EmergencyHold, self._handle_hold
        )

        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._active_command_id = ""
        self._active_mission_id = ""
        self._active_command = ""
        self._active_uavs: List[str] = []

        self.server.start()
        rospy.loginfo("mock backend ready: action=%s service=%s", self.action_name, self.hold_service_name)

    def _execute(self, goal) -> None:
        policy = self._policy_for(goal.command)
        uav_ids = [item.uav_id for item in goal.assignments]
        with self._lock:
            self._stop_event.clear()
            self._active_command_id = goal.command_id
            self._active_mission_id = goal.mission_id
            self._active_command = goal.command
            self._active_uavs = list(uav_ids)

        self._publish_state(
            mission_id=goal.mission_id,
            command_id=goal.command_id,
            command=goal.command,
            tcp_status="PENDING",
            ros_status="GOAL_RECEIVED",
            detail="mock backend received goal",
            active_uavs=uav_ids,
        )

        if policy.accept_delay_s > 0.0 and not self._wait_interruptible(policy.accept_delay_s):
            self._set_preempted(goal, "goal interrupted before ACCEPTED")
            return

        if policy.send_accepted_feedback:
            feedback = ExecuteGroupTaskFeedback()
            feedback.phase = "ACCEPTED"
            feedback.active_uavs = list(uav_ids)
            feedback.detail = "mock backend accepted command"
            self.server.publish_feedback(feedback)
            self._publish_state(
                mission_id=goal.mission_id,
                command_id=goal.command_id,
                command=goal.command,
                tcp_status="ACCEPTED",
                ros_status="RUNNING",
                detail="mock backend accepted command",
                active_uavs=uav_ids,
            )

        running_feedback = ExecuteGroupTaskFeedback()
        running_feedback.phase = "RUNNING"
        running_feedback.active_uavs = list(uav_ids)
        running_feedback.detail = "mock backend executing command"
        self.server.publish_feedback(running_feedback)

        final_status = policy.final_status.upper()
        if final_status == "TIMEOUT":
            while not rospy.is_shutdown():
                if self.server.is_preempt_requested() or self._stop_event.is_set():
                    self._set_preempted(goal, "mock timeout interrupted by hold or cancel")
                    return
                rospy.sleep(0.1)
            return

        if policy.result_delay_s > 0.0 and not self._wait_interruptible(policy.result_delay_s):
            self._set_preempted(goal, "goal interrupted before terminal state")
            return

        result = ExecuteGroupTaskResult()
        result.message = policy.message or "mock task finished"
        result.error_code = policy.error_code

        if final_status == "FAILED":
            result.final_status = "FAILED"
            result.failed_uavs = list(policy.failed_uavs or uav_ids)
            self.server.set_aborted(result, text=result.message)
            self._publish_state(
                mission_id=goal.mission_id,
                command_id=goal.command_id,
                command=goal.command,
                tcp_status="FAILED",
                ros_status="FAILED",
                detail=result.message,
                failed_uavs=result.failed_uavs,
            )
            self._clear_active()
            return

        result.final_status = "COMPLETED"
        result.completed_uavs = list(uav_ids)
        self.server.set_succeeded(result, text=result.message)
        self._publish_state(
            mission_id=goal.mission_id,
            command_id=goal.command_id,
            command=goal.command,
            tcp_status="COMPLETED",
            ros_status="COMPLETED",
            detail=result.message,
            completed_uavs=result.completed_uavs,
        )
        self._clear_active()

    def _handle_hold(self, request) -> EmergencyHoldResponse:
        with self._lock:
            self._stop_event.set()
            mission_id = self._active_mission_id or request.mission_id
            command_id = self._active_command_id or "hold-local"
            command = self._active_command or "HOLD"
            active_uavs = list(self._active_uavs or request.uav_ids)

        self._publish_state(
            mission_id=mission_id,
            command_id=command_id,
            command=command,
            tcp_status="COMPLETED",
            ros_status="HELD",
            detail=request.reason or "emergency hold triggered",
            completed_uavs=active_uavs,
        )
        return EmergencyHoldResponse(
            accepted=True,
            message=request.reason or "emergency hold triggered",
        )

    def _wait_interruptible(self, seconds: float) -> bool:
        end_time = rospy.Time.now().to_sec() + seconds
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < end_time:
            if self.server.is_preempt_requested() or self._stop_event.is_set():
                return False
            rospy.sleep(0.05)
        return not self.server.is_preempt_requested() and not self._stop_event.is_set()

    def _set_preempted(self, goal, message: str) -> None:
        result = ExecuteGroupTaskResult()
        result.final_status = "CANCELLED"
        result.failed_uavs = [item.uav_id for item in goal.assignments]
        result.error_code = "MOCK_PREEMPTED"
        result.message = message
        self.server.set_preempted(result, text=message)
        self._publish_state(
            mission_id=goal.mission_id,
            command_id=goal.command_id,
            command=goal.command,
            tcp_status="FAILED",
            ros_status="PREEMPTED",
            detail=message,
            failed_uavs=result.failed_uavs,
        )
        self._clear_active()

    def _policy_for(self, command: str) -> MockPolicy:
        merged = dict(self.defaults)
        merged.update(self.command_overrides.get(command, {}))
        return MockPolicy(
            accept_delay_s=float(merged.get("accept_delay_s", 0.1)),
            result_delay_s=float(merged.get("result_delay_s", 0.5)),
            send_accepted_feedback=bool(merged.get("send_accepted_feedback", True)),
            final_status=str(merged.get("final_status", "COMPLETED")),
            failed_uavs=list(merged.get("failed_uavs", [])),
            error_code=str(merged.get("error_code", "MOCK_BACKEND")),
            message=str(merged.get("message", "mock task finished")),
        )

    def _publish_state(
        self,
        mission_id: str,
        command_id: str,
        command: str,
        tcp_status: str,
        ros_status: str,
        detail: str,
        active_uavs=None,
        completed_uavs=None,
        failed_uavs=None,
    ) -> None:
        msg = GroupTaskState()
        msg.mission_id = mission_id
        msg.command_id = command_id
        msg.command = command
        msg.tcp_status = tcp_status
        msg.ros_status = ros_status
        msg.active_uavs = list(active_uavs or [])
        msg.completed_uavs = list(completed_uavs or [])
        msg.failed_uavs = list(failed_uavs or [])
        msg.detail = detail
        self.state_pub.publish(msg)

    def _clear_active(self) -> None:
        with self._lock:
            self._stop_event.clear()
            self._active_command_id = ""
            self._active_mission_id = ""
            self._active_command = ""
            self._active_uavs = []


def main() -> None:
    rospy.init_node("mock_backend_node", anonymous=False)
    MockBackendNode()
    rospy.spin()


if __name__ == "__main__":
    main()
