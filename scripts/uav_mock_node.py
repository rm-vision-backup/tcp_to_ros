#!/usr/bin/env python3
"""Single-UAV mock execution adapter (onboard side).

Simulates the onboard ROS node for one UAV:

- Provides ``/{exec_target}/uav_task`` Service (accept/reject/idempotent register).
- Provides ``/{exec_target}/uav_hold`` Service (safe hold).
- Publishes ``/{exec_target}/uav_task_state`` Topic (ACCEPTED/COMPLETED/FAILED).

Idempotency:
- Unique key is (mission_id, command_id, uav_id).
- Duplicate task requests return the current cached state instead of re-starting.
- Repeated terminal states are republished for reliability.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Dict

import rospy

from swarm_uav_interfaces.msg import UavTaskState
from swarm_uav_interfaces.srv import UavHold, UavHoldRequest, UavHoldResponse
from swarm_uav_interfaces.srv import UavTask, UavTaskRequest, UavTaskResponse

PROTOCOL_VERSION = "1.0"

STATE_ACCEPTED = "ACCEPTED"
STATE_COMPLETED = "COMPLETED"
STATE_FAILED = "FAILED"


@dataclass
class TaskRecord:
    key: str
    mission_id: str
    command_id: str
    uav_id: str
    command: str
    status: str
    status_seq: int
    error_code: str = ""
    message: str = ""
    held: bool = False
    terminal: bool = False


@dataclass
class UavMockPolicy:
    accept_delay_s: float = 0.1
    result_delay_s: float = 0.5
    accept: bool = True
    final_status: str = "COMPLETED"
    error_code: str = ""
    message: str = "mock UAV task finished"
    publish_interval_s: float = 0.5


class UavMockNode:
    def __init__(self) -> None:
        self.exec_target = rospy.get_param("~exec_target", "UAV1")
        self.uav_id = rospy.get_param("~uav_id", "A01")
        self.task_service = rospy.get_param(
            "~task_service", "/{}/uav_task".format(self.exec_target)
        )
        self.hold_service = rospy.get_param(
            "~hold_service", "/{}/uav_hold".format(self.exec_target)
        )
        self.state_topic = rospy.get_param(
            "~state_topic", "/{}/uav_task_state".format(self.exec_target)
        )

        self.defaults = rospy.get_param("~defaults", {})
        self.command_overrides = rospy.get_param("~command_overrides", {})

        self.state_pub = rospy.Publisher(self.state_topic, UavTaskState, queue_size=50)

        self._lock = threading.RLock()
        self._tasks: Dict[str, TaskRecord] = {}
        self._stop_event = threading.Event()
        self._seq_counter = 0

        rospy.Service(self.task_service, UavTask, self._handle_task)
        rospy.Service(self.hold_service, UavHold, self._handle_hold)
        rospy.loginfo(
            "UAV mock %s ready: task=%s hold=%s state=%s",
            self.exec_target,
            self.task_service,
            self.hold_service,
            self.state_topic,
        )

    def _policy_for(self, command: str) -> UavMockPolicy:
        merged = dict(self.defaults)
        merged.update(self.command_overrides.get(command, {}))
        return UavMockPolicy(
            accept_delay_s=float(merged.get("accept_delay_s", 0.1)),
            result_delay_s=float(merged.get("result_delay_s", 0.5)),
            accept=bool(merged.get("accept", True)),
            final_status=str(merged.get("final_status", "COMPLETED")),
            error_code=str(merged.get("error_code", "")),
            message=str(merged.get("message", "mock UAV task finished")),
            publish_interval_s=float(merged.get("publish_interval_s", 0.5)),
        )

    def _handle_task(self, request: UavTaskRequest) -> UavTaskResponse:
        key = self._key(request.mission_id, request.command_id, request.uav_id)
        with self._lock:
            cached = self._tasks.get(key)
            if cached is not None:
                return self._cached_response(cached, "duplicate task request")
            policy = self._policy_for(request.command)

        if not policy.accept:
            rospy.logwarn("UAV %s rejecting task %s", self.exec_target, key)
            return self._build_response(
                accepted=False,
                status="REJECTED",
                error_code="UAV_REJECTED",
                message=policy.message or "UAV rejected task",
            )

        if policy.accept_delay_s > 0.0:
            end = time.time() + policy.accept_delay_s
            while time.time() < end:
                if self._stop_event.is_set():
                    return self._build_response(
                        accepted=False,
                        status="ABORTED",
                        error_code="UAV_STOPPED",
                        message="UAV stopped before accept",
                    )
                rospy.sleep(0.05)

        with self._lock:
            self._seq_counter += 1
            record = TaskRecord(
                key=key,
                mission_id=request.mission_id,
                command_id=request.command_id,
                uav_id=request.uav_id or self.uav_id,
                command=request.command,
                status=STATE_ACCEPTED,
                status_seq=self._seq_counter,
            )
            self._tasks[key] = record
        self._publish(record, detail="task accepted")

        thread = threading.Thread(
            target=self._run_execution,
            args=(key, policy),
            daemon=True,
        )
        thread.start()

        return self._build_response(
            accepted=True,
            status=STATE_ACCEPTED,
            message="task accepted and handed to execution entry",
        )

    def _run_execution(self, key: str, policy: UavMockPolicy) -> None:
        with self._lock:
            record = self._tasks.get(key)
            if record is None:
                return
        rospy.sleep(policy.result_delay_s)

        with self._lock:
            record = self._tasks.get(key)
            if record is None or record.held or record.terminal:
                return
            self._seq_counter += 1
            final_status = policy.final_status.upper()
            if final_status not in {STATE_COMPLETED, STATE_FAILED}:
                final_status = STATE_COMPLETED
            record.status = final_status
            record.status_seq = self._seq_counter
            record.error_code = policy.error_code
            record.message = policy.message
            record.terminal = True
        self._publish(
            record,
            detail=policy.message or "task terminal",
            extra_delay=policy.publish_interval_s,
        )

    def _handle_hold(self, request: UavHoldRequest) -> UavHoldResponse:
        key = self._key(request.mission_id, request.command_id, request.uav_id)
        with self._lock:
            record = self._tasks.get(key)
            if record is None:
                self._seq_counter += 1
                record = TaskRecord(
                    key=key,
                    mission_id=request.mission_id,
                    command_id=request.command_id,
                    uav_id=request.uav_id or self.uav_id,
                    command="HOLD",
                    status=STATE_ACCEPTED,
                    status_seq=self._seq_counter,
                    held=True,
                )
                self._tasks[key] = record
                self._publish(record, detail="hold accepted (unknown task)")
                return self._build_hold_response(True, "hold accepted")
            if record.terminal:
                return self._build_hold_response(
                    True, "task already terminal; hold accepted"
                )
            record.held = True
            self._seq_counter += 1
            record.status = STATE_FAILED
            record.status_seq = self._seq_counter
            record.error_code = "COMMAND_HELD"
            record.message = request.reason or "task held"
            record.terminal = True
        self._publish(record, detail=record.message)
        return self._build_hold_response(True, "hold accepted and task stopped")

    def _cached_response(self, record: TaskRecord, reason: str) -> UavTaskResponse:
        # Idempotent: return current state without re-starting execution.
        # Terminal states are republished briefly because the requester may
        # filter messages arriving before its group ACCEPTED is set.
        self._publish(
            record,
            detail=reason + "; state cached",
            extra_delay=self._policy_for(record.command).publish_interval_s,
        )
        return self._build_response(
            accepted=True,
            status=record.status,
            message=record.message or reason,
        )

    def _build_response(
        self,
        accepted: bool,
        status: str,
        error_code: str = "",
        message: str = "",
    ) -> UavTaskResponse:
        response = UavTaskResponse()
        response.accepted = accepted
        response.status = status
        response.error_code = error_code
        response.message = message
        return response

    def _build_hold_response(
        self,
        accepted: bool,
        message: str = "",
        error_code: str = "",
    ) -> UavHoldResponse:
        # UavHold.srv response has only accepted/error_code/message (no status).
        response = UavHoldResponse()
        response.accepted = accepted
        response.error_code = error_code
        response.message = message
        return response

    def _publish(
        self,
        record: TaskRecord,
        detail: str = "",
        extra_delay: float = 0.0,
    ) -> None:
        self.state_pub.publish(self._to_msg(record, detail))
        # Reliability: republish terminal states briefly so slow-joining
        # subscribers (including the bridge) still converge.
        if extra_delay > 0.0 and record.terminal:
            deadline = time.time() + extra_delay
            while time.time() < deadline and not self._stop_event.is_set():
                self.state_pub.publish(self._to_msg(record, record.message))
                rospy.sleep(0.2)

    def _to_msg(self, record: TaskRecord, detail: str = "") -> UavTaskState:
        msg = UavTaskState()
        msg.protocol_version = PROTOCOL_VERSION
        msg.mission_id = record.mission_id
        msg.command_id = record.command_id
        msg.uav_id = record.uav_id
        msg.exec_target = self.exec_target
        msg.command = record.command
        msg.status = record.status
        msg.status_seq = record.status_seq
        msg.detail_stage = record.status
        msg.error_code = record.error_code
        msg.message = detail or record.message
        msg.reported_at = time.time()
        return msg

    @staticmethod
    def _key(mission_id: str, command_id: str, uav_id: str) -> str:
        return f"{mission_id}|{command_id}|{uav_id}"


def main() -> None:
    rospy.init_node("uav_mock_node", anonymous=False)
    UavMockNode()
    rospy.spin()


if __name__ == "__main__":
    main()