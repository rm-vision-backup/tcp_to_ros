#!/usr/bin/env python3
"""Group A TCP server that bridges runtime commands into ROS actions."""

from __future__ import annotations

import json
import socket
from typing import Any, Dict, Optional

import actionlib
import rospy

from tcp_to_ros.msg import ExecuteGroupTaskAction, GroupTaskState
from tcp_to_ros.srv import EmergencyHold, EmergencyHoldRequest

from tcp_to_ros.bridge import (
    completed_uavs_from_result,
    failed_uavs_from_result,
    goal_from_command,
)
from tcp_to_ros.dedup import CommandCache
from tcp_to_ros.protocol import (
    Assignment,
    CloseConnectionError,
    CommandValidationError,
    GROUP_ID,
    MAX_MESSAGE_BYTES,
    ParsedCommand,
    build_accepted,
    build_completed,
    build_failed,
    parse_json_line,
)
from tcp_to_ros.server import (
    JsonLineConnection,
    create_listener,
    linux_port_is_ephemeral_and_unreserved,
)
from tcp_to_ros.state import ActiveTaskRegistry


class TcpToRosServerNode:
    def __init__(self) -> None:
        self.host = rospy.get_param("~host", "0.0.0.0")
        self.port = int(rospy.get_param("~port", 39001))
        self.action_name = rospy.get_param("~action_name", "/group_a/execute_task")
        self.state_topic = rospy.get_param("~state_topic", "/group_a/group_task_state")
        self.emergency_service_name = rospy.get_param(
            "~emergency_hold_service",
            "/group_a/emergency_hold",
        )

        # Claim the externally visible runtime port before waiting for ROS
        # dependencies. ROS nodes use random XML-RPC ports and may otherwise
        # consume 39001 while this node is blocked in wait_for_server().
        self.listener = create_listener(self.host, self.port)
        try:
            if linux_port_is_ephemeral_and_unreserved(self.port):
                rospy.logwarn(
                    "TCP port %d is inside Linux ip_local_port_range and is not "
                    "reserved; add it to net.ipv4.ip_local_reserved_ports on the "
                    "GCS host to prevent ROS XML-RPC port collisions",
                    self.port,
                )
            self._connect_ros_backend()
        except Exception:
            self.listener.close()
            raise

    def _connect_ros_backend(self) -> None:

        self.state_pub = rospy.Publisher(
            self.state_topic, GroupTaskState, queue_size=20
        )
        self.action_client = actionlib.SimpleActionClient(
            self.action_name, ExecuteGroupTaskAction
        )
        rospy.loginfo("waiting for action server %s", self.action_name)
        self.action_client.wait_for_server()
        rospy.loginfo("connected to action server %s", self.action_name)

        rospy.loginfo("waiting for service %s", self.emergency_service_name)
        rospy.wait_for_service(self.emergency_service_name)
        self.emergency_hold = rospy.ServiceProxy(
            self.emergency_service_name, EmergencyHold
        )

        self.cache = CommandCache()
        self.tasks = ActiveTaskRegistry()
        self.connection: Optional[JsonLineConnection] = None

    def run(self) -> None:
        rospy.loginfo("tcp_to_ros listening on %s:%d", self.host, self.port)
        while not rospy.is_shutdown():
            try:
                connection, address = self.listener.accept()
            except socket.timeout:
                continue
            handler = JsonLineConnection(connection, address)
            connection.settimeout(0.5)
            self.connection = handler
            rospy.loginfo("accepted runtime connection from %s", address)
            try:
                self._serve_connection(handler)
            finally:
                self._handle_disconnect("ground station connection closed")
                handler.close()
                self.connection = None

    def _serve_connection(self, handler: JsonLineConnection) -> None:
        buffer = bytearray()
        while not rospy.is_shutdown():
            try:
                chunk = handler.connection.recv(4096)
            except socket.timeout:
                continue
            except OSError as exc:
                rospy.logwarn("TCP receive failed: %s", exc)
                return
            if not chunk:
                return
            buffer.extend(chunk)
            if len(buffer) > MAX_MESSAGE_BYTES and b"\n" not in buffer:
                rospy.logwarn("peer sent a message larger than 64 KiB without newline")
                return
            while b"\n" in buffer:
                raw, _, remainder = buffer.partition(b"\n")
                buffer = bytearray(remainder)
                if not raw.strip():
                    continue
                try:
                    decoded = json.loads(raw.decode("utf-8"))
                    self._handle_message(decoded, handler)
                except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                    rospy.logwarn("invalid JSON line from runtime: %s", exc)
                    return
                except CloseConnectionError as exc:
                    rospy.logwarn("closing runtime connection: %s", exc)
                    return

    def _handle_message(self, decoded: Any, handler: JsonLineConnection) -> None:
        try:
            command = parse_json_line(decoded)
        except CommandValidationError as exc:
            partial = self._partial_command(decoded)
            if partial is None:
                raise CloseConnectionError(str(exc))
            failed_uavs = exc.failed_uavs or partial.uav_ids
            payload = build_failed(
                partial,
                failed_uavs,
                error_code=exc.error_code,
                message=str(exc),
            )
            self.cache.set_final(partial.command_id, payload)
            handler.send_json(payload)
            self._publish_state(
                partial,
                tcp_status="FAILED",
                ros_status="REJECTED",
                detail=str(exc),
                failed_uavs=failed_uavs,
            )
            return

        cached = self.cache.get(command.command_id)
        if cached is not None:
            handler.send_json(cached.final_response)
            return

        active = self.tasks.current()
        if command.command == "HOLD":
            self._handle_hold(command, handler)
            return
        if active is not None and active.command.command_id == command.command_id:
            handler.send_json(build_accepted(command, "duplicate active command"))
            return
        if active is not None:
            payload = build_failed(
                command,
                command.uav_ids,
                error_code="COMMAND_ALREADY_ACTIVE",
                message="another group command is already running",
            )
            self.cache.set_final(command.command_id, payload)
            handler.send_json(payload)
            self._publish_state(
                command,
                tcp_status="FAILED",
                ros_status="REJECTED",
                detail="another group command is already running",
                failed_uavs=command.uav_ids,
            )
            return

        task = self.tasks.start(command)
        goal = goal_from_command(command)
        try:
            self.action_client.send_goal(
                goal,
                done_cb=lambda status, result, serial=task.goal_serial: self._on_done(
                    serial, status, result
                ),
                feedback_cb=lambda feedback, serial=task.goal_serial: self._on_feedback(
                    serial, feedback
                ),
            )
        except Exception as exc:
            self.tasks.finish(task.goal_serial)
            payload = build_failed(
                command,
                command.uav_ids,
                error_code="ACTION_SEND_FAILED",
                message=str(exc),
            )
            self.cache.set_final(command.command_id, payload)
            handler.send_json(payload)
            self._publish_state(
                command,
                tcp_status="FAILED",
                ros_status="SEND_FAILED",
                detail=str(exc),
                failed_uavs=command.uav_ids,
            )
            return

        self._publish_state(
            command,
            tcp_status="PENDING",
            ros_status="GOAL_SENT",
            detail="goal forwarded to ROS action server",
            active_uavs=command.uav_ids,
        )

    def _handle_hold(
        self, command: ParsedCommand, handler: JsonLineConnection
    ) -> None:
        active = self.tasks.suppress_current()
        if active is not None:
            self.action_client.cancel_all_goals()
            original_payload = build_failed(
                active.command,
                active.command.uav_ids,
                error_code="COMMAND_HELD",
                message=command.reason or "task stopped by HOLD",
            )
            self.cache.set_final(active.command.command_id, original_payload)

        # 协议 §7/§4：HOLD 属安全命令，先回 ACCEPTED（表示已接管并执行安全悬停）。
        accepted_payload = build_accepted(command, "hold accepted; group safety brake-hold in progress")
        handler.send_json(accepted_payload)
        self._publish_state(
            command,
            tcp_status="ACCEPTED",
            ros_status="HELD",
            detail="hold accepted; group safety brake-hold in progress",
            active_uavs=command.uav_ids,
        )

        try:
            response = self.emergency_hold(
                EmergencyHoldRequest(
                    mission_id=command.mission_id,
                    uav_ids=command.uav_ids,
                    reason=command.reason or "runtime requested safe hold",
                )
            )
        except rospy.ServiceException as exc:
            payload = build_failed(
                command,
                command.uav_ids,
                error_code="HOLD_SERVICE_FAILED",
                message=str(exc),
            )
            self.cache.set_final(command.command_id, payload)
            handler.send_json(payload)
            self._publish_state(
                command,
                tcp_status="FAILED",
                ros_status="HOLD_FAILED",
                detail=str(exc),
                failed_uavs=command.uav_ids,
            )
            return

        if response.accepted:
            payload = build_completed(
                command,
                command.uav_ids,
                message=response.message or "hold completed",
            )
            self.cache.set_final(command.command_id, payload)
            handler.send_json(payload)
            self._publish_state(
                command,
                tcp_status="COMPLETED",
                ros_status="HELD",
                detail=response.message or "hold completed",
                completed_uavs=command.uav_ids,
            )
            return

        # 部分/全部参与者未确认安全锁存：诚实回 FAILED（含未确认任务层 ID），
        # 不能以 COMPLETED 掩盖未完成的安全悬停。
        failed_uavs = list(response.unconfirmed_uavs) or list(command.uav_ids)
        payload = build_failed(
            command,
            failed_uavs,
            error_code="HOLD_UNCONFIRMED",
            message=response.message or "hold not confirmed",
        )
        self.cache.set_final(command.command_id, payload)
        handler.send_json(payload)
        self._publish_state(
            command,
            tcp_status="FAILED",
            ros_status="HOLD_FAILED",
            detail=response.message or "hold not confirmed",
            failed_uavs=failed_uavs,
        )

    def _on_feedback(self, goal_serial: int, feedback) -> None:
        task = self.tasks.current()
        if task is None or task.goal_serial != goal_serial:
            return

        phase = str(feedback.phase or "").upper()
        detail = str(feedback.detail or "")
        active_uavs = list(feedback.active_uavs or task.command.uav_ids)
        if phase == "ACCEPTED" and not task.accepted_sent:
            self.tasks.mark_accepted(task.command.command_id)
            payload = build_accepted(task.command, detail or "command accepted")
            self._safe_send(payload)
            self._publish_state(
                task.command,
                tcp_status="ACCEPTED",
                ros_status="RUNNING",
                detail=detail or "command accepted",
                active_uavs=active_uavs,
            )
            return

        self._publish_state(
            task.command,
            tcp_status="RUNNING" if task.accepted_sent else "PENDING",
            ros_status=phase or "RUNNING",
            detail=detail,
            active_uavs=active_uavs,
        )

    def _on_done(self, goal_serial: int, _status: int, result) -> None:
        task = self.tasks.finish(goal_serial)
        if task is None or task.suppressed:
            return

        command = task.command
        if not task.accepted_sent:
            accepted = build_accepted(command, "command accepted")
            self._safe_send(accepted)
            self._publish_state(
                command,
                tcp_status="ACCEPTED",
                ros_status="RUNNING",
                detail="command accepted",
                active_uavs=command.uav_ids,
            )

        final_status = str(getattr(result, "final_status", "") or "").upper()
        if final_status in {"COMPLETED", "SUCCESS", "SUCCEEDED"}:
            completed_uavs = completed_uavs_from_result(
                command.uav_ids, getattr(result, "completed_uavs", [])
            )
            payload = build_completed(
                command,
                completed_uavs,
                message=str(getattr(result, "message", "") or "task completed"),
            )
            self.cache.set_final(command.command_id, payload)
            self._safe_send(payload)
            self._publish_state(
                command,
                tcp_status="COMPLETED",
                ros_status=final_status or "COMPLETED",
                detail=str(getattr(result, "message", "") or "task completed"),
                completed_uavs=completed_uavs,
            )
            return

        failed_uavs = failed_uavs_from_result(
            command.uav_ids, getattr(result, "failed_uavs", [])
        )
        payload = build_failed(
            command,
            failed_uavs,
            error_code=str(getattr(result, "error_code", "") or "BOTTOM_TASK_FAILED"),
            message=str(getattr(result, "message", "") or "task failed"),
        )
        self.cache.set_final(command.command_id, payload)
        self._safe_send(payload)
        self._publish_state(
            command,
            tcp_status="FAILED",
            ros_status=final_status or "FAILED",
            detail=str(getattr(result, "message", "") or "task failed"),
            failed_uavs=failed_uavs,
        )

    def _safe_send(self, payload: Dict[str, Any]) -> None:
        if self.connection is None:
            return
        try:
            self.connection.send_json(payload)
        except Exception as exc:
            rospy.logwarn("failed to send TCP status: %s", exc)

    def _handle_disconnect(self, reason: str) -> None:
        active = self.tasks.suppress_current()
        if active is None:
            return
        self.action_client.cancel_all_goals()
        try:
            self.emergency_hold(
                EmergencyHoldRequest(
                    mission_id=active.command.mission_id,
                    uav_ids=active.command.uav_ids,
                    reason=reason,
                )
            )
        except rospy.ServiceException as exc:
            rospy.logwarn("emergency hold on disconnect failed: %s", exc)
        self.cache.set_final(
            active.command.command_id,
            build_failed(
                active.command,
                active.command.uav_ids,
                error_code="CONNECTION_CLOSED",
                message=reason,
            ),
        )
        self._publish_state(
            active.command,
            tcp_status="FAILED",
            ros_status="DISCONNECTED",
            detail=reason,
            failed_uavs=active.command.uav_ids,
        )

    def _partial_command(self, decoded: Any) -> Optional[ParsedCommand]:
        if not isinstance(decoded, dict):
            return None
        mission_id = decoded.get("mission_id")
        command_id = decoded.get("command_id")
        command = decoded.get("command")
        if not isinstance(mission_id, str) or not mission_id:
            return None
        if not isinstance(command_id, str) or not command_id:
            return None
        if not isinstance(command, str) or not command:
            return None
        assignments = []
        for item in decoded.get("assignments", []):
            if isinstance(item, dict):
                uav_id = item.get("uav_id")
                if isinstance(uav_id, str) and uav_id:
                    assignments.append(Assignment(uav_id=uav_id))
        if not assignments:
            return None
        timeout_s = decoded.get("timeout_s", 1.0)
        try:
            timeout_s = float(timeout_s)
        except (TypeError, ValueError):
            timeout_s = 1.0
        return ParsedCommand(
            mission_id=mission_id,
            group_id=GROUP_ID,
            command_id=command_id,
            command=command.upper(),
            timeout_s=timeout_s if timeout_s > 0 else 1.0,
            assignments=assignments,
        )

    def _publish_state(
        self,
        command: ParsedCommand,
        tcp_status: str,
        ros_status: str,
        detail: str,
        active_uavs=None,
        completed_uavs=None,
        failed_uavs=None,
    ) -> None:
        msg = GroupTaskState()
        msg.mission_id = command.mission_id
        msg.command_id = command.command_id
        msg.command = command.command
        msg.tcp_status = tcp_status
        msg.ros_status = ros_status
        msg.active_uavs = list(active_uavs or [])
        msg.completed_uavs = list(completed_uavs or [])
        msg.failed_uavs = list(failed_uavs or [])
        msg.detail = detail
        self.state_pub.publish(msg)


def main() -> None:
    rospy.init_node("tcp_server_node", anonymous=False)
    node = TcpToRosServerNode()
    try:
        node.run()
    finally:
        try:
            node.listener.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
