#!/usr/bin/env python3
"""Small helper node for triggering emergency hold during manual testing."""

from __future__ import annotations

import rospy

from tcp_to_ros.srv import EmergencyHold, EmergencyHoldRequest


def main() -> None:
    rospy.init_node("task_event_injector", anonymous=True)
    service_name = rospy.get_param("~emergency_hold_service", "/group_a/emergency_hold")
    mission_id = rospy.get_param("~mission_id", "manual-test")
    uav_ids = rospy.get_param("~uav_ids", [])
    reason = rospy.get_param("~reason", "manual hold injection")

    rospy.loginfo("waiting for service %s", service_name)
    rospy.wait_for_service(service_name)
    proxy = rospy.ServiceProxy(service_name, EmergencyHold)
    response = proxy(
        EmergencyHoldRequest(
            mission_id=mission_id,
            uav_ids=list(uav_ids),
            reason=reason,
        )
    )
    rospy.loginfo("hold response accepted=%s message=%s", response.accepted, response.message)


if __name__ == "__main__":
    main()
