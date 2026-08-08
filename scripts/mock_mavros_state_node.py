#!/usr/bin/env python3
"""Configurable MAVROS State heartbeat for safety integration tests only."""
import rospy
from mavros_msgs.msg import State


def main():
    rospy.init_node("mock_mavros_state")
    topic = rospy.get_param("~topic", "/mavros/state")
    publisher = rospy.Publisher(topic, State, queue_size=10)
    rate = rospy.Rate(float(rospy.get_param("~rate_hz", 10.0)))
    while not rospy.is_shutdown():
        publisher.publish(State(
            connected=bool(rospy.get_param("~connected", True)),
            armed=bool(rospy.get_param("~armed", False)),
            mode=rospy.get_param("~mode", "MANUAL"),
        ))
        rate.sleep()


if __name__ == "__main__":
    main()