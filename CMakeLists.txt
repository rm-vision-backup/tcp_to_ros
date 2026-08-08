cmake_minimum_required(VERSION 3.0.2)
project(tcp_to_ros)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  rospy
  std_msgs
  swarm_uav_interfaces
)

catkin_python_setup()

add_message_files(
  FILES
  GroupTaskState.msg
)

add_service_files(
  FILES
  EmergencyHold.srv
)

add_action_files(
  FILES
  ExecuteGroupTask.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  swarm_uav_interfaces
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    message_runtime
    rospy
    std_msgs
    swarm_topology_bridge
    swarm_uav_interfaces
)

catkin_install_python(PROGRAMS
  scripts/tcp_server_node.py
  scripts/mock_backend_node.py
  scripts/task_event_injector.py
  scripts/gcs_a_backend_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_gcs_a_aggregation.py)
  catkin_add_nosetests(test/test_tcp_disconnect_safety.py)
endif()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
