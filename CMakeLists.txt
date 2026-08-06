cmake_minimum_required(VERSION 3.0.2)
project(tcp_to_ros)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  rospy
  std_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  GroupTaskState.msg
  Pose3DYaw.msg
  TaskAssignment.msg
  UavTaskState.msg
)

add_service_files(
  FILES
  EmergencyHold.srv
  UavHold.srv
  UavTask.srv
)

add_action_files(
  FILES
  ExecuteGroupTask.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    message_runtime
    rospy
    std_msgs
)

catkin_install_python(PROGRAMS
  scripts/tcp_server_node.py
  scripts/mock_backend_node.py
  scripts/task_event_injector.py
  scripts/gcs_a_backend_node.py
  scripts/uav_mock_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
