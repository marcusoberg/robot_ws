cmake_minimum_required(VERSION 3.22)
project(robot_server)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Observation.msg" "msg/Capture.msg" "msg/CapturedImage.msg" "msg/Event.msg"
  "msg/Heartbeat.msg" "msg/MotionCommand.msg" "msg/CommandResult.msg"
  "srv/Control.srv" "srv/Step.srv" "srv/Reset.srv" "srv/CaptureOnce.srv"
  DEPENDENCIES geometry_msgs trajectory_msgs sensor_msgs)
# Keep executable Python separate from the generated robot_server.msg namespace.
ament_python_install_package(robot_server_node)
ament_python_install_package(robot_server_client)
install(PROGRAMS scripts/server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config assets foxglove DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
