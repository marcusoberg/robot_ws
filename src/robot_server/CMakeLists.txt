cmake_minimum_required(VERSION 3.22)
project(robot_server)
if(APPLE)
  set(CMAKE_INSTALL_RPATH "@loader_path;@loader_path/../../..")
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointGoal.msg" "msg/Status.msg" "msg/Observation.msg" "msg/Control.msg"
  DEPENDENCIES sensor_msgs)
if(APPLE)
  # ROS dispatches these libraries by basename. Declare their native dependencies
  # so dyld resolves them through CMake's rpaths before middleware dispatch.
  foreach(language c cpp)
    target_link_libraries(${PROJECT_NAME}__rosidl_typesupport_${language} PRIVATE
      ${PROJECT_NAME}__rosidl_typesupport_fastrtps_${language}
      ${PROJECT_NAME}__rosidl_typesupport_introspection_${language})
  endforeach()
endif()
ament_python_install_package(robot_server_node)
ament_python_install_package(robot_server_core)
install(PROGRAMS scripts/server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch foxglove DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
