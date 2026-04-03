cmake_minimum_required(VERSION 3.8)
project(sb_base)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Heartbeat.msg"
    "msg/ProxAlert.msg"
    "msg/Report.msg"
    "msg/TaskCommand.msg"
)

add_library(sb_base_lib SHARED
    src/drone.cpp
    src/ground.cpp
)

target_include_directories(sb_base_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sb_base_lib PUBLIC
    "rclcpp"
    "px4_msgs"
    "yaml-cpp"
    "Eigen3"
)
rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp
)
target_link_libraries(sb_base_lib PUBLIC
    ${cpp_typesupport_target}
    yaml-cpp
    Eigen3::Eigen
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS sb_base_lib
  EXPORT sb_base_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# launch
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(sb_base_targets
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
    rclcpp
    px4_msgs
    rosidl_default_runtime
    yaml-cpp
    Eigen3
)

ament_package()
