cmake_minimum_required(VERSION 3.8)
project(tracking)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sb_base REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(idrone src/idrone.cpp)
add_executable(iground src/iground.cpp)

target_link_libraries(idrone PUBLIC
  sb_base::sb_base_lib
  rclcpp::rclcpp
  px4_msgs::px4_msgs__rosidl_typesupport_cpp
)

target_link_libraries(iground PUBLIC
  sb_base::sb_base_lib
  rclcpp::rclcpp
  # px4_msgs::px4_msgs__rosidl_typesupport_cpp
)

install(TARGETS idrone
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS iground
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()