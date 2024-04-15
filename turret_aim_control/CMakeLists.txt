cmake_minimum_required(VERSION 3.8)
project(turret_aim_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turret_aim_control_interfaces REQUIRED)

include_directories(include)

add_executable(turret_controller src/turret_controller.cpp)
ament_target_dependencies(turret_controller
  geometry_msgs
  interbotix_xs_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  turret_aim_control_interfaces
)
target_link_libraries(turret_controller "${cpp_typesupport_target}")

add_executable(target src/target.cpp)
ament_target_dependencies(target
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)
target_link_libraries(target "${cpp_typesupport_target}")

install(TARGETS
  turret_controller
  target
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
