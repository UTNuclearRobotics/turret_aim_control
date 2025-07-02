cmake_minimum_required(VERSION 3.8)
project(turret_aim_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turret_aim_control_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interbotix_xs_driver REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)
find_package(interbotix_ros_core REQUIRED)
find_package(interbotix_ros_turrets REQUIRED)

include_directories(
    include
)

set(ROS_DEPS
    rclcpp
    turret_aim_control_interfaces
    geometry_msgs
    sensor_msgs
    interbotix_xs_driver
    interbotix_xs_msgs
    interbotix_ros_core
    interbotix_ros_turrets
)

add_executable(turret_server src/turret_server.cpp)
ament_target_dependencies(turret_server ${ROS_DEPS})

install (TARGETS
    turret_server
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
