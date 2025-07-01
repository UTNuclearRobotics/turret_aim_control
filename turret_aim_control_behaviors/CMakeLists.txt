cmake_minimum_required(VERSION 3.8)
project(turret_aim_control_behaviors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)

option(BUILD_SHARED_LIBS "Build shared libraries instead of static" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turret_aim_control_interfaces REQUIRED)

include_directories(
    include
)

set(BEHAVIOR_TREE_DEPS 
    rclcpp
    behaviortree_cpp
    geometry_msgs
    turret_aim_control_interfaces
)

set(LIBRARY_TYPE STATIC)
if(BUILD_SHARED_LIBS)
    set(LIBRARY_TYPE SHARED)
endif()

add_library(turret_aim_control_behaviors ${LIBRARY_TYPE}
    src/aim_turret.cpp
    src/plugin.cpp
)
target_compile_definitions(turret_aim_control_behaviors PRIVATE BUILD_${LIBRARY_TYPE}_LIBRARY)
ament_target_dependencies(turret_aim_control_behaviors ${BEHAVIOR_TREE_DEPS})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${BEHAVIOR_TREE_DEPS})

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
