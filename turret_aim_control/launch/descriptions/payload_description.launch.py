from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration
)
from launch_ros.actions import Node

import os
import xacro
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Payload
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    use_joint_pub_gui_launch_arg = LaunchConfiguration('use_joint_pub_gui')

    # Retrieve launch configuration values as strings
    robot_name = robot_name_launch_arg.perform(context)

    # Load URDF
    urdf = os.path.join(
        get_package_share_directory("turret_aim_control"),
        "urdf",
        "payload.urdf.xacro"
    )
    robot_description = xacro.process_file(
        urdf, mappings={'robot_name': robot_name}).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        parameters=[
            {'robot_description': robot_description}
        ],
        output={'both': 'screen'},
    )
    
    joint_state_publisher_gui_node = Node(
        condition=IfCondition(use_joint_pub_gui_launch_arg),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=robot_name,
        output={'both': 'screen'},
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
    ]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value='payload',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_joint_pub_gui',
            default_value='true',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
