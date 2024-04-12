from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xsturret_robot_description_launch_arguments,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Interbotix turret
    turret_simulation_launch_arg = LaunchConfiguration('turret_simulation')
    turret_model_launch_arg = LaunchConfiguration('turret_model')
    turret_name_launch_arg = LaunchConfiguration('turret_name')
    turret_description_launch_arg = LaunchConfiguration('turret_description')

    # Payload
    payload_name_launch_arg = LaunchConfiguration('payload_name')

    # RViz
    use_rviz_launch_arg = LaunchConfiguration('assembly_use_rviz')
    rviz_config_launch_arg = LaunchConfiguration('rviz_config')
    use_joint_pub_gui_launch_arg = LaunchConfiguration('use_joint_pub_gui')

    # Retrieve launch configuration values as strings
    turret_name = turret_name_launch_arg.perform(context)
    payload_name = payload_name_launch_arg.perform(context)

    # URDF turret
    xsturret_description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsturret_descriptions'),
                'launch',
                'xsturret_description.launch.py'
            ])
        ]),
        condition=IfCondition(turret_simulation_launch_arg),
        launch_arguments={
            'robot_model': turret_model_launch_arg,
            'robot_name': turret_name_launch_arg,
            'robot_description': turret_description_launch_arg,
            'use_rviz': 'false',
            'use_joint_pub': 'false',
            'use_joint_pub_gui': use_joint_pub_gui_launch_arg,
        }.items(),
    )

    # URDF payload
    payload_description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_turret_aim_control'),
                'launch',
                'descriptions',
                'payload_description.launch.py',
            ])
        ]),
        launch_arguments={
            'robot_name': payload_name_launch_arg,
            'use_joint_pub_gui': use_joint_pub_gui_launch_arg,
        }.items(),
    )

    # Attach payload to turret
    static_transform_publisher_node = Node(
        package="tf2_ros",
        executable='static_transform_publisher',
        name='static_transform_publisher',
        namespace=payload_name,
        arguments=['--x', '0',
                   '--y', '0',
                   '--z', '0',
                   '--frame-id', f'{turret_name}/surface_link',
                   '--child-frame-id', f'{payload_name}/base_link'],
        output='log',
    )

    rviz2_node = Node(
        condition=IfCondition(use_rviz_launch_arg),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_launch_arg],
        output='screen',
    )

    return [
        xsturret_description_launch_include,
        payload_description_launch_include,
        static_transform_publisher_node,
        rviz2_node,
    ]


def generate_launch_description():
    declared_arguments = []

    # Turret
    declared_arguments.append(
        DeclareLaunchArgument(
            'turret_simulation',
            default_value='true',
            choices={'true', 'false'},
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'turret_model',
            default_value='pxxls',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'turret_name',
            default_value='pxxls',
        )
    )
    declared_arguments.extend(
        declare_interbotix_xsturret_robot_description_launch_arguments(
            robot_description_launch_config_name='turret_description',
            robot_model_launch_config_name='turret_model',
            robot_name_launch_config_name='turret_name',
            base_link_frame='base_link',
            use_world_frame='false',
        )
    )

    # Payload
    declared_arguments.append(
        DeclareLaunchArgument(
            'payload_name',
            default_value='payload',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'assembly_use_rviz',
            default_value='true',
            choices={'true', 'false'},
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_turret_aim_control'),
                'test',
                'rviz',
                'assembly.rviz',
            ]),
            description="the file path to the 'robot config' YAML file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_joint_pub_gui',
            default_value='true',
            choices={'true', 'false'},
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
