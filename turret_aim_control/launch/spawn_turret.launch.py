import yaml
from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xsturret_robot_description_launch_arguments,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Load turret configs
    robot_configs_path = LaunchConfiguration('robot_configs').perform(context)
    with open(robot_configs_path, 'r') as file:
        config = yaml.safe_load(file)

    # Interbotix
    turret_description_launch_arg = LaunchConfiguration('turret_description')
    turret_logging_level_launch_arg = LaunchConfiguration(
        'xs_driver_logging_level')
    motor_configs_launch_arg = LaunchConfiguration('motor_configs')
    mode_configs_launch_arg = LaunchConfiguration('mode_configs')
    turret_controller_configs_launch_arg = LaunchConfiguration(
        'turret_controller_configs')

    # RViz
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    rviz_config_launch_arg = LaunchConfiguration('rviz_config')

    # URDF assembly of turret and payload description
    assembly_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turret_aim_control'),
                'launch',
                'descriptions',
                'assembly_description.launch.py'
            ])
        ]),
        launch_arguments={
            'turret_simulation': config['turret']['simulate'],
            'turret_model': config['turret']['model'],
            'turret_name': config['turret']['name'],
            'turret_description': turret_description_launch_arg,
            'payload_name': config['payload']['name'],
            'assembly_use_rviz': use_rviz_launch_arg,
            'rviz_config': rviz_config_launch_arg,
            'use_joint_pub_gui': 'false',
        }.items(),
    )

    # Interbotix turret control
    xsturret_control_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsturret_control'),
                'launch',
                'xsturret_control.launch.py'
            ])
        ]),
        condition=UnlessCondition(config['turret']['simulate']),
        launch_arguments={
            'robot_model': config['turret']['model'],
            'robot_name': config['turret']['name'],
            'robot_description': turret_description_launch_arg,
            'use_rviz': 'false',
            'motor_configs': motor_configs_launch_arg,
            'mode_configs': mode_configs_launch_arg,
            'load_configs': 'true',
            'use_sim_time': 'false',
            'xs_driver_logging_level': turret_logging_level_launch_arg,
        }.items(),
    )

    turret_controller_node = Node(
        package='turret_aim_control',
        executable='turret_controller',
        name=config['turret']['controller_name'],
        parameters=[
            turret_controller_configs_launch_arg,
            {
                'turret_simulate_joint_states':
                PythonExpression(
                    "True" if config['turret']['simulate'] == 'true' else "False")
            }
        ],
        output='log',
    )

    # Attach turret to a robot
    attach_turret_static_transform_publisher_node = Node(
        package="tf2_ros",
        executable='static_transform_publisher',
        name='static_transform_publisher',
        namespace=config['turret']['name'],
        arguments=[
            '--x', f"{config['turret_position']['x']}",
            '--y', f"{config['turret_position']['y']}",
            '--z', f"{config['turret_position']['z']}",
            '--frame-id', config['turret_position']['parent_frame'],
            '--child-frame-id', config['turret_position']['child_frame']
        ],
        output='log',
    )

    return [
        assembly_launch_include,
        xsturret_control_launch_include,
        turret_controller_node,
        attach_turret_static_transform_publisher_node,
    ]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_configs',
            default_value=PathJoinSubstitution([
                FindPackageShare('turret_aim_control'),
                'config',
                'default',
                'turret.yaml',
            ]),
            description="the file path to the 'turret config' YAML file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'turret_controller_configs',
            default_value=PathJoinSubstitution([
                FindPackageShare('turret_aim_control'),
                'config',
                'default',
                'turret_controller.yaml',
            ]),
            description="the file path to the 'turret controller config' YAML file.",
        )
    )

    # RViz
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('turret_aim_control'),
                'rviz',
                'spawn_turret.rviz',
            ]),
            description="the file path to the 'rviz config' YAML file.",
        )
    )

    # Interbotix
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
    declared_arguments.append(
        DeclareLaunchArgument(
            'xs_driver_logging_level',
            default_value='INFO',
            choices=('DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'),
            description='set the logging level of the X-Series Driver.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'motor_configs',
            default_value=PathJoinSubstitution([
                FindPackageShare('turret_aim_control'),
                'config',
                'default',
                'interbotix',
                'pxxls.yaml',
            ]),
            description="the file path to the 'motor config' YAML file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs',
            default_value=PathJoinSubstitution([
                FindPackageShare('turret_aim_control'),
                'config',
                'default',
                'interbotix',
                'modes.yaml',
            ]),
            description="the file path to the 'mode config' YAML file.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
