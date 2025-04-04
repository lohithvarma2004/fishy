import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetLaunchConfiguration,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    ExecuteProcess,
    RegisterEventHandler
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
import xacro
from launch.event_handlers.on_shutdown import OnShutdown

def generate_launch_description():
    # Declare the sigterm_timeout launch argument with a default value of 30 seconds
    declare_sigterm_timeout = DeclareLaunchArgument(
        'sigterm_timeout',
        default_value='30',
        description='Time to wait after SIGINT before escalating to SIGTERM'
    )

    # Set the sigterm_timeout configuration
    set_sigterm_timeout = SetLaunchConfiguration(
        'sigterm_timeout', '30'
    )

    # Set GAZEBO_PLUGIN_PATH to include required plugins
    set_gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=os.path.join(get_package_share_directory('uuv_gazebo_plugins'), 'lib')
    )

    # Set GAZEBO_MODEL_PATH to include necessary model directories
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(get_package_share_directory('uuv_gazebo_worlds'), 'models')
    )

    # Define package and file paths
    package_name = 'fish_hpurv'
    xacro_file = 'urdf/fish_hpurv_macro.urdf.xacro'
    world_file = 'worlds/ocean_waves.world'

    # Build full paths
    model_file_path = os.path.join(get_package_share_directory(package_name), xacro_file)
    world_file_path = os.path.join(get_package_share_directory('uuv_gazebo_worlds'), world_file)

    # Process the xacro file to generate the URDF string.
    robot_description_str = xacro.process_file(model_file_path).toxml()
    robot_description = ParameterValue(robot_description_str, value_type=str)

    # Include the Gazebo launch file from gazebo_ros with needed arguments
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file_path,
            'extra_gazebo_args': '--verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so --shutdown_timeout 30'
        }.items()
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Spawn the robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'fish_hpurv', '-topic', 'robot_description'],
        output='screen'
    )

    load_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
                output='screen'
            )
        ]
    )
    load_joint_trajectory_controller = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
                output='screen'
            )
        ]
    )


    # Suppress ALSA warnings by setting environment variable
    suppress_alsa_warnings = SetEnvironmentVariable(
        name='ALSA_CONFIG_PATH',
        value='/etc/alsa/alsa.conf'
    )

    # Event handler to kill gzserver on shutdown
    kill_gzserver_event_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=['pkill', '-f', 'gzserver'],
                    shell=True,
                    output='screen'
                )
            ]
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_sigterm_timeout)
    ld.add_action(set_sigterm_timeout)
    ld.add_action(set_gazebo_plugin_path)
    ld.add_action(set_gazebo_model_path)
    ld.add_action(suppress_alsa_warnings)
    ld.add_action(gazebo_launch)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(kill_gzserver_event_handler)
    ld.add_action(load_joint_state_broadcaster)
    ld.add_action(load_joint_trajectory_controller)
    return ld
