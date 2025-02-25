import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo with ocean world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'world': PathJoinSubstitution([
                    FindPackageShare('Fish_HPURV_description'),
                    'worlds',
                    'ocean.world'
                ])
            }.items()
        ),

        # Run robot_state_publisher (converts xacro to URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    FindExecutable(name='xacro'), ' ',
                    PathJoinSubstitution([
                        FindPackageShare('Fish_HPURV_description'),
                        'urdf',
                        'Fish_HPURV_macro.urdf.xacro'
                    ])
                ])
            }]
        ),

        # Delay the robot spawn by 60 seconds to let Gazebo fully load the world.
        TimerAction(
            period=60.0,  # delay in seconds
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'Fish_HPURV',
                        '-topic', 'robot_description'
                    ],
                    output='screen'
                )
            ]
        )
    ])
