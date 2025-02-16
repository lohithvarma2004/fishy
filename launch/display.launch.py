import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # ✅ Open Gazebo and Spawn Your Robot
    return LaunchDescription([
        # Start Gazebo (Empty world for now)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ])
        ),

        # Convert Xacro to URDF
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

        # ✅ Spawn Robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'Fish_HPURV',
                '-topic', 'robot_description'
            ],
            output='screen'
        )
    ])

