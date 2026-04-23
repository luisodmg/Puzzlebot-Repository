"""
mini_challenge.launch.py
Launch the path_generator + waypoint_follower together.

Usage (Gazebo):
    ros2 launch puzzlebot_control mini_challenge.launch.py use_sim_time:=true

Usage (real robot):
    ros2 launch puzzlebot_control mini_challenge.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ── 3-Waypoint path ───────────────────────────────────────────────────────
    # P1: 0.50 m straight ahead  in 3 s
    # P2: turn left, 0.50 m up   in 3 s
    # P3: turn left, 0.50 m back in 3 s
    waypoints = [
        0.50, 0.00, 3.0,
        0.00, 0.50, 3.0,
        0.00, 0.50, 3.0,
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='puzzlebot_control',
            executable='path_generator',
            name='path_generator',
            parameters=[{
                'use_sim_time': sim_time,
                'waypoints': waypoints,
            }],
            output='screen',
        ),

        Node(
            package='puzzlebot_control',
            executable='waypoint_follower',
            name='waypoint_follower',
            parameters=[{
                'use_sim_time': sim_time,
                'default_linear_speed':  0.20,
                'default_angular_speed': 0.50,
            }],
            output='screen',
        ),
    ])
