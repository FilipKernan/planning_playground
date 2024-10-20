from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    pkg_share = os.path.join(get_package_share_directory('your_package'))

    # Launch arguments
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_share, 'worlds', 'world.sdf'),
            description='SDF world file'),

        # Start Gazebo with the world
        Node(
            package='gazebo_ros',
            executable='gzserver',
            output='screen',
            arguments=[LaunchConfiguration('world')]
        ),
        Node(
            package='gazebo_ros',
            executable='gzclient',
            output='screen',
        ),

        # Launch the costmap server
        Node(
            package='nav2_costmap_2d',
            executable='costmap_2d',
            name='global_costmap',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'costmap.yaml')],
        ),
    ])

