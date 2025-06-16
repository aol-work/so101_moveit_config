#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        description='Path to the ROS bag directory'
    )

    topic_arg = DeclareLaunchArgument(
        'topic',
        default_value='/pose_stamped',
        description='Topic name containing PoseStamped messages'
    )

    # Create the IK processor node
    ik_processor_node = Node(
        package='ik_bag_processor',
        executable='ik_processor',
        name='ik_processor',
        parameters=[{
            'bag_path': LaunchConfiguration('bag_path'),
            'topic': LaunchConfiguration('topic'),
        }],
        arguments=[
            LaunchConfiguration('bag_path'),
            '--topic', LaunchConfiguration('topic')
        ],
        output='screen'
    )

    return LaunchDescription([
        bag_path_arg,
        topic_arg,
        ik_processor_node,
    ])
