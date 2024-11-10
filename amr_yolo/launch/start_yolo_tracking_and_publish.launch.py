#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node for yolo_tracking_publisher from my_image_subscriber package
    yolo_tracking_publisher_node = Node(
        package='my_image_subscriber',
        executable='yolo_tracking_publisher',
        name='yolo_tracking_publisher',
        output='screen',
        parameters=[
            # Add any necessary parameters here
            # Example: {'parameter_name': 'value'}
        ]
    )

    # Node for publish_initial_and_goal from turtlebot3_custom_publishers package
    publish_initial_and_goal_node = Node(
        package='turtlebot3_custom_publishers',
        executable='publish_initial_and_goal',
        name='publish_initial_and_goal',
        output='screen',
        parameters=[
            # Add any necessary parameters here
            # Example: {'parameter_name': 'value'}
        ]
    )

    return LaunchDescription([
        yolo_tracking_publisher_node,
        publish_initial_and_goal_node
    ])
