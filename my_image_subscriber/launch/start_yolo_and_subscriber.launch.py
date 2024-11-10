#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node for yolo_publisher with the 'user' parameter set to 'theo'
    yolo_publisher_node = Node(
        package='my_image_subscriber',
        executable='yolo_publisher',
        name='yolo_publisher',
        output='screen',
        parameters=[{'user': 'theo'}]  # Passing the '--user theo' argument
    )

    # Node for image_subscriber
    image_subscriber_node = Node(
        package='my_image_subscriber',
        executable='image_subscriber',
        name='image_subscriber',
        output='screen'
    )

    return LaunchDescription([
        yolo_publisher_node,
        image_subscriber_node
    ])
