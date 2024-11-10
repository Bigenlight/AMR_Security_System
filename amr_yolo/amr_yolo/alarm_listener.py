#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading
import time
import os
from ament_index_python.packages import get_package_share_directory

class AlarmListener(Node):
    def __init__(self):
        super().__init__('alarm_listener')

        # Subscribe to the 'alarm' topic
        self.subscription = self.create_subscription(
            String,
            'alarm',
            self.alarm_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Variables to manage subprocesses
        self.nav_process = None
        self.yolo_process = None
        self.is_processing = False  # Flag to prevent multiple triggers

    def alarm_callback(self, msg):
        self.get_logger().info(f"Received alarm message: {msg.data}")

        if msg.data.lower() == 'car detected' and not self.is_processing:
            self.is_processing = True  # Prevent multiple triggers
            threading.Thread(target=self.start_launches).start()
        else:
            if msg.data.lower() != 'car detected':
                self.get_logger().info("Received non-relevant alarm message.")
            else:
                self.get_logger().info("Launch already in progress. Ignoring the message.")

    def start_launches(self):
        try:
            # Start the navigation launch file
            nav_command = [
                'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
                f'map:={self.get_map_path()}'
            ]
            self.get_logger().info("Starting navigation launch...")
            self.nav_process = subprocess.Popen(nav_command)
            self.get_logger().info("Navigation launch started.")

            # Wait for 10 seconds before starting the next launch file
            self.get_logger().info("Waiting for 10 seconds before starting YOLO tracking launch...")
            time.sleep(10)

            # Start the YOLO tracking launch file
            yolo_command = [
                'ros2', 'launch', 'amr_yolo', 'start_yolo_tracking_and_publish.launch.py'
            ]
            self.get_logger().info("Starting YOLO tracking launch...")
            self.yolo_process = subprocess.Popen(yolo_command)
            self.get_logger().info("YOLO tracking launch started.")

        except Exception as e:
            self.get_logger().error(f"Exception occurred while launching: {e}")
        # Do not reset is_processing to allow only one launch per node lifetime

    def get_map_path(self):
        # Retrieve the map path from a parameter or use a default path
        # You can declare a parameter or modify this method as needed
        default_map_path = os.path.expanduser('~/map.yaml')
        #default_map_path = '/home/rokey3/map.yaml'
        map_path = self.declare_parameter('map_path', default_map_path).value
        self.get_logger().info(f"Using map path: {map_path}")
        return map_path

    def destroy_node(self):
        # Clean up subprocesses if the node is destroyed
        if self.nav_process and self.nav_process.poll() is None:
            self.nav_process.terminate()
            self.get_logger().info("Terminated navigation process.")
        if self.yolo_process and self.yolo_process.poll() is None:
            self.yolo_process.terminate()
            self.get_logger().info("Terminated YOLO tracking process.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AlarmListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("AlarmListener node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
