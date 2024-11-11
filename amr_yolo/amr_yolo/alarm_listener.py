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
        self.robot_process = None
        self.yolo_process = None
        self.is_processing = False  # Flag to prevent multiple triggers

        # Declare a parameter for the map path
        self.declare_parameter('map_path', os.path.expanduser('~/map.yaml'))

    def alarm_callback(self, msg):
        self.get_logger().info(f"Received alarm message: '{msg.data}'")

        if msg.data.lower() == 'car detected' and not self.is_processing:
            self.is_processing = True  # Prevent multiple triggers
            self.get_logger().info("Triggering launch of navigation, robot bringup, and YOLO tracking nodes.")
            threading.Thread(target=self.start_launches).start()
        else:
            if msg.data.lower() != 'car detected':
                self.get_logger().info("Received non-relevant alarm message.")
            else:
                self.get_logger().info("Launch already in progress. Ignoring the message.")

    def start_launches(self):
        try:
            # Retrieve the map path from parameters
            map_path = self.get_map_path()

            # Verify that the map file exists
            if not os.path.isfile(map_path):
                self.get_logger().error(f"Map file not found at: {map_path}")
                return

            # 1. Start the navigation launch file
            nav_command = [
                'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
                f'map:={map_path}'
            ]
            self.get_logger().info(f"Starting navigation launch with map: {map_path}")
            self.nav_process = subprocess.Popen(nav_command)
            self.get_logger().info("Navigation launch started successfully.")

            # 2. Start the robot bringup launch file immediately after navigation
            robot_command = [
                'ros2', 'launch', 'turtlebot3_bringup', 'robot.launch.py'
            ]
            self.get_logger().info("Starting robot bringup launch...")
            self.robot_process = subprocess.Popen(robot_command)
            self.get_logger().info("Robot bringup launch started successfully.")

            # 3. Wait for 10 seconds before starting the YOLO tracking launch file
            self.get_logger().info("Waiting for 10 seconds before starting YOLO tracking launch...")
            time.sleep(10)

            # 4. Start the YOLO tracking launch file
            yolo_command = [
                'ros2', 'launch', 'amr_yolo', 'start_yolo_tracking_and_publish.launch.py'
            ]
            self.get_logger().info("Starting YOLO tracking launch...")
            self.yolo_process = subprocess.Popen(yolo_command)
            self.get_logger().info("YOLO tracking launch started successfully.")

        except Exception as e:
            self.get_logger().error(f"Exception occurred while launching: {e}")
        # Do not reset is_processing to allow only one launch per node lifetime

    def get_map_path(self):
        # Get the map_path parameter
        map_path = self.get_parameter('map_path').value
        # Expand environment variables and user (~) in the path
        map_path = os.path.expandvars(os.path.expanduser(map_path))
        self.get_logger().info(f"Using map path: {map_path}")
        return map_path

    def destroy_node(self):
        # Clean up subprocesses if the node is destroyed
        if self.nav_process and self.nav_process.poll() is None:
            self.nav_process.terminate()
            self.get_logger().info("Terminated navigation process.")
        if self.robot_process and self.robot_process.poll() is None:
            self.robot_process.terminate()
            self.get_logger().info("Terminated robot bringup process.")
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
