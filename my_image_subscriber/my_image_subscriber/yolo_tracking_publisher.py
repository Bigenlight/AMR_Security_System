# yolo_tracking_publisher.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
from nav2_msgs.action import FollowWaypoints  # Import FollowWaypoints action
from rclpy.action import ActionClient
from action_msgs.srv import CancelGoal  # Import CancelGoal service

class YOLOTrackingPublisher(Node):
    def __init__(self):
        super().__init__('yolo_tracking_publisher')
        self.image_publisher = self.create_publisher(Image, 'tracked_image', 10)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.model = YOLO('/home/rokey3/1_ws/src/best.pt')
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjust timer frequency as needed

        # Screen center x-coordinate
        self.screen_center_x = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) / 2)
        self.alignment_tolerance = 50  # Pixel tolerance for alignment

        # Create FollowWaypoints action client
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # Navigation cancellation status tracking variable
        self.navigation_cancelled = False

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame from webcam.")
            return

        # Run tracking and get results
        results = self.model.track(source=frame, show=False, tracker='bytetrack.yaml')

        highest_confidence_detection = None
        highest_confidence = 0.6  # Minimum confidence threshold

        # Iterate over results to find the object with the highest confidence and class_id == 0
        for result in results:
            for detection in result.boxes.data:
                if len(detection) >= 6:
                    x1, y1, x2, y2, confidence, class_id = detection[:6]
                    # Check if the detection meets the confidence and class ID criteria
                    if confidence > highest_confidence and int(class_id) == 0:
                        highest_confidence = confidence
                        highest_confidence_detection = (x1, y1, x2, y2, confidence, class_id)

        # If a detection is found with confidence > 0.6 and class_id == 0
        if highest_confidence_detection:
            x1, y1, x2, y2, confidence, class_id = highest_confidence_detection
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            # Draw the bounding box and center point on the frame
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            label_text = f'Conf: {confidence:.2f} Class: {int(class_id)}'
            cv2.putText(frame, label_text, (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # If navigation hasn't been cancelled yet, send a cancellation request
            if not self.navigation_cancelled:
                self.cancel_navigation()
                self.navigation_cancelled = True  # Ensure it's called only once

            # Publish movement commands to align the object with the center and move forward
            twist = Twist()
            alignment_error = center_x - self.screen_center_x

            # Proportional control for angular speed
            angular_speed_gain = 0.005  # Adjust as needed
            max_angular_speed = 0.2     # Max angular speed
            max_linear_speed = 0.11     # Max linear speed (adjusted for your robot)

            if abs(alignment_error) > self.alignment_tolerance:
                # Rotate to align with the object
                twist.angular.z = -angular_speed_gain * alignment_error
                # Limit the angular speed to prevent too fast rotation
                twist.angular.z = max(-max_angular_speed, min(max_angular_speed, twist.angular.z))
                twist.linear.x = 0.0  # Stop moving forward while aligning
            else:
                # When aligned, move forward
                twist.angular.z = 0.0
                twist.linear.x = max_linear_speed  # Move forward

            self.cmd_publisher.publish(twist)

        else:
            # No valid detection; you may choose to stop the robot or keep previous commands
            # Optionally, reset navigation_cancelled if you want the robot to resume navigation
            pass

        # Convert the frame to a ROS 2 Image message and publish
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_publisher.publish(msg)

    def cancel_navigation(self):
        """
        Cancel all FollowWaypoints action goals.
        """
        self.get_logger().info('Sending navigation cancel request...')
        # Create a client for the cancel goal service
        cancel_client = self.create_client(CancelGoal, 'follow_waypoints/_action/cancel_goal')
        if not cancel_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Unable to connect to FollowWaypoints cancel goal service!')
            return

        # Create a CancelGoal request with empty goal_info to cancel all goals
        request = CancelGoal.Request()
        # Leaving goal_info empty cancels all goals

        future = cancel_client.call_async(request)
        future.add_done_callback(self.cancel_navigation_callback)

    def cancel_navigation_callback(self, future):
        """
        Callback after sending navigation cancel request.
        """
        try:
            response = future.result()
            if len(response.goals_canceling) > 0:
                self.get_logger().info('Navigation goals have been successfully cancelled.')
            else:
                self.get_logger().warn('There are no navigation goals to cancel.')
        except Exception as e:
            self.get_logger().error(f'Error occurred while cancelling navigation: {e}')

    def destroy_node(self):
        super().destroy_node()
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = YOLOTrackingPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        if rclpy.ok():
            node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
