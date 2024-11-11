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
import threading
import time
import math

class YOLOTrackingPublisher(Node):
    def __init__(self):
        super().__init__('yolo_tracking_publisher')

        # Publishers
        self.processed_image_publisher = self.create_publisher(Image, 'processed_image_amr', 10)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Bridge and Model
        self.bridge = CvBridge()
        self.model = YOLO('/home/rokey3/1_ws/src/best.pt')  # Update the model path as needed

        # Video capture setup
        self.cap = cv2.VideoCapture(0)  # Adjust this to your camera source
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera.")
            return

        # Screen center x-coordinate
        self.screen_center_x = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) / 2)
        self.alignment_tolerance = 50  # Pixel tolerance for alignment

        # Create FollowWaypoints action client
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # Navigation cancellation status tracking variable
        self.navigation_cancelled = False

        # Control flags
        self.frame_count = 0  # Used to process every other frame
        self.lock = threading.Lock()
        self.current_frame = None
        self.processed_frame = None

        # Class names (assuming class 0: car, class 1: dummy)
        self.class_names = ['car', 'dummy']

        # Start threads
        threading.Thread(target=self.capture_frames, daemon=True).start()
        threading.Thread(target=self.process_frames, daemon=True).start()
        self.timer = self.create_timer(0.2, self.publish_image)  # Adjust timer frequency to 5Hz

    def capture_frames(self):
        """Continuously capture frames from the camera in a separate thread."""
        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.current_frame = frame.copy()
            else:
                self.get_logger().warn("Failed to capture frame from webcam.")
            time.sleep(0.01)  # Slight delay to control frame rate

    def process_frames(self):
        """Process frames for object detection and movement commands in a separate thread."""
        while rclpy.ok():
            if self.current_frame is not None and self.frame_count % 2 == 0:
                with self.lock:
                    frame_to_process = self.current_frame.copy()
            else:
                time.sleep(0.01)
                self.frame_count += 1
                continue

            # Run YOLO detection
            results = self.model(frame_to_process)

            # Variables to hold tracking info
            highest_confidence_detection = None
            highest_confidence = 0.7  # Minimum confidence threshold for tracking

            # Iterate over results to draw bounding boxes and find the best class 0 detection
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    confidence = box.conf[0]
                    class_id = int(box.cls[0])

                    # Draw bounding boxes and labels for all detections
                    label_text = f'{self.class_names[class_id]}: {confidence:.2f}'
                    color = (0, 255, 0) if class_id == 0 else (255, 0, 0)  # Green for class 0, blue for others
                    cv2.rectangle(frame_to_process, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame_to_process, label_text, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                    # Check for class 0 detections for tracking
                    if confidence > highest_confidence and class_id == 0:
                        highest_confidence = confidence
                        highest_confidence_detection = (x1, y1, x2, y2, confidence, class_id)

            # If a class 0 detection is found, proceed with tracking
            if highest_confidence_detection:
                x1, y1, x2, y2, confidence, class_id = highest_confidence_detection
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)

                # Draw center point on the frame
                cv2.circle(frame_to_process, (center_x, center_y), 5, (0, 0, 255), -1)

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
                # No valid class 0 detection; you may choose to stop the robot or keep previous commands
                pass

            # Resize the processed frame to reduce data size
            resized_frame = cv2.resize(frame_to_process, (320, 240))  # Resize to 320x240

            # Update processed frame with resized image
            with self.lock:
                self.processed_frame = resized_frame.copy()

            self.frame_count += 1
            time.sleep(0.01)  # Adjust delay if necessary to control processing rate

    def publish_image(self):
        """Publish the latest processed frame."""
        if self.processed_frame is not None:
            with self.lock:
                ros_image = self.bridge.cv2_to_imgmsg(self.processed_frame, encoding="bgr8")
            self.processed_image_publisher.publish(ros_image)

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
