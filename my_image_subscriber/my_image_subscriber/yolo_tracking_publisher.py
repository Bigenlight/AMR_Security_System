import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YOLOTrackingPublisher(Node):
    def __init__(self):
        super().__init__('yolo_tracking_publisher')
        self.image_publisher = self.create_publisher(Image, 'tracked_image', 10)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.model = YOLO('/home/rokey3/1_ws/src/best.pt')
        self.cap = cv2.VideoCapture(1)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjust timer frequency as needed

        # Screen center x-coordinate
        self.screen_center_x = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) / 2)
        self.alignment_tolerance = 30  # Pixel tolerance for alignment

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame from webcam.")
            return

        # Run tracking and get results
        results = self.model.track(source=frame, show=False, tracker='bytetrack.yaml')

        highest_confidence_detection = None
        highest_confidence = 0.0

        # Iterate over results to find the object with the highest confidence
        for result in results:
            for detection in result.boxes.data:
                if len(detection) >= 6:
                    x1, y1, x2, y2, confidence, class_id = detection[:6]
                    if confidence > highest_confidence:
                        highest_confidence = confidence
                        highest_confidence_detection = (x1, y1, x2, y2, confidence, class_id)

        # If a detection is found, draw it and control the robot
        if highest_confidence_detection:
            x1, y1, x2, y2, confidence, class_id = highest_confidence_detection
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            # Draw the bounding box and center point on the frame
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            label_text = f'Track_ID: N/A, Conf: {confidence:.2f} Class: {int(class_id)}'
            cv2.putText(frame, label_text, (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Publish movement commands to align the object with the center and move forward
            twist = Twist()
            alignment_error = center_x - self.screen_center_x

            # Proportional control for angular speed
            angular_speed_gain = 0.005  # Adjust this gain as needed
            max_angular_speed = 0.5     # Max angular speed
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

        # Convert the frame to a ROS 2 Image message and publish
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_publisher.publish(msg)



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
