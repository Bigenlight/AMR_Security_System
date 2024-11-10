# yolo_tracking_publisher.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
from nav2_msgs.action import FollowWaypoints  # FollowWaypoints 액션 임포트
from rclpy.action import ActionClient
from action_msgs.srv import CancelGoal

class YOLOTrackingPublisher(Node):
    def __init__(self):
        super().__init__('yolo_tracking_publisher')
        self.image_publisher = self.create_publisher(Image, 'tracked_image', 10)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.model = YOLO('/home/rokey3/1_ws/src/best.pt')
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 타이머 주기 조절 가능

        # 화면 중심 x좌표
        self.screen_center_x = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) / 2)
        self.alignment_tolerance = 50  # 정렬을 위한 픽셀 허용 오차

        # FollowWaypoints 액션 클라이언트 생성
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # 네비게이션 취소 상태 추적 변수
        self.navigation_cancelled = False

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("웹캠에서 프레임을 캡처하지 못했습니다.")
            return

        # 트래킹 실행 및 결과 가져오기
        results = self.model.track(source=frame, show=False, tracker='bytetrack.yaml')

        highest_confidence_detection = None
        highest_confidence = 0.0

        # 결과를 순회하며 가장 높은 신뢰도의 객체 찾기
        for result in results:
            for detection in result.boxes.data:
                if len(detection) >= 6:
                    x1, y1, x2, y2, confidence, class_id = detection[:6]
                    if confidence > highest_confidence:
                        highest_confidence = confidence
                        highest_confidence_detection = (x1, y1, x2, y2, confidence, class_id)

        # 감지가 이루어진 경우, 박스를 그리며 로봇 제어
        if highest_confidence_detection:
            x1, y1, x2, y2, confidence, class_id = highest_confidence_detection
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            # 프레임에 바운딩 박스와 중심점 그리기
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            label_text = f'Track_ID: N/A, Conf: {confidence:.2f} Class: {int(class_id)}'
            cv2.putText(frame, label_text, (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 네비게이션이 아직 취소되지 않았다면, 취소 요청
            if not self.navigation_cancelled:
                self.cancel_navigation()
                self.navigation_cancelled = True  # 한 번만 호출되도록 설정

            # 트래킹 모드에서 로봇 제어 명령 발행
            twist = Twist()
            alignment_error = center_x - self.screen_center_x

            # 비례 제어를 위한 각속도 이득
            angular_speed_gain = 0.005  # 필요에 따라 조절
            max_angular_speed = 0.2     # 최대 각속도
            max_linear_speed = 0.11     # 최대 선속도 (로봇에 맞게 조절)

            if abs(alignment_error) > self.alignment_tolerance:
                # 객체와 정렬을 위해 회전
                twist.angular.z = -angular_speed_gain * alignment_error
                # 과도한 회전을 방지하기 위해 각속도 제한
                twist.angular.z = max(-max_angular_speed, min(max_angular_speed, twist.angular.z))
                twist.linear.x = 0.0  # 정렬 중에는 전진하지 않음
            else:
                # 정렬이 완료되면 전진
                twist.angular.z = 0.0
                twist.linear.x = max_linear_speed  # 전진

            self.cmd_publisher.publish(twist)

        # 프레임을 ROS 2 Image 메시지로 변환하여 발행
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_publisher.publish(msg)

    def cancel_navigation(self):
        """
        Cancel all FollowWaypoints action goals.
        """
        self.get_logger().info('Sending navigation cancel request...')
        # Ensure the action server is connected
        if not self.follow_waypoints_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Unable to connect to FollowWaypoints action server!')
            return

        # Create a CancelGoal request to cancel all goals
        cancel_request = CancelGoal.Request()
        # Leave goal_info empty to cancel all goals
        cancel_future = self.follow_waypoints_client.cancel_goals_async(cancel_request)
        cancel_future.add_done_callback(self.cancel_navigation_callback)

    def cancel_navigation_callback(self, future):
        """
        Callback after sending navigation cancel request.
        """
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
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
        node.get_logger().info("노드가 사용자에 의해 중단되었습니다.")
    finally:
        if rclpy.ok():
            node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
