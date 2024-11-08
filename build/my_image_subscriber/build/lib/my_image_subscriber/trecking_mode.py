#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import math

class DetectionTracker(Node):
    def __init__(self):
        super().__init__('detection_tracker')

        # 파라미터 설정
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('desired_distance', 1.0)  # 미터 단위
        self.declare_parameter('Kp_yaw', 0.002)
        self.declare_parameter('Kp_linear_increase', 0.1)
        self.declare_parameter('Kp_linear_decrease', 0.1)
        self.declare_parameter('max_linear_speed', 0.5)    # m/s
        self.declare_parameter('min_linear_speed', 0.0)    # m/s
        self.declare_parameter('desired_speed', 0.2)       # m/s

        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.desired_d = self.get_parameter('desired_distance').get_parameter_value().double_value
        self.Kp_yaw = self.get_parameter('Kp_yaw').get_parameter_value().double_value
        self.Kp_linear_increase = self.get_parameter('Kp_linear_increase').get_parameter_value().double_value
        self.Kp_linear_decrease = self.get_parameter('Kp_linear_decrease').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.min_linear_speed = self.get_parameter('min_linear_speed').get_parameter_value().double_value
        self.desired_speed = self.get_parameter('desired_speed').get_parameter_value().double_value

        # 퍼블리셔 설정
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 서브스크라이버 설정
        self.detection_sub = self.create_subscription(
            String,
            '/detection_info_amr',
            self.detection_callback,
            10
        )

        # 상태 변수
        self.target_found = False
        self.latest_detection = None

        self.get_logger().info('Detection Tracker node has been started.')

    def detection_callback(self, msg):
        """
        YOLO 감지 결과를 처리하는 콜백 함수.
        class_number=0인 객체를 감지하면 트래킹 모드로 전환.
        """
        try:
            detections = json.loads(msg.data)
            # detections는 리스트로 가정
            for detection in detections:
                if detection.get('class_number') == 0 and detection.get('confidence', 0.0) > 0.2:
                    self.target_found = True
                    self.latest_detection = detection
                    self.get_logger().info(f"Target detected: {detection}")
                    return
            # class_number=0인 객체가 없으면
            self.target_found = False
            self.latest_detection = None
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON Decode Error: {e}")

    def estimate_distance(self, diagonal_length):
        """
        바운딩 박스의 대각선을 이용하여 거리를 추정.
        실제 환경에 맞게 보정 필요.
        """
        if diagonal_length == 0:
            return float('inf')
        # 카메라의 초점 거리(f)와 실제 물체의 높이(H)를 알고 있다고 가정
        # 거리는 (H * f) / h 이미지에서의 물체 높이
        # 여기서는 단순히 대각선 길이를 이용한 예시 계산
        focal_length = 700  # 예시 값 (픽셀 단위)
        real_height = 1.5    # 실제 물체의 높이 (미터 단위)
        estimated_distance = (real_height * focal_length) / diagonal_length
        return estimated_distance

    def track_target(self):
        """
        트래킹 모드에서 객체를 추적하여 cmd_vel을 퍼블리시.
        """
        if not self.target_found or self.latest_detection is None:
            return

        detection = self.latest_detection

        # 바운딩 박스의 중앙 x 좌표 계산
        x1, y1, x2, y2 = detection['box_coordinates']
        center_x = (x1 + x2) / 2.0
        error_x = center_x - (self.image_width / 2.0)

        # Yaw 속도 계산 (오류에 비례)
        angular_z = -self.Kp_yaw * error_x

        # 바운딩 박스의 대각선 길이 계산
        width = x2 - x1
        height = y2 - y1
        diagonal = math.sqrt(width**2 + height**2)
        estimated_d = self.estimate_distance(diagonal)

        # 선형 속도 조절
        linear_x = self.desired_speed
        if self.desired_d < estimated_d * 0.95:
            # 선형 속도 증가
            linear_x += self.Kp_linear_increase * (estimated_d - self.desired_d)
            linear_x = min(linear_x, self.max_linear_speed)
        elif self.desired_d > estimated_d * 1.05:
            # 선형 속도 감소
            linear_x -= self.Kp_linear_decrease * (self.desired_d - estimated_d)
            linear_x = max(linear_x, self.min_linear_speed)

        # 트위스트 메시지 작성
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        self.get_logger().info(f"Yaw Error: {error_x:.2f}, Estimated Distance: {estimated_d:.2f} m, "
                               f"Linear Speed: {linear_x:.2f} m/s, Angular Speed: {angular_z:.2f} rad/s")

        # cmd_vel 퍼블리시
        self.cmd_vel_pub.publish(twist)

    def run(self):
        """
        메인 루프.
        """
        timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        타이머 콜백 함수.
        트래킹 모드일 때 주기적으로 트래킹 함수 호출.
        """
        if self.target_found:
            self.track_target()
        else:
            # 네비게이션 모드에서는 별도의 동작을 하지 않음
            pass

def main(args=None):
    rclpy.init(args=args)
    detection_tracker = DetectionTracker()
    detection_tracker.run()
    rclpy.spin(detection_tracker)
    detection_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
