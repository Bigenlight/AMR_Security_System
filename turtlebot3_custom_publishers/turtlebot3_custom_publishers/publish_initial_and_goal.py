import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient

class InitialAndWaypointsPublisher(Node):
    def __init__(self):
        super().__init__('initial_and_waypoints_publisher')

        # QoS 설정: initialpose는 기본 QoS (VOLATILE)
        initialpose_qos = QoSProfile(depth=10)

        # 퍼블리셔 생성
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', initialpose_qos)
        self.waypoints_publisher = self.create_publisher(MarkerArray, '/waypoints', initialpose_qos)  # MarkerArray 퍼블리셔

        # 액션 클라이언트 생성: FollowWaypoints 액션 클라이언트
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # 노드 시작 시간 기록
        self.start_time = self.get_clock().now()

        # 상태 설정: 'initial' 또는 'send_waypoints'
        self.current_state = 'initial'

        # 웨이포인트 리스트 정의 (PoseStamped 타입)
        self.waypoints = self.generate_waypoints()

        # 타이머 설정: 0.5초마다 콜백 호출
        timer_period = 0.5  # 초 단위
        self.timer = self.create_timer(timer_period, self.publish_messages)

    def generate_waypoints(self):
        """
        PoseStamped 타입의 웨이포인트 리스트를 생성합니다.
        """
        waypoints = []

        # 웨이포인트 0 (사용자가 제공한 데이터로 수정됨)
        pose0 = PoseStamped()
        pose0.header.frame_id = 'map'
        pose0.pose.position.x = 0.3625
        pose0.pose.position.y = -0.0844
        pose0.pose.position.z = 0.0
        pose0.pose.orientation.x = 0.0
        pose0.pose.orientation.y = 0.0
        pose0.pose.orientation.z = -0.0049
        pose0.pose.orientation.w = 0.9999
        waypoints.append(pose0)

        # 웨이포인트 1 (사용자가 제공한 데이터로 수정됨)
        pose1 = PoseStamped()
        pose1.header.frame_id = 'map'
        pose1.pose.position.x = 0.3
        pose1.pose.position.y = -0.8094
        pose1.pose.position.z = 0.0
        pose1.pose.orientation.x = 0.0
        pose1.pose.orientation.y = 0.0
        pose1.pose.orientation.z = 0.9809
        pose1.pose.orientation.w = 0.1945
        waypoints.append(pose1)

        # 웨이포인트 2
        pose2 = PoseStamped()
        pose2.header.frame_id = 'map'
        pose2.pose.position.x = -0.4315
        pose2.pose.position.y = -0.6317
        pose2.pose.position.z = 0.0
        pose2.pose.orientation.x = 0.0
        pose2.pose.orientation.y = 0.0
        pose2.pose.orientation.z = 0.9557
        pose2.pose.orientation.w = 0.2945
        waypoints.append(pose2)

        # 웨이포인트 3
        pose3 = PoseStamped()
        pose3.header.frame_id = 'map'
        pose3.pose.position.x = -1.0218
        pose3.pose.position.y = -0.1412
        pose3.pose.position.z = 0.2
        pose3.pose.orientation.x = 0.0
        pose3.pose.orientation.y = 0.0
        pose3.pose.orientation.z = -0.9967
        pose3.pose.orientation.w = 0.0814
        waypoints.append(pose3)

        # 웨이포인트 4
        pose4 = PoseStamped()
        pose4.header.frame_id = 'map'
        pose4.pose.position.x = -1.4410
        pose4.pose.position.y = -0.6268
        pose4.pose.position.z = 0.0
        pose4.pose.orientation.x = 0.0
        pose4.pose.orientation.y = 0.0
        pose4.pose.orientation.z = -0.6835
        pose4.pose.orientation.w = 0.7300
        waypoints.append(pose4)

        return waypoints

    def publish_messages(self):
        current_time = self.get_clock().now()

        if self.current_state == 'initial':
            elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # 초 단위로 변환
            if elapsed_time < 10.0:
                # /initialpose 메시지 작성 및 발행
                initialpose_msg = PoseWithCovarianceStamped()
                initialpose_msg.header.frame_id = 'map'
                initialpose_msg.header.stamp = current_time.to_msg()
                initialpose_msg.pose.pose.position.x = 0.03124990686774254  # 사용자 제공 값
                initialpose_msg.pose.pose.position.y = -0.009375147521495819  # 사용자 제공 값
                initialpose_msg.pose.pose.position.z = 0.0
                initialpose_msg.pose.pose.orientation.x = 0.0
                initialpose_msg.pose.pose.orientation.y = 0.0
                initialpose_msg.pose.pose.orientation.z = 0.009502555788036916  # 사용자 제공 값
                initialpose_msg.pose.pose.orientation.w = 0.9999548496974727  # 사용자 제공 값
                initialpose_msg.pose.covariance = [
                    0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
                ]

                self.initialpose_publisher.publish(initialpose_msg)
                self.get_logger().info('Published /initialpose')
            else:
                # 상태를 'send_waypoints'로 변경하고 시작 시간 업데이트
                self.current_state = 'send_waypoints'
                self.start_time = self.get_clock().now()
                self.get_logger().info('Switching to sending /follow_waypoints')

                # FollowWaypoints 목표 전송
                self.send_follow_waypoints_goal()

    def send_follow_waypoints_goal(self):
        """
        FollowWaypoints 액션 서버에 웨이포인트 리스트를 전송합니다.
        """
        if not self.follow_waypoints_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('FollowWaypoints 액션 서버가 사용 불가능합니다!')
            return

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints

        self.get_logger().info('FollowWaypoints 목표 전송 중...')
        send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        목표 수락 여부를 확인하는 콜백 함수.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('FollowWaypoints 목표가 거부되었습니다.')
            return

        self.get_logger().info('FollowWaypoints 목표가 수락되었습니다.')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        목표 도착 후 결과를 처리하는 콜백 함수.
        """
        result = future.result().result
        status = future.result().status

        if status == 2:
            self.get_logger().info('FollowWaypoints 목표가 취소되었습니다.')
        elif status == 3:
            self.get_logger().info('FollowWaypoints 목표가 실행 중입니다...')
        elif status == 4:
            self.get_logger().info('FollowWaypoints 목표가 실패했습니다.')
        elif status == 5:
            self.get_logger().info('FollowWaypoints 목표가 완료되었습니다!')
            # 모든 웨이포인트 실행 완료 후 노드 종료
            self.get_logger().info('모든 웨이포인트 실행 완료. 노드를 종료합니다.')
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """
        목표 도착 중 피드백을 처리하는 콜백 함수.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'피드백 수신: 현재 위치 - (x: {feedback.current_pose.pose.position.x}, y: {feedback.current_pose.pose.position.y})')

def main(args=None):
    rclpy.init(args=args)
    node = InitialAndWaypointsPublisher()
    rclpy.spin(node)  # 콜백 처리를 위해 노드 실행
    # 노드가 종료되면 여기로 이동
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
