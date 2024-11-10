#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import copy

class InitialAndWaypointsPublisher(Node):
    def __init__(self):
        super().__init__('initial_and_waypoints_publisher')

        # QoS 설정: initialpose는 기본 QoS (VOLATILE)
        initialpose_qos = QoSProfile(depth=10)

        # QoS 설정: waypoints는 VOLATILE Durability
        waypoints_qos = QoSProfile(depth=10)
        waypoints_qos.durability = DurabilityPolicy.VOLATILE

        # 퍼블리셔 생성
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', initialpose_qos)
        self.waypoints_publisher = self.create_publisher(MarkerArray, '/waypoints', waypoints_qos)  # MarkerArray 퍼블리셔

        # 액션 클라이언트 생성
        self.goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 노드 시작 시간 기록
        self.start_time = self.get_clock().now()

        # 상태 설정: 'initial' 또는 'waypoints'
        self.current_state = 'initial'

        # 웨이포인트 리스트 정의 (PoseStamped 타입)
        self.waypoints = self.generate_waypoints()

        # 각 웨이포인트별 발행 횟수 추적
        self.current_waypoint_index = 0
        self.max_publishes_per_waypoint = 10  # 각 웨이포인트당 10번 발행

        # 타이머 설정: 0.5초마다 콜백 호출
        timer_period = 0.5  # 초 단위
        self.timer = self.create_timer(timer_period, self.publish_messages)

    def generate_waypoints(self):
        """
        PoseStamped 타입의 웨이포인트 리스트를 생성합니다.
        """
        waypoints = []

        # 웨이포인트 1 (수정됨)
        pose1 = PoseStamped()
        pose1.header.frame_id = 'map'
        pose1.pose.position.x = 0.24999737739562988
        pose1.pose.position.y = -0.7093759775161743
        pose1.pose.position.z = 0.0
        pose1.pose.orientation.x = 0.0
        pose1.pose.orientation.y = 0.0
        pose1.pose.orientation.z = 0.9988731702878725
        pose1.pose.orientation.w = 0.0474593476467486
        waypoints.append(pose1)

        # 웨이포인트 2
        pose2 = PoseStamped()
        pose2.header.frame_id = 'map'
        pose2.pose.position.x = -0.4314682185649872
        pose2.pose.position.y = -0.6317271590232849
        pose2.pose.position.z = 0.0
        pose2.pose.orientation.x = 0.0
        pose2.pose.orientation.y = 0.0
        pose2.pose.orientation.z = 0.9556588551325824
        pose2.pose.orientation.w = 0.2944760645734756
        waypoints.append(pose2)

        # 웨이포인트 3
        pose3 = PoseStamped()
        pose3.header.frame_id = 'map'
        pose3.pose.position.x = -1.0217899084091187
        pose3.pose.position.y = -0.1412075310945511
        pose3.pose.position.z = 0.2
        pose3.pose.orientation.x = 0.0
        pose3.pose.orientation.y = 0.0
        pose3.pose.orientation.z = -0.9966806227907764
        pose3.pose.orientation.w = 0.08141090930207111
        waypoints.append(pose3)

        # 웨이포인트 4
        pose4 = PoseStamped()
        pose4.header.frame_id = 'map'
        pose4.pose.position.x = -1.4410333633422852
        pose4.pose.position.y = -0.6268379092216492
        pose4.pose.position.z = 0.2
        pose4.pose.orientation.x = 0.0
        pose4.pose.orientation.y = 0.0
        pose4.pose.orientation.z = -0.6834678690015524
        pose4.pose.orientation.w = 0.7299805970315079
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
                # 상태를 'waypoints'로 변경하고 시작 시간 업데이트
                self.current_state = 'waypoints'
                self.start_time = self.get_clock().now()
                self.get_logger().info('Switching to publishing /waypoints')

        elif self.current_state == 'waypoints':
            if self.current_waypoint_index < len(self.waypoints):
                waypoint = self.waypoints[self.current_waypoint_index]

                marker_array = MarkerArray()

                if self.current_waypoint_index == 0:
                    # 첫 번째 웨이포인트에 대해 세 개의 Marker 생성
                    markers = self.create_markers_for_first_waypoint(waypoint)
                    marker_array.markers.extend(markers)
                    self.get_logger().info(f'Published first waypoint with {len(markers)} markers.')
                else:
                    # 나머지 웨이포인트에 대해 단일 Marker 생성
                    marker = self.create_marker_from_pose(waypoint, self.current_waypoint_index + 1)
                    marker_array.markers.append(marker)
                    self.get_logger().info(f'Published waypoint [ID: {marker.id}]')

                self.waypoints_publisher.publish(marker_array)

                # 목표 지점 전송 (NavigateToPose 액션 클라이언트 사용)
                self.send_goal(waypoint)

                self.current_waypoint_index += 1
            else:
                # 모든 웨이포인트를 발행 완료, 노드 종료
                self.get_logger().info('Finished publishing all waypoints. Shutting down node.')
                self.timer.cancel()
                self.destroy_node()
                rclpy.shutdown()

    def create_markers_for_first_waypoint(self, pose_stamped: PoseStamped) -> list:
        """
        첫 번째 웨이포인트를 위한 세 개의 Marker를 생성합니다.
        """
        markers = []

        # Marker 0: TYPE 0 (CUBE)
        marker0 = Marker()
        marker0.header.frame_id = pose_stamped.header.frame_id
        marker0.header.stamp = self.get_clock().now().to_msg()  # 현재 시간으로 설정
        marker0.ns = ''
        marker0.id = 0
        marker0.type = Marker.CUBE
        marker0.action = Marker.ADD
        marker0.pose = pose_stamped.pose
        marker0.scale.x = 0.3
        marker0.scale.y = 0.05
        marker0.scale.z = 0.02
        marker0.color.r = 0.0
        marker0.color.g = 1.0  # 초록색 (0.0 - 1.0 범위)
        marker0.color.b = 0.0
        marker0.color.a = 1.0
        marker0.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        marker0.frame_locked = False
        markers.append(marker0)

        # Marker 1: TYPE 2 (ARROW)
        marker1 = Marker()
        marker1.header.frame_id = pose_stamped.header.frame_id
        marker1.header.stamp = self.get_clock().now().to_msg()  # 현재 시간으로 설정
        marker1.ns = ''
        marker1.id = 1
        marker1.type = Marker.ARROW
        marker1.action = Marker.ADD
        marker1.pose = pose_stamped.pose
        marker1.scale.x = 0.3
        marker1.scale.y = 0.05
        marker1.scale.z = 0.02
        marker1.color.r = 1.0  # 빨간색 (0.0 - 1.0 범위)
        marker1.color.g = 0.0
        marker1.color.b = 0.0
        marker1.color.a = 1.0
        marker1.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        marker1.frame_locked = False
        markers.append(marker1)

        # Marker 2: TYPE 9 (TEXT_VIEW_FACING)
        marker2 = Marker()
        marker2.header.frame_id = pose_stamped.header.frame_id
        marker2.header.stamp = self.get_clock().now().to_msg()  # 현재 시간으로 설정
        marker2.ns = ''
        marker2.id = 2
        marker2.type = Marker.TEXT_VIEW_FACING
        marker2.action = Marker.ADD
        marker2.pose = pose_stamped.pose
        marker2.pose.position.z += 0.2  # 텍스트를 위로 올림
        marker2.scale.z = 0.07
        marker2.color.r = 0.0
        marker2.color.g = 1.0  # 초록색 (0.0 - 1.0 범위)
        marker2.color.b = 0.0
        marker2.color.a = 1.0
        marker2.text = 'wp_1'
        marker2.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        marker2.frame_locked = False
        markers.append(marker2)

        return markers

    def create_marker_from_pose(self, pose_stamped: PoseStamped, marker_id: int) -> Marker:
        """
        PoseStamped 메시지로부터 Marker 메시지를 생성합니다.
        """
        marker = Marker()
        marker.header.frame_id = pose_stamped.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'waypoints'
        marker.id = marker_id

        # Marker 유형 설정 (예: CUBE)
        marker.type = Marker.CUBE
        marker.scale.x = 0.07
        marker.scale.y = 0.07
        marker.scale.z = 0.07
        marker.pose = pose_stamped.pose

        marker.color.r = 0.0
        marker.color.g = 1.0  # 초록색
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        marker.frame_locked = False

        return marker

    def send_goal(self, waypoint: PoseStamped):
        """
        NavigateToPose 액션 서버에 목표 지점을 전송합니다.
        """
        if not self.goal_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose 액션 서버가 사용 불가능합니다!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint

        self.get_logger().info(f'목표 지점 전송 중: (x: {waypoint.pose.position.x}, y: {waypoint.pose.position.y})')
        send_goal_future = self.goal_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        목표 지점 수락 여부를 확인하는 콜백 함수.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('목표가 거부되었습니다.')
            return

        self.get_logger().info('목표가 수락되었습니다.')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        목표 지점 도착 후 결과를 처리하는 콜백 함수.
        """
        result = future.result().result
        status = future.result().status

        if status == 4:
            self.get_logger().info('목표가 취소되었습니다.')
        elif status == 5:
            self.get_logger().info('목표 달성에 실패했습니다.')
        elif status == 3:
            self.get_logger().info('목표 달성에 성공했습니다!')

    def feedback_callback(self, feedback_msg):
        """
        목표 지점 도착 중 피드백을 처리하는 콜백 함수.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'피드백 수신: (x: {feedback.current_pose.pose.position.x}, y: {feedback.current_pose.pose.position.y})')

def main(args=None):
    rclpy.init(args=args)
    node = InitialAndWaypointsPublisher()
    rclpy.spin(node)  # 콜백 처리를 위해 노드 실행
    # 노드가 종료되면 여기로 이동
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
