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

        # 웨이포인트 리스트 정의
        self.waypoints = self.generate_waypoints()

        # 각 웨이포인트별 발행 횟수 추적
        self.current_waypoint_index = 0
        self.max_publishes_per_waypoint = 10  # 각 웨이포인트당 10번 발행

        # 타이머 설정: 0.5초마다 콜백 호출
        timer_period = 0.5  # 초 단위
        self.timer = self.create_timer(timer_period, self.publish_messages)

    def generate_waypoints(self):
        """
        제공된 웨이포인트 데이터를 기반으로 Marker 메시지 리스트를 생성합니다.
        """
        waypoints = []

        # 웨이포인트 1
        marker1 = Marker()
        marker1.header.frame_id = 'map'
        marker1.ns = 'waypoints'
        marker1.id = 1
        marker1.type = Marker.TEXT_VIEW_FACING
        marker1.action = Marker.ADD
        marker1.pose.position.x = 0.29292383790016174
        marker1.pose.position.y = -0.22626881301403046
        marker1.pose.position.z = 0.2
        marker1.pose.orientation.x = 0.0
        marker1.pose.orientation.y = 0.0
        marker1.pose.orientation.z = -0.7329017748561758
        marker1.pose.orientation.w = 0.6803344680469067
        marker1.scale.x = 0.07
        marker1.scale.y = 0.07
        marker1.scale.z = 0.07
        marker1.color.r = 0.0
        marker1.color.g = 1.0  # 초록색
        marker1.color.b = 0.0
        marker1.color.a = 1.0
        marker1.lifetime.sec = 0
        marker1.lifetime.nanosec = 0
        marker1.frame_locked = False
        marker1.text = 'wp_1'
        waypoints.append(marker1)

        # 웨이포인트 2
        marker2 = Marker()
        marker2.header.frame_id = 'map'
        marker2.ns = 'waypoints'
        marker2.id = 2
        marker2.type = Marker.ARROW
        marker2.action = Marker.ADD
        marker2.pose.position.x = -0.4314682185649872
        marker2.pose.position.y = -0.6317271590232849
        marker2.pose.position.z = 0.0
        marker2.pose.orientation.x = 0.0
        marker2.pose.orientation.y = 0.0
        marker2.pose.orientation.z = 0.9556588551325824
        marker2.pose.orientation.w = 0.2944760645734756
        marker2.scale.x = 0.3
        marker2.scale.y = 0.05
        marker2.scale.z = 0.02
        marker2.color.r = 0.0
        marker2.color.g = 1.0  # 초록색
        marker2.color.b = 0.0
        marker2.color.a = 1.0
        marker2.lifetime.sec = 0
        marker2.lifetime.nanosec = 0
        marker2.frame_locked = False
        waypoints.append(marker2)

        # 웨이포인트 3
        marker3 = Marker()
        marker3.header.frame_id = 'map'
        marker3.ns = 'waypoints'
        marker3.id = 3
        marker3.type = Marker.CUBE
        marker3.action = Marker.ADD
        marker3.pose.position.x = -1.0217899084091187
        marker3.pose.position.y = -0.1412075310945511
        marker3.pose.position.z = 0.2
        marker3.pose.orientation.x = 0.0
        marker3.pose.orientation.y = 0.0
        marker3.pose.orientation.z = -0.9966806227907764
        marker3.pose.orientation.w = 0.08141090930207111
        marker3.scale.x = 0.07
        marker3.scale.y = 0.07
        marker3.scale.z = 0.07
        marker3.color.r = 0.0
        marker3.color.g = 1.0  # 초록색
        marker3.color.b = 0.0
        marker3.color.a = 1.0
        marker3.lifetime.sec = 0
        marker3.lifetime.nanosec = 0
        marker3.frame_locked = False
        marker3.text = 'wp_3'
        waypoints.append(marker3)

        # 웨이포인트 4
        marker4 = Marker()
        marker4.header.frame_id = 'map'
        marker4.ns = 'waypoints'
        marker4.id = 4
        marker4.type = Marker.TEXT_VIEW_FACING
        marker4.action = Marker.ADD
        marker4.pose.position.x = -1.4410333633422852
        marker4.pose.position.y = -0.6268379092216492
        marker4.pose.position.z = 0.2
        marker4.pose.orientation.x = 0.0
        marker4.pose.orientation.y = 0.0
        marker4.pose.orientation.z = -0.6834678690015524
        marker4.pose.orientation.w = 0.7299805970315079
        marker4.scale.x = 0.07
        marker4.scale.y = 0.07
        marker4.scale.z = 0.07
        marker4.color.r = 0.0
        marker4.color.g = 1.0  # 초록색
        marker4.color.b = 0.0
        marker4.color.a = 1.0
        marker4.lifetime.sec = 0
        marker4.lifetime.nanosec = 0
        marker4.frame_locked = False
        marker4.text = 'wp_4'
        waypoints.append(marker4)

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
                initialpose_msg.pose.pose.position.x = 0.03124990686774254
                initialpose_msg.pose.pose.position.y = -0.009375147521495819
                initialpose_msg.pose.pose.position.z = 0.0
                initialpose_msg.pose.pose.orientation.x = 0.0
                initialpose_msg.pose.pose.orientation.y = 0.0
                initialpose_msg.pose.pose.orientation.z = 0.009502555788036916
                initialpose_msg.pose.pose.orientation.w = 0.9999548496974727
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

                # 웨이포인트 메시지 발행
                marker_copy = copy.deepcopy(waypoint)
                marker_copy.header.stamp = current_time.to_msg()
                marker_array = MarkerArray()
                marker_array.markers.append(marker_copy)
                self.waypoints_publisher.publish(marker_array)
                self.get_logger().info(f'Published waypoint [ID: {marker_copy.id}]')

                # 목표 지점 전송
                self.send_goal(waypoint)

                self.current_waypoint_index += 1
            else:
                # 모든 웨이포인트를 발행 완료, 노드 종료
                self.get_logger().info('Finished publishing all waypoints. Shutting down node.')
                self.timer.cancel()
                self.destroy_node()
                rclpy.shutdown()

    def send_goal(self, waypoint: PoseStamped):
        if not self.goal_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint

        self.get_logger().info(f'Sending goal {waypoint.pose.position.x}, {waypoint.pose.position.y}')
        send_goal_future = self.goal_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:
            self.get_logger().info('Goal was canceled.')
        elif status == 5:
            self.get_logger().info('Goal failed.')
        elif status == 3:
            self.get_logger().info('Goal succeeded!')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.current_pose.pose.position.x}, {feedback.current_pose.pose.position.y}')

def main(args=None):
    rclpy.init(args=args)
    node = InitialAndWaypointsPublisher()
    rclpy.spin(node)  # 콜백 처리를 위해 노드 실행
    # 노드가 종료되면 여기로 이동
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
