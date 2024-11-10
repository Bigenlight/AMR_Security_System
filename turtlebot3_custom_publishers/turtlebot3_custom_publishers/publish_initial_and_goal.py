#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class InitialAndWaypointsPublisher(Node):
    def __init__(self):
        super().__init__('initial_and_waypoints_publisher')

        # QoS 설정
        initialpose_qos = QoSProfile(depth=10)
        waypoints_qos = QoSProfile(depth=10)
        waypoints_qos.durability = DurabilityPolicy.VOLATILE

        # 퍼블리셔 생성
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', initialpose_qos)
        self.waypoints_publisher = self.create_publisher(MarkerArray, '/waypoints', waypoints_qos)

        # 액션 클라이언트 생성
        self.goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 노드 시작 시간 기록
        self.start_time = self.get_clock().now()
        self.current_state = 'initial'
        self.waypoints = self.generate_waypoints()
        self.current_waypoint_index = 0

        # 타이머 설정: 0.5초마다 콜백 호출
        self.timer = self.create_timer(0.5, self.publish_messages)

    def generate_waypoints(self):
        waypoints = []

        # 웨이포인트 0 - 사용자 지정
        pose0 = PoseStamped()
        pose0.header.frame_id = 'map'
        pose0.pose.position.x = 0.3625
        pose0.pose.position.y = -0.0844
        pose0.pose.orientation.z = -0.0049
        pose0.pose.orientation.w = 0.9999
        waypoints.append(pose0)

        # 웨이포인트 1
        pose1 = PoseStamped()
        pose1.header.frame_id = 'map'
        pose1.pose.position.x = 0.3
        pose1.pose.position.y = -0.8094
        pose1.pose.orientation.z = 0.9809
        pose1.pose.orientation.w = 0.1945
        waypoints.append(pose1)

        # 웨이포인트 2
        pose2 = PoseStamped()
        pose2.header.frame_id = 'map'
        pose2.pose.position.x = -0.4315
        pose2.pose.position.y = -0.6317
        pose2.pose.orientation.z = 0.9557
        pose2.pose.orientation.w = 0.2945
        waypoints.append(pose2)

        # 웨이포인트 3
        pose3 = PoseStamped()
        pose3.header.frame_id = 'map'
        pose3.pose.position.x = -1.0218
        pose3.pose.position.y = -0.1412
        pose3.pose.orientation.z = -0.9967
        pose3.pose.orientation.w = 0.0814
        waypoints.append(pose3)

        # 웨이포인트 4
        pose4 = PoseStamped()
        pose4.header.frame_id = 'map'
        pose4.pose.position.x = -1.4410
        pose4.pose.position.y = -0.6268
        pose4.pose.orientation.z = -0.6835
        pose4.pose.orientation.w = 0.7300
        waypoints.append(pose4)

        return waypoints

    def publish_messages(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        if self.current_state == 'initial' and elapsed_time < 10.0:
            initialpose_msg = PoseWithCovarianceStamped()
            initialpose_msg.header.frame_id = 'map'
            initialpose_msg.header.stamp = current_time.to_msg()
            initialpose_msg.pose.pose.position.x = 0.03125
            initialpose_msg.pose.pose.position.y = -0.0094
            initialpose_msg.pose.pose.orientation.z = 0.0095
            initialpose_msg.pose.pose.orientation.w = 0.9999
            initialpose_msg.pose.covariance[0] = 0.25  # 예시로 일부만 설정
            initialpose_msg.pose.covariance[7] = 0.25
            initialpose_msg.pose.covariance[35] = 0.06853891909122467
            self.initialpose_publisher.publish(initialpose_msg)
            self.get_logger().info('Published /initialpose')
        elif self.current_state == 'initial' and elapsed_time >= 10.0:
            # 상태 전환
            self.current_state = 'waypoints'
            self.publish_next_waypoint()

    def publish_next_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
            marker_array = MarkerArray()
            marker = self.create_marker_from_pose(waypoint, self.current_waypoint_index + 1)
            marker_array.markers.append(marker)
            self.waypoints_publisher.publish(marker_array)
            self.get_logger().info(f'Published waypoint {self.current_waypoint_index}')
            self.send_goal(waypoint)
        else:
            self.get_logger().info('All waypoints reached. Shutting down.')
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()

    def send_goal(self, waypoint: PoseStamped):
        if not self.goal_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server is unavailable!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint
        self.get_logger().info(f'Sending goal: (x: {waypoint.pose.position.x}, y: {waypoint.pose.position.y})')
        send_goal_future = self.goal_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected.')
            return

        self.get_logger().info('Goal was accepted.')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        if future.result().status == 3:
            self.get_logger().info('Successfully reached the waypoint!')
            self.current_waypoint_index += 1
            self.publish_next_waypoint()
        else:
            self.get_logger().info('Failed to reach the waypoint or was canceled.')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: (x: {feedback.current_pose.pose.position.x}, y: {feedback.current_pose.pose.position.y})')

    def create_marker_from_pose(self, pose_stamped: PoseStamped, marker_id: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = pose_stamped.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'waypoints'
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.scale.x = 0.07
        marker.scale.y = 0.07
        marker.scale.z = 0.07
        marker.pose = pose_stamped.pose
        marker.color.g = 1.0
        marker.color.a = 1.0
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = InitialAndWaypointsPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
