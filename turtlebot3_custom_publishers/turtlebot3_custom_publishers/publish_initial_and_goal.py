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

        initialpose_qos = QoSProfile(depth=10)
        waypoints_qos = QoSProfile(depth=10)
        waypoints_qos.durability = DurabilityPolicy.VOLATILE

        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', initialpose_qos)
        self.waypoints_publisher = self.create_publisher(MarkerArray, '/waypoints', waypoints_qos)
        self.goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.start_time = self.get_clock().now()
        self.current_state = 'initial'
        self.waypoints = self.generate_waypoints()

        self.current_waypoint_index = 0
        self.waypoint_0_reached = False
        self.waypoint_1_reached = False

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.publish_messages)

    def generate_waypoints(self):
        waypoints = []

        # 웨이포인트 0
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

        if self.current_state == 'initial':
            elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
            if elapsed_time < 10.0:
                initialpose_msg = PoseWithCovarianceStamped()
                initialpose_msg.header.frame_id = 'map'
                initialpose_msg.header.stamp = current_time.to_msg()
                # 초기 위치 설정
                initialpose_msg.pose.pose.position.x = 0.03125
                initialpose_msg.pose.pose.position.y = -0.0094
                initialpose_msg.pose.pose.orientation.z = 0.0095
                initialpose_msg.pose.pose.orientation.w = 0.9999
                self.initialpose_publisher.publish(initialpose_msg)
                self.get_logger().info('Published /initialpose')
            else:
                self.current_state = 'waypoints'
                self.start_time = self.get_clock().now()
                self.get_logger().info('Switching to publishing /waypoints')

        elif self.current_state == 'waypoints':
            if self.current_waypoint_index < len(self.waypoints):
                if (self.current_waypoint_index > 1 and not self.waypoint_0_reached and not self.waypoint_1_reached):
                    self.get_logger().info('Waiting for waypoint 0 and 1 to be reached.')
                    return

                waypoint = self.waypoints[self.current_waypoint_index]
                marker_array = MarkerArray()

                if self.current_waypoint_index in [0, 1]:
                    markers = self.create_markers_for_waypoint(waypoint, self.current_waypoint_index + 1)
                    marker_array.markers.extend(markers)
                    self.get_logger().info(f'Published waypoint {self.current_waypoint_index} with {len(markers)} markers.')
                else:
                    marker = self.create_marker_from_pose(waypoint, self.current_waypoint_index + 1)
                    marker_array.markers.append(marker)
                    self.get_logger().info(f'Published waypoint [ID: {marker.id}]')

                self.waypoints_publisher.publish(marker_array)
                self.send_goal(waypoint)

                self.current_waypoint_index += 1
            else:
                self.get_logger().info('Finished publishing all waypoints. Shutting down node.')
                self.timer.cancel()
                self.destroy_node()
                rclpy.shutdown()

    def send_goal(self, waypoint: PoseStamped):
        if not self.goal_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose 액션 서버가 사용 불가능합니다!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint

        self.get_logger().info(f'목표 지점 전송 중: (x: {waypoint.pose.position.x}, y: {waypoint.pose.position.y})')
        send_goal_future = self.goal_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('목표가 거부되었습니다.')
            return

        self.get_logger().info('목표가 수락되었습니다.')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status

        if status == 4:
            self.get_logger().info('목표가 취소되었습니다.')
        elif status == 5:
            self.get_logger().info('목표 달성에 실패했습니다.')
        elif status == 3:
            self.get_logger().info('목표 달성에 성공했습니다!')
            if self.current_waypoint_index == 1:
                self.waypoint_0_reached = True
            elif self.current_waypoint_index == 2:
                self.waypoint_1_reached = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'피드백 수신: (x: {feedback.current_pose.pose.position.x}, y: {feedback.current_pose.pose.position.y})')

    def create_markers_for_waypoint(self, pose_stamped: PoseStamped, waypoint_number: int) -> list:
        markers = []

        # Marker 0: TYPE 0 (CUBE)
        marker0 = Marker()
        marker0.header.frame_id = pose_stamped.header.frame_id
        marker0.ns = ''
        marker0.id = 0
        marker0.type = Marker.CUBE
        marker0.pose = pose_stamped.pose
        marker0.scale.x = 0.3
        marker0.scale.y = 0.05
        marker0.scale.z = 0.02
        marker0.color.g = 1.0
        marker0.color.a = 1.0
        markers.append(marker0)

        # Marker 1: TYPE 2 (ARROW)
        marker1 = Marker()
        marker1.header.frame_id = pose_stamped.header.frame_id
        marker1.ns = ''
        marker1.id = 1
        marker1.type = Marker.ARROW
        marker1.pose = pose_stamped.pose
        marker1.scale.x = 0.05
        marker1.scale.y = 0.05
        marker1.scale.z = 0.05
        marker1.color.r = 1.0
        marker1.color.a = 1.0
        markers.append(marker1)

        # Marker 2: TYPE 9 (TEXT_VIEW_FACING)
        marker2 = Marker()
        marker2.header.frame_id = pose_stamped.header.frame_id
        marker2.ns = ''
        marker2.id = 2
        marker2.type = Marker.TEXT_VIEW_FACING
        marker2.pose = pose_stamped.pose
        marker2.pose.position.z += 0.2
        marker2.scale.z = 0.07
        marker2.color.g = 1.0
        marker2.color.a = 1.0
        marker2.text = f'wp_{waypoint_number}'
        markers.append(marker2)

        return markers

    def create_marker_from_pose(self, pose_stamped: PoseStamped, marker_id: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = pose_stamped.header.frame_id
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
