#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class InitialAndGoalPublisher(Node):
    def __init__(self):
        super().__init__('initial_and_goal_publisher')

        # QoS 설정 (latched topic과 유사한 효과를 위해)
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # 퍼블리셔 생성 (QoS 설정 적용)
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos_profile)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', qos_profile)

        # 메시지 발행
        self.publish_initial_pose()
        self.publish_goal_pose()

    def publish_initial_pose(self):
        # /initialpose 메시지 작성
        initialpose_msg = PoseWithCovarianceStamped()
        initialpose_msg.header.frame_id = 'map'
        initialpose_msg.header.stamp = self.get_clock().now().to_msg()
        initialpose_msg.pose.pose.position.x = 1.0
        initialpose_msg.pose.pose.position.y = 2.0
        initialpose_msg.pose.pose.position.z = 0.0
        initialpose_msg.pose.pose.orientation.x = 0.0
        initialpose_msg.pose.pose.orientation.y = 0.0
        initialpose_msg.pose.pose.orientation.z = 0.0
        initialpose_msg.pose.pose.orientation.w = 1.0
        initialpose_msg.pose.covariance = [0.0] * 36

        # 메시지 발행
        self.initialpose_publisher.publish(initialpose_msg)
        self.get_logger().info('Published /initialpose')

    def publish_goal_pose(self):
        # /goal_pose 메시지 작성
        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.frame_id = 'map'
        goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        goal_pose_msg.pose.position.x = 3.0
        goal_pose_msg.pose.position.y = 4.0
        goal_pose_msg.pose.position.z = 0.0
        goal_pose_msg.pose.orientation.x = 0.0
        goal_pose_msg.pose.orientation.y = 0.0
        goal_pose_msg.pose.orientation.z = 0.707
        goal_pose_msg.pose.orientation.w = 0.707

        # 메시지 발행
        self.goal_pose_publisher.publish(goal_pose_msg)
        self.get_logger().info('Published /goal_pose')

def main(args=None):
    rclpy.init(args=args)
    node = InitialAndGoalPublisher()
    rclpy.spin(node)  # 노드를 계속 실행 상태로 유지
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
