#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class InitialAndGoalPublisher(Node):
    def __init__(self):
        super().__init__('initial_and_goal_publisher')

        # QoS 설정: TRANSIENT_LOCAL로 설정하여 새로운 구독자에게 마지막 메시지를 전달
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # 퍼블리셔 생성
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos_profile)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', qos_profile)

        # 타이머 생성: 0.1초 후에 한번만 호출
        self.timer = self.create_timer(0.1, self.publish_messages)

    def publish_messages(self):
        # /initialpose 메시지 작성
        initialpose_msg = PoseWithCovarianceStamped()
        initialpose_msg.header.frame_id = 'map'
        initialpose_msg.header.stamp = self.get_clock().now().to_msg()
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

        # /goal_pose 메시지 작성
        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.frame_id = 'map'
        goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        goal_pose_msg.pose.position.x = -1.44
        goal_pose_msg.pose.position.y = -0.59
        goal_pose_msg.pose.position.z = 0.0
        goal_pose_msg.pose.orientation.x = 0.0
        goal_pose_msg.pose.orientation.y = 0.0
        goal_pose_msg.pose.orientation.z = 0.06853891909122467
        goal_pose_msg.pose.orientation.w = 0.9976596080365759  # z에 따른 w 값 계산: w = sqrt(1 - z^2)

        # 메시지 발행
        self.initialpose_publisher.publish(initialpose_msg)
        self.get_logger().info('Published /initialpose')

        self.goal_pose_publisher.publish(goal_pose_msg)
        self.get_logger().info('Published /goal_pose')

        # 타이머 취소하고 노드 종료
        self.timer.cancel()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = InitialAndGoalPublisher()
    rclpy.spin(node)  # 노드를 계속 실행 상태로 유지하여 콜백 처리
    # 노드가 종료되면 여기로 이동
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
