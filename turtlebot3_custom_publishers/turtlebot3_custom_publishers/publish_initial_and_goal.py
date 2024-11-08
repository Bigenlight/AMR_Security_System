#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.duration import Duration
import time

class InitialAndGoalPublisher(Node):
    def __init__(self):
        super().__init__('initial_and_goal_publisher')

        # QoS 설정: TRANSIENT_LOCAL로 설정하여 새로운 구독자에게 마지막 메시지를 전달
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # 퍼블리셔 생성
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos_profile)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', qos_profile)

        # 타이머 설정: 0.5초마다 콜백 호출
        timer_period = 0.5  # 초 단위
        self.timer = self.create_timer(timer_period, self.publish_messages)

        # 노드 시작 시간 기록
        self.start_time = self.get_clock().now()

        # 상태 설정: 'initial' 또는 'goal'
        self.current_state = 'initial'

    def publish_messages(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # 초 단위로 변환

        if self.current_state == 'initial':
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
                # 상태를 'goal'로 변경하고 시작 시간 업데이트
                self.current_state = 'goal'
                self.start_time = self.get_clock().now()
                self.get_logger().info('Switching to publishing /goal_pose')

        elif self.current_state == 'goal':
            if elapsed_time < 10.0:
                # /goal_pose 메시지 작성 및 발행
                goal_pose_msg = PoseStamped()
                goal_pose_msg.header.frame_id = 'map'
                goal_pose_msg.header.stamp = current_time.to_msg()
                goal_pose_msg.pose.position.x = -1.44
                goal_pose_msg.pose.position.y = -0.59
                goal_pose_msg.pose.position.z = 0.0
                goal_pose_msg.pose.orientation.x = 0.0
                goal_pose_msg.pose.orientation.y = 0.0
                goal_pose_msg.pose.orientation.z = 0.06853891909122467
                goal_pose_msg.pose.orientation.w = 0.9976596080365759  # z에 따른 w 값 계산: w = sqrt(1 - z^2)

                self.goal_pose_publisher.publish(goal_pose_msg)
                self.get_logger().info('Published /goal_pose')
            else:
                # 타이머 취소하고 노드 종료
                self.get_logger().info('Finished publishing /goal_pose. Shutting down node.')
                self.timer.cancel()
                self.destroy_node()
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = InitialAndGoalPublisher()
    rclpy.spin(node)  # 콜백 처리를 위해 노드 실행
    # 노드가 종료되면 여기로 이동
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
