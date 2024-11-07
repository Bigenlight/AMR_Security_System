import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import os
import cv2
import sqlite3
from datetime import datetime
from cv_bridge import CvBridge

VIDEO_SAVE_FOLDER = "captured_videos"
os.makedirs(VIDEO_SAVE_FOLDER, exist_ok=True)

# 데이터베이스 함수
def create_tables():
    connection = sqlite3.connect('mydatabase.db')
    cursor = connection.cursor()

    # 테이블 생성
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS detection_table (
        id INTEGER PRIMARY KEY,
        name TEXT NOT NULL
    );
    """)
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS violation_detected (
        id INTEGER PRIMARY KEY,
        name TEXT NOT NULL,
        time TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    );
    """)
    cursor.execute("""
    CREATE TABLE IF NOT EXISTS videos (
        id INTEGER PRIMARY KEY,
        filename TEXT NOT NULL,
        timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    );
    """)

    print("테이블이 성공적으로 생성되었습니다.")
    connection.commit()
    connection.close()

def save_video_to_db(filename):
    connection = sqlite3.connect('mydatabase.db')
    cursor = connection.cursor()
    cursor.execute("INSERT INTO videos (filename) VALUES (?);", (filename,))
    connection.commit()
    connection.close()

class VideoCaptureNode(Node):
    def __init__(self):
        super().__init__('video_capture_node')
        create_tables()

        # CvBridge 초기화
        self.bridge = CvBridge()
        self.video_writer = None
        self.filename = None

        # 'processed_image' 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            'processed_image',
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info("VideoCaptureNode가 시작되었으며 'processed_image' 토픽을 구독합니다.")

    def image_callback(self, msg):
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # VideoWriter 초기화
        if self.video_writer is None:
            self.initialize_video_writer(cv_image)

        # 프레임을 비디오 파일에 작성
        self.video_writer.write(cv_image)

    def initialize_video_writer(self, frame):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"{VIDEO_SAVE_FOLDER}/video_{timestamp}.mp4"

        # 프레임 크기 얻기
        frame_height, frame_width = frame.shape[:2]

        # 코덱 정의 및 VideoWriter 객체 생성
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(self.filename, fourcc, 20.0, (frame_width, frame_height))
        self.get_logger().info(f"녹화 시작: {self.filename}")

    def destroy_node(self):
        self.get_logger().info("녹화를 종료하고 비디오를 저장합니다.")
        if self.video_writer is not None:
            self.video_writer.release()
            save_video_to_db(self.filename)
            self.get_logger().info(f"비디오 저장 완료: {self.filename}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    video_capture_node = VideoCaptureNode()

    try:
        rclpy.spin(video_capture_node)
    except KeyboardInterrupt:
        pass
    finally:
        video_capture_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
