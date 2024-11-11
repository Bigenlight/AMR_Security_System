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

    # 테이블 생성 (source 필드 추가)
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
        source TEXT NOT NULL,
        timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    );
    """)

    print("테이블이 성공적으로 생성되었습니다.")
    connection.commit()
    connection.close()

def save_video_to_db(filename, source):
    try:
        connection = sqlite3.connect('mydatabase.db')
        cursor = connection.cursor()
        cursor.execute("INSERT INTO videos (filename, source) VALUES (?, ?);", (filename, source))
        connection.commit()
    except sqlite3.Error as e:
        print(f"데이터베이스 에러: {e}")
    finally:
        if connection:
            connection.close()

class VideoCaptureNode(Node):
    def __init__(self):
        super().__init__('video_capture_node')
        create_tables()

        # CvBridge 초기화
        self.bridge = CvBridge()
        
        # 각 토픽에 대한 VideoWriter와 파일명을 저장할 딕셔너리
        self.video_writers = {}
        self.filenames = {}
        self.sources = ['processed_image', 'processed_image_amr']  # 구독할 토픽 리스트

        for source in self.sources:
            self.video_writers[source] = None
            self.filenames[source] = None

        # 이미지 토픽 구독 설정
        self.subscription_list = []  # 기존 self.subscriptions를 self.subscription_list로 변경
        for source in self.sources:
            subscription = self.create_subscription(
                Image,
                source,
                lambda msg, s=source: self.image_callback(msg, s),
                10
            )
            self.subscription_list.append(subscription)
            self.get_logger().info(f"{source} 토픽을 구독합니다.")


    def image_callback(self, msg, source):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"{source} 이미지 변환 실패: {e}")
            return

        # VideoWriter 초기화
        if self.video_writers[source] is None:
            self.initialize_video_writer(cv_image, source)

        # VideoWriter가 정상적으로 초기화되었는지 확인
        if self.video_writers[source].isOpened():
            # 프레임을 비디오 파일에 작성
            self.video_writers[source].write(cv_image)
        else:
            self.get_logger().error(f"{source} VideoWriter가 열리지 않았습니다.")

    def initialize_video_writer(self, frame, source):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filenames[source] = f"{VIDEO_SAVE_FOLDER}/{source}_video_{timestamp}.mp4"

        # 프레임 크기 얻기
        frame_height, frame_width = frame.shape[:2]

        # 코덱 정의 및 VideoWriter 객체 생성
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writers[source] = cv2.VideoWriter(self.filenames[source], fourcc, 20.0, (frame_width, frame_height))

        if self.video_writers[source].isOpened():
            self.get_logger().info(f"{source} 녹화 시작: {self.filenames[source]}")
        else:
            self.get_logger().error(f"{source} VideoWriter 초기화 실패: {self.filenames[source]}")
            self.video_writers[source] = None  # 초기화 실패 시 None으로 설정

    def destroy_node(self):
        self.get_logger().info("녹화를 종료하고 비디오를 저장합니다.")
        for source in self.sources:
            if self.video_writers[source] is not None:
                self.video_writers[source].release()
                save_video_to_db(self.filenames[source], source)
                self.get_logger().info(f"{source} 비디오 저장 완료: {self.filenames[source]}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    video_capture_node = VideoCaptureNode()

    try:
        rclpy.spin(video_capture_node)
    except KeyboardInterrupt:
        video_capture_node.get_logger().info("사용자에 의해 노드가 중단되었습니다.")
    finally:
        video_capture_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
