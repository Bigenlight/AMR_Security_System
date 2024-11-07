import rclpy
from rclpy.node import Node
import os
import cv2
import time
import sqlite3
from datetime import datetime

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
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다.")
            return

        # 비디오 저장을 위한 설정
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"{VIDEO_SAVE_FOLDER}/video_{timestamp}.mp4"
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.out = cv2.VideoWriter(self.filename, fourcc, 20.0, (frame_width, frame_height))

        self.get_logger().info(f"녹화 시작: {self.filename}")

        # 비디오 녹화를 위한 타이머 설정 (빠른 주기로 프레임 캡처)
        self.timer = self.create_timer(0.05, self.record_video)  # 20 FPS

    def record_video(self):
        ret, frame = self.cap.read()
        if ret:
            self.out.write(frame)
        else:
            self.get_logger().error("프레임을 읽을 수 없습니다.")

    def destroy_node(self):
        self.get_logger().info("녹화를 종료하고 비디오를 저장합니다.")
        if self.cap.isOpened():
            self.cap.release()
        self.out.release()
        save_video_to_db(self.filename)
        self.get_logger().info(f"비디오 저장 완료: {self.filename}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    video_capture_node = VideoCaptureNode()

    if not video_capture_node.cap.isOpened():
        rclpy.shutdown()
        return

    try:
        rclpy.spin(video_capture_node)
    except KeyboardInterrupt:
        pass
    finally:
        video_capture_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
