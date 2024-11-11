# image_subscriber.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import numpy as np
from flask import Flask, render_template, request, redirect, url_for, session, flash, Response
import os
from ament_index_python.packages import get_package_share_directory
import time

# ROS ImageSubscriber Node
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # 두 개의 토픽을 구독
        self.subscription1 = self.create_subscription(
            Image,
            'processed_image',
            self.listener_callback1,
            10)
        self.subscription2 = self.create_subscription(
            Image,
            'processed_image_amr',
            self.listener_callback2,
            10)
        
        self.bridge = CvBridge()
        self.latest_ros_frame1 = None
        self.latest_ros_frame2 = None
        self.lock_ros1 = threading.Lock()
        self.lock_ros2 = threading.Lock()
        self.frame_available_ros1 = threading.Event()
        self.frame_available_ros2 = threading.Event()
        
        self.get_logger().info('ImageSubscriber node has been started.')

    def listener_callback1(self, msg):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock_ros1:
                self.latest_ros_frame1 = cv_image.copy()
            self.frame_available_ros1.set()
            self.get_logger().debug('Received new frame on processed_image')
        except Exception as e:
            self.get_logger().error(f'Error converting image from processed_image: {e}')

    def listener_callback2(self, msg):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock_ros2:
                self.latest_ros_frame2 = cv_image.copy()
            self.frame_available_ros2.set()
            self.get_logger().debug('Received new frame on processed_image_amr')
        except Exception as e:
            self.get_logger().error(f'Error converting image from processed_image_amr: {e}')

# Flask 애플리케이션 설정
package_name = 'my_image_subscriber'
package_share_directory = get_package_share_directory(package_name)
template_dir = os.path.join(package_share_directory, 'templates')  # 템플릿 디렉토리 경로

app = Flask(__name__, template_folder=template_dir)
app.secret_key = 'your_secret_key'  # 세션 보안을 위한 비밀 키

# 하드코딩된 사용자 자격 증명 (데모용)
USERNAME = 'user'
PASSWORD = 'password'

# Flask 로그인 페이지 라우트
@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        # 폼 데이터 수집
        username = request.form['username']
        password = request.form['password']

        # 자격 증명 확인
        if username == USERNAME and password == PASSWORD:
            # 세션에 사용자 이름 저장 후 웰컴 페이지로 리디렉션
            session['username'] = username
            flash('Login successful!', 'success')
            return redirect(url_for('welcome'))
        else:
            flash('Invalid username or password!', 'danger')
            return redirect(url_for('login'))

    # GET 요청 시 로그인 페이지 표시
    return render_template('login_center.html')

# Flask 웰컴 페이지 라우트
@app.route('/welcome')
def welcome():
    # 사용자가 로그인했는지 확인
    if 'username' not in session:
        flash('Please log in first!', 'warning')
        return redirect(url_for('login'))
    # 웰컴 페이지 렌더링
    return render_template('welcome_center_two_cam.html', username=session['username'])

# Flask 비디오 피드1 라우트 (processed_image 토픽에서 이미지 수신)
@app.route('/video_feed1')
def video_feed1():
    return Response(generate_ros_frames1(app.image_subscriber_node), mimetype='multipart/x-mixed-replace; boundary=frame')

# Flask 비디오 피드2 라우트 (processed_image_amr 토픽에서 이미지 수신)
@app.route('/video_feed2')
def video_feed2():
    return Response(generate_ros_frames2(app.image_subscriber_node), mimetype='multipart/x-mixed-replace; boundary=frame')

# 비디오 프레임 생성 함수1 (processed_image 토픽)
def generate_ros_frames1(image_subscriber_node):
    while True:
        # 새로운 프레임이 도착할 때까지 대기
        image_subscriber_node.frame_available_ros1.wait()
        with image_subscriber_node.lock_ros1:
            frame = image_subscriber_node.latest_ros_frame1.copy() if image_subscriber_node.latest_ros_frame1 is not None else None
            image_subscriber_node.frame_available_ros1.clear()
        if frame is not None:
            # 프레임 크기 조정 (인코딩 시간 단축)
            frame = cv2.resize(frame, (480, 360))  # 필요에 따라 해상도 조정
            # JPEG 인코딩 최적화 (속도 향상을 위해 품질 낮춤)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]  # 50으로 낮춰 인코딩 속도 향상
            ret, buffer = cv2.imencode('.jpg', frame, encode_param)
            if not ret:
                app.logger.warning('Failed to encode ROS frame1')
                continue
            frame = buffer.tobytes()
            # 프레임 전송
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            # 프레임이 없을 경우 짧은 대기
            time.sleep(0.01)  # 너무 짧게 설정하여 빠르게 다음 프레임을 처리

# 비디오 프레임 생성 함수2 (processed_image_amr 토픽)
def generate_ros_frames2(image_subscriber_node):
    while True:
        # 새로운 프레임이 도착할 때까지 대기
        image_subscriber_node.frame_available_ros2.wait()
        with image_subscriber_node.lock_ros2:
            frame = image_subscriber_node.latest_ros_frame2.copy() if image_subscriber_node.latest_ros_frame2 is not None else None
            image_subscriber_node.frame_available_ros2.clear()
        if frame is not None:
            # 프레임 크기 조정 (인코딩 시간 단축)
            frame = cv2.resize(frame, (480, 360))  # 필요에 따라 해상도 조정
            # JPEG 인코딩 최적화 (속도 향상을 위해 품질 낮춤)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]  # 50으로 낮춰 인코딩 속도 향상
            ret, buffer = cv2.imencode('.jpg', frame, encode_param)
            if not ret:
                app.logger.warning('Failed to encode ROS frame2')
                continue
            frame = buffer.tobytes()
            # 프레임 전송
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            # 프레임이 없을 경우 짧은 대기
            time.sleep(0.01)  # 너무 짧게 설정하여 빠르게 다음 프레임을 처리

# 로그아웃 라우트
@app.route('/logout')
def logout():
    # 세션 정리 후 로그인 페이지로 리디렉션
    session.pop('username', None)
    flash('Logged out successfully!', 'info')
    return redirect(url_for('login'))

# Flask 애플리케이션 실행 함수
def run_flask_app():
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)  # debug=True로 설정

# 메인 함수
def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    app.image_subscriber_node = node  # Flask 애플리케이션에 ROS 노드 인스턴스 연결

    # Flask 애플리케이션을 별도의 스레드에서 실행
    flask_thread = threading.Thread(target=run_flask_app)
    flask_thread.daemon = True
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
