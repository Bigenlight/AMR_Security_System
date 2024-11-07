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
        self.subscription = self.create_subscription(
            Image,
            'processed_image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.latest_ros_frame = None
        self.lock_ros = threading.Lock()
        self.frame_available_ros = threading.Event()
        self.get_logger().info('ImageSubscriber node has been started.')

    def listener_callback(self, msg):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock_ros:
                self.latest_ros_frame = cv_image.copy()
            self.frame_available_ros.set()
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

# Flask 애플리케이션 설정
package_name = 'my_image_subscriber'
package_share_directory = get_package_share_directory(package_name)
template_dir = os.path.join(package_share_directory, 'templates')

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

# Flask 비디오 피드1: ROS 토픽에서 이미지 수신
def generate_ros_frames(image_subscriber_node):
    while True:
        # 새로운 프레임이 도착할 때까지 대기
        image_subscriber_node.frame_available_ros.wait()
        with image_subscriber_node.lock_ros:
            frame = image_subscriber_node.latest_ros_frame.copy() if image_subscriber_node.latest_ros_frame is not None else None
            image_subscriber_node.frame_available_ros.clear()
        if frame is not None:
            # 프레임 크기 조정 (인코딩 시간 단축)
            frame = cv2.resize(frame, (480, 360))  # 필요에 따라 해상도 조정
            # JPEG 인코딩 최적화 (속도 향상을 위해 품질 낮춤)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]  # 50으로 낮춰 인코딩 속도 향상
            ret, buffer = cv2.imencode('.jpg', frame, encode_param)
            if not ret:
                app.logger.warning('Failed to encode ROS frame')
                continue
            frame = buffer.tobytes()
            # 프레임 전송
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            # 프레임이 없을 경우 짧은 대기
            time.sleep(0.1)

# Flask 비디오 피드2: camera_id=2 직접 접근
def generate_camera_frames(camera_id):
    camera = cv2.VideoCapture(camera_id)
    if not camera.isOpened():
        app.logger.error(f"Camera {camera_id} could not be opened.")
        return

    while True:
        try:
            success, frame = camera.read()
            if not success:
                app.logger.warning(f'Failed to read frame from camera {camera_id}')
                break
            else:
                # 프레임 크기 조정 (인코딩 시간 단축)
                frame = cv2.resize(frame, (480, 360))  # 필요에 따라 해상도 조정
                # JPEG 인코딩 최적화 (속도 향상을 위해 품질 낮춤)
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]  # 50으로 낮춰 인코딩 속도 향상
                ret, buffer = cv2.imencode('.jpg', frame, encode_param)
                if not ret:
                    app.logger.warning(f'Failed to encode frame from camera {camera_id}')
                    continue
                frame = buffer.tobytes()
                # 프레임 전송
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        except Exception as e:
            app.logger.error(f"Error: {e}")
            break

# 저장 좌표 리스트
pt_1 = (460, 0)
pt_2 = (640, 0)
pt_3 = (640, 120)
pt_4 = (460, 120)
coordinates = [pt_1, pt_2, pt_3, pt_4]

# Flask 비디오 피드2: camera_id=2에 좌표 표시
def generate_frames_with_overlay(camera_id):
    camera = cv2.VideoCapture(camera_id)
    if not camera.isOpened():
        app.logger.error(f"Camera {camera_id} could not be opened.")
        return

    while True:
        try:
            success, frame = camera.read()
            if not success:
                app.logger.warning(f'Failed to read frame from camera {camera_id}')
                break
            else:
                # 프레임 크기 조정 (인코딩 시간 단축)
                frame = cv2.resize(frame, (480, 360))  # 필요에 따라 해상도 조정
                # 좌표에 원 그리기
                for (x, y) in coordinates:
                    cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)  # 반지름 5의 빨간 원
                # 사각형 그리기
                pts = np.array(coordinates, np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)  # 초록색 사각형
                # JPEG 인코딩 최적화 (속도 향상을 위해 품질 낮춤)
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]  # 50으로 낮춰 인코딩 속도 향상
                ret, buffer = cv2.imencode('.jpg', frame, encode_param)
                if not ret:
                    app.logger.warning(f'Failed to encode frame from camera {camera_id}')
                    continue
                frame = buffer.tobytes()
                # 프레임 전송
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        except Exception as e:
            app.logger.error(f"Error: {e}")
            break

# Flask 비디오 피드1 라우트 (ROS 토픽에서 이미지 수신)
@app.route('/video_feed1')
def video_feed1():
    return Response(generate_ros_frames(app.image_subscriber_node), mimetype='multipart/x-mixed-replace; boundary=frame')

# Flask 비디오 피드2 라우트 (camera_id=2 직접 접근)
@app.route('/video_feed2')
def video_feed2():
    return Response(generate_camera_frames(2), mimetype='multipart/x-mixed-replace; boundary=frame')

# Flask 비디오 피드2 라우트 (camera_id=2에 좌표 오버레이)
@app.route('/video_feed2_overlay')
def video_feed2_overlay():
    return Response(generate_frames_with_overlay(2), mimetype='multipart/x-mixed-replace; boundary=frame')

# 로그아웃 라우트
@app.route('/logout')
def logout():
    # 세션 정리 후 로그인 페이지로 리디렉션
    session.pop('username', None)
    flash('Logged out successfully!', 'info')
    return redirect(url_for('login'))

# Flask 애플리케이션 실행 함수
def run_flask_app():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

# 카메라 캡처 스레드 함수
def camera_capture_thread(camera_id):
    if camera_id == 2:
        # 오버레이가 필요한 경우
        generate_frames_with_overlay(camera_id)
    else:
        generate_camera_frames(camera_id)

# 메인 함수
def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    app.image_subscriber_node = node  # Flask 애플리케이션에 ROS 노드 인스턴스 연결

    # Flask 애플리케이션을 별도의 스레드에서 실행
    flask_thread = threading.Thread(target=run_flask_app)
    flask_thread.daemon = True
    flask_thread.start()

    # camera_id=2 캡처를 별도의 스레드에서 실행
    camera_thread = threading.Thread(target=generate_camera_frames, args=(2,))
    camera_thread.daemon = True
    camera_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
