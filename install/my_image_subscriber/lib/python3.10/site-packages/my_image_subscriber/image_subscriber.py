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

# ROS ImageSubscriber Node
class ImageSubscriber(Node):
    latest_frame = None
    lock = threading.Lock()

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'processed_image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with ImageSubscriber.lock:
            ImageSubscriber.latest_frame = cv_image.copy()
        cv2.imshow("Processed Image", cv_image)
        cv2.waitKey(1)

# Get package share directory for templates
package_name = 'my_image_subscriber'
package_share_directory = get_package_share_directory(package_name)
template_dir = os.path.join(package_share_directory, 'templates')

# Initialize Flask app with template folder
app = Flask(__name__, template_folder=template_dir)
app.secret_key = 'your_secret_key'  # Needed to secure sessions

# Hardcoded user credentials for demonstration
USERNAME = 'user'
PASSWORD = 'password'

# Flask routes
@app.route('/')
def home():
    # If user is logged in, redirect to welcome page
    if 'username' in session:
        return redirect(url_for('welcome'))
    # Otherwise, show the login page
    return redirect(url_for('login'))

@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        # Retrieve form data
        username = request.form['username']
        password = request.form['password']
        
        # Check credentials
        if username == USERNAME and password == PASSWORD:
            # Store username in session and redirect to welcome
            session['username'] = username
            flash('Login successful!', 'success')
            return redirect(url_for('welcome'))
        else:
            flash('Invalid username or password!', 'danger')
            return redirect(url_for('login'))
    
    # Display login page for GET requests
    return render_template('login_center.html')

@app.route('/welcome')
def welcome():
    # Ensure user is logged in
    if 'username' not in session:
        flash('Please log in first!', 'warning')
        return redirect(url_for('login'))
    # Render welcome page
    return render_template('welcome_center_two_cam.html', username=session['username'])

def generate_frames(camera_id):
    camera = cv2.VideoCapture(camera_id)  # 0 is the default camera
    if not camera.isOpened():
        app.logger.error(f"Camera {camera_id} could not be opened.")
        return

    while True:
        # Read the camera frame
        success, frame = camera.read()
        if not success:
            break
        else:
            # Encode the frame in JPEG format
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            # Concatenate frame bytes with multipart data structure
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# List of store coordinates
pt_1 = (460, 0)
pt_2 = (640, 0)
pt_3 = (640, 120)
pt_4 = (460, 120)
coordinates = [pt_1, pt_2, pt_3, pt_4]

def generate_frames_box(camera_id):
    camera = cv2.VideoCapture(camera_id)
    if not camera.isOpened():
        app.logger.error(f"Camera {camera_id} could not be opened.")
        return

    while True:
        success, frame = camera.read()
        app.logger.warning(f"Failed to read frame from camera {camera_id}")
        if not success:
            break
        else:
            # Draw each collected coordinate on the frame
            for (x, y) in coordinates:
                cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)  # Red circle with radius 5
            
            # Convert coordinates to the format required by cv2.polylines
            pts = np.array(coordinates, np.int32)
            pts = pts.reshape((-1, 1, 2))

            # Draw the quadrilateral
            cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed1')
def video_feed1():
    return Response(generate_frames_box(0), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed2')
def video_feed2():
    return Response(generate_frames(2), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/logout')
def logout():
    # Clear the session and redirect to login
    session.pop('username', None)
    flash('Logged out successfully!', 'info')
    return redirect(url_for('login'))

# Function to run Flask app
def run_flask_app():
    app.run(host='0.0.0.0', port=5000, debug=False)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()

    # Start Flask app in a separate thread
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
