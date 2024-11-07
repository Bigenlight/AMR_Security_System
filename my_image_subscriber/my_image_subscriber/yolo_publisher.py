import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time
from flask import Flask, Response

app = Flask(__name__)

def run_flask_app():
    app.run(host='0.0.0.0', port=5000, debug=False)

@app.route('/')
def index():
    return "System Monitor"

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def generate_frames():
    while True:
        with ImageSubscriber.lock:
            if ImageSubscriber.latest_frame is not None:
                ret, buffer = cv2.imencode('.jpg', ImageSubscriber.latest_frame)
                frame = buffer.tobytes()
            else:
                continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.1)

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

        # Start Flask app in a separate thread
        flask_thread = threading.Thread(target=run_flask_app)
        flask_thread.daemon = True
        flask_thread.start()

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with ImageSubscriber.lock:
            ImageSubscriber.latest_frame = cv_image.copy()
        cv2.imshow("Processed Image", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
