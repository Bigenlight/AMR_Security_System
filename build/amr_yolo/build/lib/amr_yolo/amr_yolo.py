import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO
import cv2
import threading
import time
import json

class AmrYoloNode(Node):
    def __init__(self, model_path):
        super().__init__('amr_yolo')
        
        # Publisher
        self.detection_publisher = self.create_publisher(String, 'detection_info_arm', 10)
        
        # YOLO Model
        self.model = YOLO(model_path)
        
        # Camera Setup (Using cam0)
        self.cap = cv2.VideoCapture(0)  # Use cam0; adjust if necessary
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Optional: Set width
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Optional: Set height
        
        # Control flags
        self.lock = threading.Lock()
        self.current_frame = None
        
        # Start threads
        threading.Thread(target=self.capture_frames, daemon=True).start()
        threading.Thread(target=self.process_frames, daemon=True).start()
        
    def capture_frames(self):
        """Continuously capture frames from the camera in a separate thread."""
        while True:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.current_frame = frame.copy()
            time.sleep(0.01)  # Slight delay to control frame rate
    
    def process_frames(self):
        """Process frames for object detection in a separate thread."""
        while True:
            if self.current_frame is not None:
                with self.lock:
                    frame_to_process = self.current_frame.copy()
                
                detection_info_list = []  # List to store detection info
                
                # Run YOLO object detection
                results = self.model(frame_to_process)
                for r in results:
                    for box in r.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cls = int(box.cls[0])
                        detection_info = {
                            'box_coordinates': [x1, y1, x2, y2],
                            'class_number': cls
                        }
                        detection_info_list.append(detection_info)
                
                # Publish the detection info
                if detection_info_list:
                    detection_info_msg = String()
                    detection_info_msg.data = json.dumps(detection_info_list)
                    self.detection_publisher.publish(detection_info_msg)
                
            time.sleep(0.01)  # Adjust delay if necessary to control processing rate
    
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()
    
def main(args=None):
    rclpy.init(args=args)
    
    # Parse command-line arguments for model path
    import argparse
    parser = argparse.ArgumentParser(description='AmrYolo Node')
    parser.add_argument('--user', type=str, default='rokey', help='Path to the YOLO model')
    args, unknown = parser.parse_known_args()
    
    #model_path = parsed_args.model
    # 모델 경로 생성
    model_path = f'/home/{args.user}/1_ws/src/best.pt'
    
    node = AmrYoloNode(model_path)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
