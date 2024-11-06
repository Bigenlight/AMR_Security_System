import cv2
from ultralytics import YOLO

# Load the YOLOv8 model (choose from 'yolov8n.pt', 'yolov8s.pt', etc.)
model = YOLO('best.pt')

# Initialize the webcam
cap = cv2.VideoCapture(0)  # Use 0 for the default webcam

# Check if the webcam is opened correctly
if not cap.isOpened():
    print("Cannot open webcam")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Cannot receive frame. Exiting...")
        break

    # Run YOLOv8 inference on the frame
    results = model(frame)

    # Visualize the results on the frame
    annotated_frame = results[0].plot()

    # Display the resulting frame
    cv2.imshow('YOLOv8 Real-Time Detection', annotated_frame)

    # Exit the loop when 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
