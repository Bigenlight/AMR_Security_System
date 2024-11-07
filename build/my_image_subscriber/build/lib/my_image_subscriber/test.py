import cv2

def test_camera(camera_id):
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Camera {camera_id} is not opened")
        return
    print(f"Camera {camera_id} is opened successfully.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"Failed to grab frame from camera {camera_id}")
            break
        cv2.imshow(f"Camera {camera_id}", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_camera(2)
    #test_camera(0) # 두 번째 카메라가 있는 경우 주석을 해제하세요
