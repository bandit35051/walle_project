import cv2
import time
import threading
import numpy as np

# Import shared state if available; ensure wall_e_state is defined in your memory module.
try:
    from memory import wall_e_state
except ImportError:
    wall_e_state = {}  # Fallback if not available

# Shared frame and lock for thread-safe access
shared_frame = None
frame_lock = threading.Lock()

def camera_capture():
    """
    Continuously captures frames from the camera and stores a copy in shared_frame.
    This function should be started in a background thread.
    """
    global shared_frame
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if ret:
            with frame_lock:
                shared_frame = frame.copy()
        time.sleep(0.05)
    cap.release()

def get_shared_frame():
    """Return a copy of the current shared frame, or None if no frame is available."""
    with frame_lock:
        return shared_frame.copy() if shared_frame is not None else None

# SLAM Navigation using a shared camera frame
def slam_navigation(orb):
    """
    Performs SLAM navigation using a provided ORB detector.
    The ORB detector should be instantiated in your main module and passed in.
    """
    while True:
        frame = get_shared_frame()
        if frame is None:
            time.sleep(0.01)
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = orb.detectAndCompute(gray, None)
        frame_with_keypoints = cv2.drawKeypoints(gray, keypoints, None, color=(0, 255, 0), flags=0)
        cv2.imshow("SLAM Navigation", frame_with_keypoints)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()

# Facial Recognition using the shared frame
def detect_faces():
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    while True:
        frame = get_shared_frame()
        if frame is None:
            time.sleep(0.01)
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.1, 4)
        # Update shared state if available
        if 'faces_detected' in wall_e_state:
            wall_e_state['faces_detected'] = len(faces)
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.imshow('Face Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()

# Object Recognition using the shared frame
def detect_objects():
    net = cv2.dnn.readNetFromCaffe('MobileNetSSD_deploy.prototxt', 'MobileNetSSD_deploy.caffemodel')
    CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
               "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
               "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
               "sofa", "train", "tvmonitor"]
    while True:
        frame = get_shared_frame()
        if frame is None:
            time.sleep(0.01)
            continue
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)
        net.setInput(blob)
        detections = net.forward()
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.5:
                idx = int(detections[0, 0, i, 1])
                label = CLASSES[idx]
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)
                cv2.putText(frame, f"{label}: {confidence * 100:.2f}%", (startX, startY - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.imshow("Object Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()

# Launch face and object detection in background threads
threading.Thread(target=detect_faces, daemon=True).start()
threading.Thread(target=detect_objects, daemon=True).start()

# Note: Do not start camera_capture() here. It should be started from your main module.
