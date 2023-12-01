import cv2
import mediapipe as mp
import numpy as np

# Initialize MediaPipe FaceMesh and drawing modules
mp_face_mesh = mp.solutions.face_mesh
mp_drawing = mp.solutions.drawing_utils

# Create a black background image
height, width = 480, 640  # Adjust the size as needed
black_background = np.zeros((height, width, 3), dtype=np.uint8)

# Create a FaceMesh object
face_mesh = mp_face_mesh.FaceMesh(static_image_mode=False, max_num_faces=1, min_detection_confidence=0.5)

# Capture video from webcam (you can replace this with your video source)
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Flip the frame horizontally for a later selfie-view display
    frame = cv2.flip(frame, 1)

    # Create a black background image for each frame
    black_background = np.zeros((height, width, 3), dtype=np.uint8)

    # Process the frame to get face landmarks
    results = face_mesh.process(frame)

    # Draw only face mesh lines on the black background
    if results.multi_face_landmarks and len(results.multi_face_landmarks[0].landmark) > 0:
        # Draw only face mesh lines without landmarks
        for connection in mp_face_mesh.FACEMESH_TESSELATION:
            start_point = connection[0]
            end_point = connection[1]
            start_landmark = results.multi_face_landmarks[0].landmark[start_point]
            end_landmark = results.multi_face_landmarks[0].landmark[end_point]
            start_point_px = (int(start_landmark.x * width), int(start_landmark.y * height))
            end_point_px = (int(end_landmark.x * width), int(end_landmark.y * height))
            cv2.line(black_background, start_point_px, end_point_px, (255, 255, 255), 1)

    # Display the result
    cv2.imshow('Face Mesh', black_background)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
