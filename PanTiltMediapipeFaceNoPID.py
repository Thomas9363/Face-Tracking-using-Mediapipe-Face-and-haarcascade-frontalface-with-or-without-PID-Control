import cv2
import mediapipe as mp
import time 
mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16,address=0x6F) # Initialize the PCA9685 servo controller
PAN_CHANNEL,TILT_CHANNEL  = 0, 1 # Channels for servo control
pan_initial_angle=90.0 # Set the initial angles for servos
tilt_initial_angle=75.0
kit.servo[PAN_CHANNEL].angle = pan_initial_angle # Move servos to initial angles
kit.servo[TILT_CHANNEL].angle = tilt_initial_angle
cols, rows =640, 480 # Display window 640x480, 352x288 or 320x240

pan_angle = pan_initial_angle # servo initial position in degree
tilt_angle= tilt_initial_angle 
#offset=cols//16 
offset=cols//40


# For webcam input:
cap = cv2.VideoCapture(0)
cap.set(3, cols)
cap.set(4, rows)

pTime = 0.0 #time tracking for FPS
cTime = 0.0


with mp_face_detection.FaceDetection(
    model_selection=0, min_detection_confidence=0.5) as face_detection:
    while cap.isOpened():
        success, image = cap.read()
        image=cv2.flip(image, 0)
        if not success:
            print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
            continue

    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = face_detection.process(image)

    # Draw the face detection annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results.detections:
            for detection in results.detections:
        #mp_drawing.draw_detection(image, detection)
                location_data = detection.location_data
                if location_data.format == location_data.RELATIVE_BOUNDING_BOX:
                    bb = location_data.relative_bounding_box
                bb_box = [
                bb.xmin, bb.ymin,
                bb.width, bb.height,
              ]
#                    shape=image.shape
                relative_x = int(bb.xmin * cols)
                relative_y = int(bb.ymin * rows)
                relative_x_w = int((bb.xmin+bb.width) * cols)
                relative_y_h = int((bb.ymin+bb.height) * rows)
                face_center_x=(relative_x+relative_x_w)//2
                face_center_y=(relative_y+relative_y_h)//2
                pan_error=face_center_x-cols//2
                tilt_error=face_center_y-rows//2
                if pan_error >offset:
                    pan_angle += 0.3 #0.3
                    kit.servo[0].angle = pan_angle
                if pan_error< -offset:
                    pan_angle -= 0.3
                    kit.servo[0].angle = pan_angle
                if pan_error< -offset:
                    pan_angle -= 0.3
                    kit.servo[0].angle = pan_angle
                print(face_center_x, ",", "{0:.2f}".format(pan_angle))
                if tilt_error > offset:
                    tilt_angle += 0.3
                if tilt_error< -offset:
                    tilt_angle -= 0.3
                kit.servo[1].angle = tilt_angle
                cv2.rectangle(image,(cols//2-offset, rows//2-offset),(cols//2+offset, rows//2+offset),(0,255,255),1) #draw bounding box of face (yellow)          

                print(relative_x, relative_y,relative_x_w, relative_y_h)
#           cv2.circle(image, (relative_x, relative_y), 5, (0, 255, 255),3)
#           cv2.circle(image, (relative_x_w, relative_y_h), 5, (0, 255, 255), 3)
                cv2.circle(image, (face_center_x, face_center_y),50, 
                        (0, 255, 255), 1)  # draw bounding box of face (yellow)

#           cv2.rectangle(image, (relative_x, relative_y), (relative_x_w, relative_y_h),
#                         (0, 255, 255), 1)  # draw bounding box of face (yellow)
                cv2.line(image, (face_center_x, 0), (face_center_x, rows), (0, 255, 255), 1)
                cv2.line(image, (0, face_center_y), (cols,face_center_y), (0, 255, 255), 1) 
        cTime = time.time() # calculate and display FPS
        fps = 1/(cTime-pTime)
        pTime = cTime
        cv2.putText(image, f"FPS : {int(fps)}", (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2) 


    #      print(f"RBBox: {bb_box}")
    # Flip the image horizontally for a selfie-view display.
    # cv2.flip(image, 1)
        cv2.imshow("Frame", image)
    #cv2.imshow('MediaPipe Face Detection', cv2.flip(image, 1))
        if cv2.waitKey(5) & 0xFF == 27:
            break
kit.servo[0].angle = pan_initial_angle  # Reset the servos to their initial positions before exiting the program
kit.servo[1].angle = tilt_initial_angle


cap.release()
cv2.destroyAllWindows()
