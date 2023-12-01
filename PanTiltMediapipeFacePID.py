import cv2
import mediapipe as mp
import time
import numpy as np
mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16,address=0x6F) # Initialize the PCA9685 servo controller
PAN_CHANNEL,TILT_CHANNEL, LASER_CHANNEL    = 0, 1, 15 # Channels for servo control
pan_initial_angle=90.0 # Set the initial angles for servos
tilt_initial_angle=75.0

PAN_ANGLE_MIN, PAN_ANGLE_MAX = 5, 175 # Servo angles range of motion
TILT_ANGLE_MIN, TILT_ANGLE_MAX = 50, 120

kit.servo[PAN_CHANNEL].angle = pan_initial_angle # Move servos to initial angles
kit.servo[TILT_CHANNEL].angle = tilt_initial_angle
kit._pca.channels[LASER_CHANNEL].duty_cycle = 0 #from 0 to 65535
# cols, rows =640, 480 # Display window 640x480, 352x288 or 320x240
cols, rows =640, 480 # Display window 640x480, 352x288 or 320x240

pan_angle = pan_initial_angle # servo initial position in degree
tilt_angle= tilt_initial_angle 
#offset=cols//16 
setpoint=cols//40
# PAN_KP,PAN_KI,PAN_KD = 0.0163, 0.0, 0.0 # PID for 352x288
# TILT_KP,TILT_KI,TILT_KD = 0.0145, 0.0, 0.0 
PAN_KP,PAN_KI,PAN_KD = 0.0081, 0.0, 0.0 # PID for 640x480
TILT_KP,TILT_KI,TILT_KD = 0.008, 0.0, 0.0 

pan_integral = 0.0 # Initialize the PID controllers for pan and tilt
pan_last_time = time.time()
tilt_integral = 0.0
tilt_last_time = time.time()
pan_error_prior = 0.0 # Initialize the initial error values
tilt_error_prior = 0.0
panTarget, tiltTarget=0, 0 # how many count in target area


def calculate_pid(error, kp, ki, kd, integral, last_time, error_prior): # PID control function
    current_time = time.time()
    delta_time = current_time - last_time

    proportional = error
    integral += error * delta_time
    derivative = (error - error_prior) / delta_time

    output = kp * proportional + ki * integral + kd * derivative

    return output, integral, current_time


# For webcam input:
cap = cv2.VideoCapture(0)
# cap = cv2.VideoCapture(0)
cap.set(3, cols)
cap.set(4, rows)

pTime = 0.0 #time tracking for FPS
cTime = 0.0
fps=0

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
                pan_error = face_center_x - cols // 2
                tilt_error = face_center_y - rows // 2

                if abs(pan_error)> setpoint: # Perform PID control to calculate pan and tilt angles
                    pan_output, pan_integral, pan_last_time = calculate_pid(pan_error, PAN_KP, PAN_KI, PAN_KD, pan_integral, pan_last_time, pan_error_prior)
                    pan_angle = np.clip(pan_angle + pan_output, PAN_ANGLE_MIN, PAN_ANGLE_MAX)
                    kit.servo[0].angle =pan_angle
                    panTrigger=False
                    panTarget=0
#                     print(pan_error)np.clip
                else:
                    panTrigger=True
                    panTarget=panTarget+1
                if abs(tilt_error)>setpoint:
                    tilt_output, tilt_integral, tilt_last_time = calculate_pid(tilt_error, TILT_KP, TILT_KI, TILT_KD, tilt_integral, tilt_last_time, tilt_error_prior)
                    tilt_angle = np.clip(tilt_angle + tilt_output, TILT_ANGLE_MIN, TILT_ANGLE_MAX) # Adjust tilt angles based on PID output
                    kit.servo[1].angle =tilt_angle
                    tiltTrigger=False
                    tiltTarget=0
                else:
                    tiltTrigger=True # adding one to the tilt count
                    tiltTarget=tiltTarget+1
                if panTarget>setpoint and tiltTarget>setpoint: # if both count reach 50, fire laser
                    kit._pca.channels[LASER_CHANNEL].duty_cycle = 65535
                else: 
                    kit._pca.channels[LASER_CHANNEL].duty_cycle = 0


                pan_error_prior = pan_error # Update the error_prior values for the next iteration
                tilt_error_prior = tilt_error

                cv2.rectangle(image,(cols//2-setpoint, rows//2-setpoint),(cols//2+setpoint, rows//2+setpoint),(0,255,0),1) #draw bounding box of face (yellow)          

#                 print(relative_x, relative_y,relative_x_w, relative_y_h)
#           cv2.circle(image, (relative_x, relative_y), 5, (0, 255, 255),3)
#           cv2.circle(image, (relative_x_w, relative_y_h), 5, (0, 255, 255), 3)
                cv2.circle(image, (face_center_x, face_center_y),50, 
                        (0, 255, 255), 1)  # draw bounding box of face (yellow)

#           cv2.rectangle(image, (relative_x, relative_y), (relative_x_w, relative_y_h),
#                         (0, 255, 255), 1)  # draw bounding box of face (yellow)
                cv2.line(image, (face_center_x, 0), (face_center_x, rows), (0, 255, 255), 1)
                cv2.line(image, (0, face_center_y), (cols,face_center_y), (0, 255, 255), 1) 
        cTime = time.time() # calculate and display FPS
        fps = 0.9*fps+0.1*(1/(cTime-pTime))
        pTime = cTime
        cv2.putText(image, f"FPS : {int(fps)}", (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2) 
#         cv2.putText(image, f"FPS : {int(fps)}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 4) 


    #      print(f"RBBox: {bb_box}")
    # Flip the image horizontally for a selfie-view display.
    # cv2.flip(image, 1)
        cv2.imshow("Frame", image)
    #cv2.imshow('MediaPipe Face Detection', cv2.flip(image, 1))
        if cv2.waitKey(5) & 0xFF == 27:
            break
kit.servo[0].angle = pan_initial_angle  # Reset the servos to their initial positions before exiting the program
kit.servo[1].angle = tilt_initial_angle
kit._pca.channels[LASER_CHANNEL].duty_cycle = 0

cap.release()
cv2.destroyAllWindows()
