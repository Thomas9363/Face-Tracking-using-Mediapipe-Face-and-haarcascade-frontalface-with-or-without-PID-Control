import cv2
import numpy as np
import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16, address=0x6F) # Initialize servo controller

PAN_CHANNEL,TILT_CHANNEL, LASER_CHANNEL  = 0, 1, 15 # Channels for servo control

PAN_ANGLE_MIN, PAN_ANGLE_MAX = 5, 175 # Servo angles range of motion
TILT_ANGLE_MIN, TILT_ANGLE_MAX = 50, 120

pan_angle = 90 # Set initial angles for servos
tilt_angle = 75
kit.servo[PAN_CHANNEL].angle = pan_angle # Move servos to initial angle
kit.servo[TILT_CHANNEL].angle = tilt_angle
kit._pca.channels[LASER_CHANNEL].duty_cycle = 0 #from 0 to 65535

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml') #Model used

cols, rows = 640, 480 # 640x480, (0.55 352x288) or( 0.5 320x240)  (0.8 512, 384)(0.7 448, 336)(0.6 384, 288)
setpoint=cols//40
# PAN_KP,PAN_KI,PAN_KD = 0.014, 0.0, 0.0 # for 352x288
# TILT_KP,TILT_KI,TILT_KD = 0.012, 0.0, 0.0 #0.12
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

cap = cv2.VideoCapture(0) # Initialize the video capture
cap.set(3, cols)  
cap.set(4, rows)  

pTime = 0.0 #time tracking for FPS
cTime = 0.0
fps=0.0

while True:
    ret, frame = cap.read() # Read the frame from the video capture
    frame = cv2.flip(frame, 0)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Convert the frame to grayscalen
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5) # Perform face detection
    for (x, y, w, h) in faces:
        face_center_x = x + w // 2 # Calculate the center of the face
        face_center_y = y + h // 2
        # Calculate error for pan and tilt
        pan_error = face_center_x - cols // 2
        tilt_error = face_center_y - rows // 2
#            print(format(pan_error, ".2f"))

        if abs(pan_error)> setpoint: # Perform PID control to calculate pan and tilt angles
            pan_output, pan_integral, pan_last_time = calculate_pid(pan_error, PAN_KP, PAN_KI, PAN_KD, pan_integral, pan_last_time, pan_error_prior)
            pan_angle = np.clip(pan_angle + pan_output, PAN_ANGLE_MIN, PAN_ANGLE_MAX)
            kit.servo[0].angle =pan_angle
#            print(pan_error)
        if abs(tilt_error)>setpoint:
            tilt_output, tilt_integral, tilt_last_time = calculate_pid(tilt_error, TILT_KP, TILT_KI, TILT_KD, tilt_integral, tilt_last_time, tilt_error_prior)
            tilt_angle = np.clip(tilt_angle + tilt_output, TILT_ANGLE_MIN, TILT_ANGLE_MAX) # Adjust tilt angles based on PID output
            kit.servo[1].angle =tilt_angle
            
        if abs(pan_error)<setpoint and abs(tilt_error)<setpoint: 
            inTarget=inTarget+1
            if inTarget>15: #in target for a period of time, say 15 counts
                kit._pca.channels[LASER_CHANNEL].duty_cycle = 65535
        else:
            inTarget=0 # reset target count if move out
            kit._pca.channels[LASER_CHANNEL].duty_cycle = 0
            
        pan_error_prior = pan_error # Update the error_prior values for the next iteration
        tilt_error_prior = tilt_error
        
        cv2.rectangle(frame,(cols//2-setpoint, rows//2-setpoint),(cols//2+setpoint, rows//2+setpoint),(0,255,0),1) #draw setpoint box
        cv2.line(frame, (face_center_x, 0), (face_center_x, rows), (0, 255, 255), 1) #draw moving cross
        cv2.line(frame, (0, face_center_y), (cols, face_center_y), (0, 255, 255), 1) 
        cv2.circle(frame, (face_center_x, face_center_y), 50, (0, 255, 255), 1)

        cTime = time.time() # calculate and display FPS
        fps = 0.9*fps+0.1*(1/(cTime-pTime))

        pTime = cTime
        cv2.putText(frame, f"FPS : {int(fps)}", (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        print(int(fps))
    cv2.imshow('Face Tracking', frame) # Show the frame
    
    if cv2.waitKey(1) & 0xFF == 27: # Break the loop when 'q' is pressed
        break

kit.servo[PAN_CHANNEL].angle = 90 # Reset the servos to their starting positions and turn off laser
kit.servo[TILT_CHANNEL].angle = 90
kit._pca.channels[LASER_CHANNEL].duty_cycle = 0
cap.release() # Release the video capture and clean up
cv2.destroyAllWindows()
