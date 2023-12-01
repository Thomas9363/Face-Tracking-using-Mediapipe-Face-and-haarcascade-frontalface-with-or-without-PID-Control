import cv2
import numpy as np
import time 
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16,address=0x6F) # Initialize the PCA9685 servo controller
PAN_CHANNEL,TILT_CHANNEL  = 0, 1 # Channels for servo control
pan_initial_angle=90.0 # Set the initial angles for servos
tilt_initial_angle=75.0
kit.servo[PAN_CHANNEL].angle = pan_initial_angle # Move servos to initial angles
kit.servo[TILT_CHANNEL].angle = tilt_initial_angle
faceCascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml') # Model used
angle_increment=0.3
# faceCascade = cv2.CascadeClassifier('aGest.xml') # Model used
cols, rows =352, 288 # Display window 640x480, 352x288 or 320x240

pan_angle = pan_initial_angle # servo initial position in degree
tilt_angle= tilt_initial_angle 
#offset=cols//16 
offset=cols//40
cap = cv2.VideoCapture(0) # Initialize the video capture
cap.set(3, cols)
cap.set(4, rows)

pTime = 0 #time tracking for FPS
cTime = 0
fps=0.0
try:
    while True:
        _,frame = cap.read() # Read the frame from the video capture 
        frame=cv2.flip(frame,0)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Convert the frame to grayscale for face detection
        faces = faceCascade.detectMultiScale(gray,scaleFactor=1.3,minNeighbors=5,minSize=(20, 20)) # Perform face detection
        largest_face = None # Track the largest face
        largest_face_area = 0
        for (x, y, w, h) in faces:
            face_area = w * h
            if face_area > largest_face_area:
                largest_face = (x, y, w, h)
                largest_face_area = face_area
        if largest_face is not None: # If a face is detected, perform tracking
            x, y, w, h = largest_face
            face_center_x = x + w // 2 # Calculate the center of the face
            face_center_y = y + h // 2

            pan_error=face_center_x-cols//2
            tilt_error=face_center_y-rows//2

            if pan_error >offset:
                pan_angle += angle_increment #0.3
                kit.servo[0].angle = pan_angle
            if pan_error< -offset:
                pan_angle -= angle_increment
                kit.servo[0].angle = pan_angle
#            print(face_center_x, ",", "{0:.2f}".format(pan_angle))
            if tilt_error > offset:
                tilt_angle += angle_increment
            if tilt_error< -offset:
                tilt_angle -= angle_increment
            kit.servo[1].angle = tilt_angle

            cv2.circle(frame, (face_center_x, face_center_y), cols//20, (0, 255, 255), 1)
            
#             cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,255),1) #draw bounding box of face (yellow)
            cv2.rectangle(frame,(cols//2-offset, rows//2-offset),(cols//2+offset, rows//2+offset),(0,255,255),1) #draw bounding box of face (yellow)

            cv2.line(frame, (face_center_x, 0), (face_center_x, rows), (0, 255, 255), 1) #draw moving cross (yellow)
            cv2.line(frame, (0, face_center_y), (cols, face_center_y), (0, 255, 255), 1) 

        cTime = time.time() # calculate and display FPS
        fps = 0.9*fps+0.1*(1/(cTime-pTime))
        print(int(fps))
        pTime = cTime
        cv2.putText(frame, f"FPS : {int(fps)}", (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2) 
        
        cv2.imshow("Frame", frame) # Show the frame
        
        if cv2.waitKey(1) & 0xFF == 27: # Break the loop when 'q' is pressed
            break
finally: 
    kit.servo[0].angle = pan_initial_angle  # Reset the servos to their initial positions before exiting the program
    kit.servo[1].angle = tilt_initial_angle

    cap.release() # Release the video capture and clean up
    cv2.destroyAllWindows()
