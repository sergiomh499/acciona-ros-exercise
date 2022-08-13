#!/usr/bin/env python3
# Importing Libraries
import rospy
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp
from math import hypot, atan2 
import numpy as np


if __name__ == '__main__':
    # Initializing the Model
    mpHands = mp.solutions.hands
    hands = mpHands.Hands(static_image_mode=False,
        # model_complexity=1,
        min_detection_confidence=0.75,
        min_tracking_confidence=0.75,
        max_num_hands=1)
    
    Draw = mp.solutions.drawing_utils

    # Initializing ROS
    # init turtle controller node
    rospy.init_node('camera_controller')
    rate = rospy.Rate(20.0)
    # init vel publisher
    vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1) 
    cmd = Twist()
    
    # start capturing video from webcam
    cap = cv2.VideoCapture(0)
    
    while not rospy.is_shutdown():
        # read video frame by frame
        result, frame = cap.read()

        if result:
            # flip image
            frame = cv2.flip(frame, 1)
        
            # convert BGR image to RGB image
            frameRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
            # process the RGB image
            Process = hands.process(frameRGB)
        
            landmarkList = []
            # if hands are present in image(frame)
            if Process.multi_hand_landmarks:
                # detect handmarks
                for handlm in Process.multi_hand_landmarks:
                    for _id, landmarks in enumerate(handlm.landmark):
                        # store height and width of image
                        height, width, color_channels = frame.shape
        
                        # calculate and append x, y coordinates
                        # of handmarks from image(frame) to lmList
                        x, y = int(landmarks.x*width), int(landmarks.y*height)
                        landmarkList.append([_id, x, y])
        
                    # draw Landmarks
                    Draw.draw_landmarks(frame, handlm,
                                        mpHands.HAND_CONNECTIONS)
        
            # if landmarks list is not empty
            if landmarkList != []:
                # store x,y coordinates of (tip of) thumb
                x_1, y_1 = landmarkList[4][1], landmarkList[4][2]
        
                # store x,y coordinates of (tip of) index finger
                x_2, y_2 = landmarkList[8][1], landmarkList[8][2]
        
                # draw circle on thumb and index finger tip
                cv2.circle(frame, (x_1, y_1), 7, (0, 255, 0), cv2.FILLED)
                cv2.circle(frame, (x_2, y_2), 7, (0, 255, 0), cv2.FILLED)
        
                # draw line from tip of thumb to tip of index finger
                cv2.line(frame, (x_1, y_1), (x_2, y_2), (0, 255, 0), 3)
        
                # calculate square root of the sum of squares and atan2 of the specified arguments
                L = hypot(x_2-x_1, y_2-y_1)
                A = atan2(x_2-x_1, y_2-y_1)

                # 1-D linear interpolation
                vel_level = np.interp(L, [15, 220], [0, 100])

                # calculate velocity
                vel_lineal = 1/80 * vel_level
                if abs(1/A) > 0.3:
                    vel_angular = - 3 * 1/A
                else:
                    vel_angular = 0

                # limitation of angular velocity
                if abs(vel_angular) > 3.0:
                    vel_angular = vel_angular/abs(vel_angular) * 3.0

                # publish velocity
                cmd.linear.x = vel_lineal
                cmd.angular.z = vel_angular
                vel_pub.publish(cmd)
        
            # display Video and when 'q' is entered, destroy the window
            cv2.imshow('Image', frame)
            if cv2.waitKey(1) & 0xff == ord('q'):
                break
            
        else:
            print("Not frame received.")
    
        rate.sleep()