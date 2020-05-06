#!/usr/bin/env python

import numpy as np
import cv2
import math
import rospy 
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import TwistStamped, Vector3
from keyboardControl import setMode, armCopter, takeOff, land, disarmCopter, commandVelocityPub, setVelocity
import time

# this arcuo ID can be found at: http://chev.me/arucogen
arucoID = 72 # ID: 72
markerSize = 10 # 100 mm
cameraMatrix = np.loadtxt('cameraMatrix.txt', delimiter=',')
cameraDistortion = np.loadtxt('cameraDistortion.txt', delimiter=',')
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL) # aruco auto dict
parameters = cv2.aruco.DetectorParameters_create()


if __name__=="__main__":
    # setting up opencv and aruco
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    font = cv2.FONT_HERSHEY_SIMPLEX

    setMode()
    armCopter()
    takeOff()
    time.sleep(5)

    while True:
        setVelocity.twist.linear = Vector3(x = 0, y = 0, z = 0)
        setVelocity.twist.angular = Vector3(x = 0, y = 0, z = 0)

        ret, frame = cap.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        corners, ids, _ = cv2.aruco.detectMarkers(image=gray, dictionary=arucoDict, parameters=parameters, cameraMatrix=cameraMatrix, distCoeff=cameraDistortion)
        
        if ids is not None and ids[0] == arucoID:
            ret = cv2.aruco.estimatePoseSingleMarkers(corners, markerSize, cameraMatrix, cameraDistortion)

            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            x, y, z = tvec[0], tvec[1], tvec[2]

            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.aruco.drawAxis(frame, cameraMatrix, cameraDistortion, rvec, tvec, 10)

            myString = f"Aruco: x={x}  y={y}  z={z}"
            cv2.putText(frame, myString, (0, 100), font, 1, (255, 255, 255), 2, cv2.LINE_AA)

            if x > 10:
                # move left
                setVelocity.twist.linear.x = -0.5
            if x < -10:
                # move right
                setVelocity.twist.linear.x = 0.5
            if y < -8:
                # move up
                setVelocity.twist.linear.z = 0.5
            if y > 6:
                # move down
                setVelocity.twist.linear.z = -0.5
            if z < 27:
                # move forward
                setVelocity.twist.linear.y = 0.5
            if z > 50:
                # move backward
                setVelocity.twist.linear.y = -0.5
            
            # publishing the TwistStamped message    
            commandVelocityPub.publish(setVelocity)

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break
    
    land()
    disarmCopter()
    time.sleep(5)