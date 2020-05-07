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
# Helpful video on the topic: https://www.youtube.com/watch?v=wlT_0fhGrGg
arucoID = 72 # ID: 72
markerSize = 10 # 100 mm
cameraMatrix = np.loadtxt('cameraMatrix.txt', delimiter=',')
cameraDistortion = np.loadtxt('cameraDistortion.txt', delimiter=',')
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL) # aruco auto dict
parameters = cv2.aruco.DetectorParameters_create()

RotationMatrix = np.matrix([[1., 0., 0.], [0., -1., 0.], [0., 0., -1.]])

# Code for Rotation matrix to Euler Angles
# from https://www.learnopencv.com/rotation-matrix-to-euler-angles/
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


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

            _, _, yaw = rotationMatrixToEulerAngles(RotationMatrix*np.matrix(cv2.Rodrigues(rvec)[0]).T)
            yaw = math.degrees(yaw)

            myString = f"Aruco: x={int(x)}  y={int(y)}  z={int(z)} yaw = {int(yaw)}"
            cv2.putText(frame, myString, (0, 100), font, 1, (255, 255, 255), 2, cv2.LINE_AA)

            if x > 10:
                # move left
                setVelocity.twist.linear.y = 1
            if x < -15:
                # move right
                setVelocity.twist.linear.y = -1
            if y < -8:
                # move up
                setVelocity.twist.linear.z = 0.5
            if y > 6:
                # move down
                setVelocity.twist.linear.z = -0.5
            if z < 30:
                # move forward
                setVelocity.twist.linear.x = 1
            if z > 50:
                # move backward
                setVelocity.twist.linear.x = -1
            if yaw > 25:
                # rotate right
                setVelocity.twist.angular.z = -0.5
            if yaw < -25:
                # rotate left
                setVelocity.twist.angular.z = 0.5
            
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