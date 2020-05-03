#!/usr/bin/env python

import rospy
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
import time

rospy.init_node('mavros_final_project')
rate = rospy.Rate(10)

def setMode():
    print("Setting Mode to Guided: mode guided")
    rospy.wait_for_service('/mavros/set_mode')
    try: 
        mavSetMode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        mavSetMode(custom_mode="Guided")
    except rospy.ServiceException as e:
        print(e)

def armCopter():
    print("Arming throttle: arm throttle")
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        mavArm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        mavArm(value = True)
    except rospy.ServiceException as e:
        print(e)

def takeOff():
    print("Taking off: takeoff 10")
    try:
        mavTakeOff = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        mavTakeOff(altitude=10)
    except rospy.ServiceException as e:
        print(e)

def land():
    print("Landing: mode land")
    try:
        mavLand = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        mavLand(altitude=10)
    except rospy.ServiceException as e:
        print(e)

def disarmCopter():
    print("Disarming throttle: disarm")
    try:
        mavDisarm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        mavDisarm(value = False)
    except rospy.ServiceException as e:
        print(e)

if __name__=="__main__":
    setMode()
    armCopter()
    takeOff()
    time.sleep(10) # hover for 10 ms
    land()
    disarmCopter()
    print("Done")
