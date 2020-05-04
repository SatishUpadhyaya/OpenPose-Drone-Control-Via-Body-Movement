#!/usr/bin/env python

import rospy
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import TwistStamped
import time
import pygame, sys
import pygame.locals

rospy.init_node('mavros_final_project')
rate = rospy.Rate(10)

commandVelocityPub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 10)

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
        mavTakeOff(altitude=1)
    except rospy.ServiceException as e:
        print(e)

def land():
    print("Landing: mode land")
    try:
        mavLand = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        mavLand(altitude=1)
    except rospy.ServiceException as e:
        print(e)

def disarmCopter():
    print("Disarming throttle: disarm")
    try:
        mavDisarm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        mavDisarm(value = False)
    except rospy.ServiceException as e:
        print(e)

def moveLeft():
    print("Move Left:")
    setVelocity = TwistStamped()
    setVelocity.twist.linear.x = -0.5
    commandVelocityPub.publish(setVelocity)

def moveRight():
    print("Move Right:")
    setVelocity = TwistStamped()
    setVelocity.twist.linear.x = 0.5
    commandVelocityPub.publish(setVelocity)

def moveForward():
    print("Move Forward:")
    setVelocity = TwistStamped()
    setVelocity.twist.linear.y = 0.5
    commandVelocityPub.publish(setVelocity)

def moveBackward():
    print("Move Back:")
    setVelocity = TwistStamped()
    setVelocity.twist.linear.y = -0.5
    commandVelocityPub.publish(setVelocity)

def moveUp():
    print("Move Up:")
    setVelocity = TwistStamped()
    setVelocity.twist.linear.z = 0.5
    commandVelocityPub.publish(setVelocity)

def moveDown():
    print("Move Down:")
    setVelocity = TwistStamped()
    setVelocity.twist.linear.z = -0.5
    commandVelocityPub.publish(setVelocity)

def rotateRight():
    print("Rotate Right:")
    setVelocity = TwistStamped()
    setVelocity.twist.angular.z = 0.5
    commandVelocityPub.publish(setVelocity)

def rotateLeft():
    print("Rotate Left:")
    setVelocity = TwistStamped()
    setVelocity.twist.angular.z = -0.5
    commandVelocityPub.publish(setVelocity)

if __name__=="__main__":
    print("MAIN FUNCTION")
    pygame.init()
    BLACK = (0, 0, 0)
    WIDTH = (500)
    HEIGHT = (500)
    windowSurface = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)
    windowSurface.fill(BLACK)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_t:
                    setMode()
                    armCopter()
                    takeOff()
                    time.sleep(5)
                if event.key == pygame.K_l:
                    land()
                    disarmCopter()
                    time.sleep(5)
                if event.key == pygame.K_w:
                    moveForward()
                if event.key == pygame.K_a:
                    moveLeft()
                if event.key == pygame.K_d:
                    moveRight()
                if event.key == pygame.K_s:
                    moveBackward()
                if event.key == pygame.K_q:
                    moveUp()
                if event.key == pygame.K_e:
                    moveDown()
                if event.key == pygame.K_z:
                    rotateLeft()
                if event.key == pygame.K_x:
                    rotateRight()
            elif event.type == pygame.QUIT:
                quit()