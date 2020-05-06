#!/usr/bin/env python

import rospy
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import TwistStamped, Vector3
import time
import pygame
import pygame.locals

rospy.init_node('mavros_final_project')
rate = rospy.Rate(10)

commandVelocityPub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 10)
setVelocity = TwistStamped()

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
    print("Taking off: takeoff 1")
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


if __name__=="__main__":
    # Setting up pygame
    pygame.init()
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    WIDTH = 600
    HEIGHT = 500

    pygame.display.set_caption('Drone Control')
    windowSurface = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)
    windowSurface.fill(BLACK)
    font = pygame.font.Font(pygame.font.get_default_font(), 10)
    myText = 't: Takeoff, l: Land, w: Forward, a: Left, s: Backward, d: Right, q: Up, e: Down, z: RotateLeft, x: RotateRight'
    myTextSurface = font.render(myText, True, WHITE)
    textRect = myTextSurface.get_rect()
    textRect.center = (WIDTH // 2, HEIGHT // 2)
    windowSurface.blit(myTextSurface, textRect)
  
    pygame.display.update()  

    while True:
        setVelocity.twist.linear = Vector3(x = 0, y = 0, z = 0)
        setVelocity.twist.angular = Vector3(x = 0, y = 0, z = 0)

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
                    # move forward
                    setVelocity.twist.linear.y = 0.5
                if event.key == pygame.K_a:
                    # move left
                    setVelocity.twist.linear.x = -0.5
                if event.key == pygame.K_d:
                    # move right
                    setVelocity.twist.linear.x = 0.5
                if event.key == pygame.K_s:
                    # move backward
                    setVelocity.twist.linear.y = -0.5
                if event.key == pygame.K_q:
                    # move up
                    setVelocity.twist.linear.z = 0.5
                if event.key == pygame.K_e:
                    # move down
                    setVelocity.twist.linear.z = -0.5
                if event.key == pygame.K_z:
                    # rotate left
                    setVelocity.twist.angular.z = 0.5
                if event.key == pygame.K_x:
                    # rotate right
                    setVelocity.twist.angular.z = -0.5

                # publishing the TwistStamped message    
                commandVelocityPub.publish(setVelocity)
                
            elif event.type == pygame.QUIT:
                quit()