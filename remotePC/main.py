#!/usr/bin/env python
"""
This is for control all system.
writer : Miseon Kim
last update : 2020.11.15
"""
############# ROS #############
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
###############################
import cv2
import numpy as np
import math
import time #for FPS
import sys

############ SYSTEM MODE ############
mode_start = 100
mode_lineDetect = 0
mode_obstacle = 1
mode_obstacle_finish = 2
mode_parking = 3
mode_parking_end = 4
mode_parkingSign = 5
mode_rightSign = 6
mode_leftSign = 7
mode_Sign_end = 8
mode_Stop = 10
mode_finish = 20

########## ROS INIT ##########
rospy.init_node('main')
pubTolineDetect = rospy.Publisher('/lineInfo', Twist, queue_size=20)
pubToParking = rospy.Publisher('/parkingInfo', Twist, queue_size=20)
putToObstacle = rospy.Publisher('/obstacleInfo', Twist, queue_size=20)
##############################

parking = False

linear_x = 0
angular_z = 0

modeInfo = Twist()
mode = mode_lineDetect
def checkMode(mode_):
    global mode
    global parking
    mode = mode_.linear.y  
    if parking==False and mode==mode_parkingSign:
        modeInfo.linear.y = mode_parkingSign
        print('Detect Parking Sign!!: %d' %mode)

    elif mode==mode_parking:
        modeInfo.linear.y = mode_parking
        print('READY TO PARK!!: %d' %mode)

    elif mode==mode_parking_end:
        parking = True
        mode = mode_lineDetect
        modeInfo.linear.y = mode_lineDetect
        print('Parking finished: %d' %mode)

    elif mode==mode_rightSign:
        modeInfo.linear.y = mode_rightSign
        print('Detect Right Sign!!: %d' %mode)

    elif mode==mode_leftSign:
        modeInfo.linear.y == mode_leftSign
        print('Detect Left Sign!!: %d' %mode)

    elif mode==mode_Sign_end:
        mode = mode_lineDetect
        modeInfo.linear.y = mode_lineDetect
        print('READY TO LINEDETECT!!: %d' %mode)

    elif mode==mode_obstacle:
        modeInfo.linear.y = mode_obstacle
        print('Obstacle Mode: %d' %mode)

    elif mode==mode_obstacle_finish:
        mode = mode_lineDetect
        modeInfo.linear.y = mode_lineDetect
        print('Obstacle finished: %d' %mode)

    elif mode==mode_finish:
        mode = mode_finish
        modeInfo.linear.y = mode_lineDetect
        print('!!!!! FINISH !!!!!!')

    elif mode==mode_Stop:
        print('!!!!!! SUCCESS !!!!!')
        # sys.exit()
        # rospy.is_shutdown()


    else:
        modeInfo.linear.y = mode_lineDetect
        # print('LineDetect Mode: %d' %mode)

    pubTolineDetect.publish(modeInfo)
    pubToParking.publish(modeInfo)
    putToObstacle.publish(modeInfo)    

start_flag = False

# main
if __name__ == '__main__':
    start = rospy.wait_for_message('/startFlag', Int32)
    if start.data == mode_start:
        start_flag = True
        modeInfo.linear.y = mode_lineDetect
        pubTolineDetect.publish(modeInfo)
        rospy.loginfo('!!! START !!!')
    if start_flag == True:
        rospy.Subscriber('/signFlag', Twist, checkMode)
        rospy.Subscriber('/parkingReadyFlag', Twist, checkMode)
        rospy.Subscriber('/obstacleFlag', Twist, checkMode)
        rospy.Subscriber('/parkingFlag', Twist, checkMode)
        rospy.Subscriber('/finishFlag', Twist, checkMode)
    rospy.spin()