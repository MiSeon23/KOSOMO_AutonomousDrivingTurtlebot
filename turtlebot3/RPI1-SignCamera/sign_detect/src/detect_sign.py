#!/usr/bin/env python
'''
This is about detecting the traffic sign in very simple way.
writer : Jaehoon Jeong
last update : 2020.11.15
'''
import rospy
import cv2
import numpy as np
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
# from matplotlib import pyplot as plt
# from cv_bridge import CvBridge, CvBridgeError

############ SYSTEM MODE ############
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
######################################

cap = cv2.VideoCapture(0)

if cap.isOpened()==False:
    print("Can\'t open the Video")
    exit()

width = 640
height = 480

count_right = 0
count_left = 0
w = 0
h = 0
Srec = 6000
Shex = 16000
Scir = 3000

def callback():

    success, cv_image = cap.read()

    if success==False:
        print("theres no video")

    cv_image=cv2.flip(cv_image, -1)
    
    cv_image_blue = np.copy(cv_image)
    cv_image_red = np.copy(cv_image)
    ############### preprocessing ###############
    cv_hsv_blue = cv2.cvtColor(cv_image_blue, cv2.COLOR_BGR2HSV)
    blue_low = (120-20, 120, 70)
    blue_high = (120+10, 255, 210)
    blue_mask = cv2.inRange(cv_hsv_blue, blue_low, blue_high)   

    cv_hsv_red = cv2.cvtColor(cv_image_red, cv2.COLOR_BGR2HSV)
    red_low = (160-10, 70, 30)
    red_high = (160+25, 200, 255)
    
    red_mask = cv2.inRange(cv_hsv_red, red_low, red_high)          
    
    cv_image_rec = np.copy(blue_mask)	    
    cv_image_cir = np.copy(blue_mask)        
    
    _,contours_red,hierarchy_red=cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(cv_image,contours_red,-1,(255,0,255),1)

    _,contours,hierarchy=cv2.findContours(cv_image_rec, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(cv_image,contours,-1,(0,0,255),1)
    ###############################################

    global count_right
    global count_left

    def setLabel(img, pts, label):
        global w
        global h
        (x,y,w,h)=cv2.boundingRect(pts)
        pt1=(x,y)
        pt2=(x+w,y+h)
        if w*h >= 2000:
            cv2.rectangle(img,pt1,pt2,(0,255,0),2)
            cv2.putText(img,label,(pt1[0],pt1[1]-3),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255))
    
    twist = Twist()
    
    for cont in contours_red:
        approx=cv2.approxPolyDP(cont,cv2.arcLength(cont,True)*0.02,True)
        vtc=len(approx)
        if vtc == 8:
            setLabel(cv_image, cont, "STOP")
            if w*h > Shex:
                print("stop")
                twist.linear.y = mode_Stop
                pubToMain = rospy.Publisher('/signFlag', Twist, queue_size=20)
                pubToMain.publish(twist)
                pubToMove = rospy.Publisher('/stopFlag', Twist, queue_size=1)
                pubToMove.publish(twist)

    for cont in contours:
        approx=cv2.approxPolyDP(cont,cv2.arcLength(cont,True)*0.02,True)
        vtc=len(approx)
        
        if vtc==3:
            setLabel(cv_image,cont,'Tri')
        
        ############### detect PARK ###############
        if vtc==4:
            setLabel(cv_image,cont,'Rec')
            if w*h > Srec:
                print("!!! PARKING SIGN !!!")

                twist.linear.y = mode_parkingSign
                pubToMain = rospy.Publisher('/signFlag', Twist, queue_size=20)
                pubToMain.publish(twist)

        elif vtc==5:
            setLabel(cv_image,cont,'Pen')
        
        ############### detect Circle ###############
        else:
            area=cv2.contourArea(cont)
            (x,y),r=cv2.minEnclosingCircle(cont)
            x=int(x)
            y=int(y)
            r=int(r)

            if area != 0 :    
                ratio=r*r*math.pi/area
            
                if int(ratio)==1:
                    setLabel(cv_image,cont,'Cir')
                    
                    cv2.circle(cv_image,(x,y),r,(255,255,0),3)
                    cv2.line(cv_image,(x-r,y),(x+r,y),(0,0,255),2)
                    cv2.line(cv_image,(x,y-r),(x,y+r),(0,0,255),2)
                
                    count3 = 0
                    count4 = 0

                    if x+r>=640 or x-r<=0 or y+r>=480 or y-r<=0:
                        print("over")
                    
                    else:
                        if area > Scir:
                            for j in range(x-r,x,1):
                                for i in range(y,y+r,1):
                                    pixel = cv_image_cir[i][j]
                                    if pixel != 0:
                                        count3 = count3 + 1
                            
                            for k in range (x+1,x+r+1,1):    
                                for m in range(y,y+r,1):    
                                    pixel2 = cv_image_cir[m][k]
                                    if pixel2 != 0:
                                        count4 = count4 +1
                            
                            if count3 > count4:
                                count_left=count_left+1
                            else:
                                count_right=count_right+1
                            if count_left > 10 or count_right > 10:
                                if count_left > count_right:
                                    count_left=0
                                    count_right=0
                                    print("left")
                                    twist.linear.y = mode_leftSign
                                    pubToMain = rospy.Publisher('/signFlag', Twist, queue_size=20)
                                    pubToMain.publish(twist)
                                    
                                else:
                                    count_left=0
                                    count_right=0
                                    print("right")
                                    twist.linear.y = mode_rightSign
                                    pubToMain = rospy.Publisher('/signFlag', Twist, queue_size=20)
                                    pubToMain.publish(twist)

    # cv2.imshow('cv_cir',cv_image_cir)
    # cv2.imshow('result',cv_image)
    # cv2.imshow('cv_rec',cv_image_rec)
    # cv2.waitKey(1)
 
if __name__ == '__main__':
    rospy.init_node('hsv')
    while not rospy.is_shutdown():
        callback()