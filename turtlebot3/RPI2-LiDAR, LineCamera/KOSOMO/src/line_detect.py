#!/usr/bin/env python
"""
This is for line detecting and steering.
writer : Miseon Kim, help : Heewon Son
last update : 2020.11.15
""" 
########## ROS ##########
import rospy
from geometry_msgs.msg import Twist
#from cv_bridge import CvBridge, CvBridgeError
#from sensor_msgs.msg import Image
#########################
import cv2
import numpy as np
import math
import time #for FPS
from collections import deque

cap = cv2.VideoCapture(0)

if cap.isOpened()==False:
    print("Can\'t open the Video")
    exit()

width = 640
height = 480

# save as color
fourcc = cv2.VideoWriter_fourcc(*'XVID')
writer = cv2.VideoWriter('output.avi', fourcc, 30.0, (width, height))

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

#################### MACRO #####################
mode_default = 10
mode_noLine = 11
mode_leftLine = 12
mode_rightLine = 13
mode_allLines = 14
mode_detectCurve = 15

velocityMAX = 0.22
anagularMAX = 0.315

#################### PARAMETERS #####################
channel_count = 3

ROI_vertices_left = [
    (0,0),
    (int(width*3/6),0),
    (int(width*3/6), height),
    (0,height)
]
ROI_vertices_right = [
    (width,height),
    (int((width*3)/6), height),
    (int((width*3)/6), 0),
    (width, 0)
]
ROI_vertices_noLine = [
    (0,int(height/4)),
    (width, int(height/4)),
    (width, height),
    (0, height)
]

coefficient_mid_0 = deque([])
coefficient_mid_1 = deque([])
coefficient_queSize = 5

twist_queSize = 0
linear_x_que = deque([])
angular_z_que = deque([])

before_err = 0  # for no line
before_avg_slope = 1.5

def getParameters(mode):
    if mode == mode_default:
        min_y = 0
        max_y = int(height)
        slope_threshold = 0.7
    elif mode == mode_noLine:
        min_y = 0
        max_y = int(height/2)
        slope_threshold = 0.2
    return min_y, max_y, slope_threshold
####################################################

def ROI(img, vertices, channel_count):
    mask = np.zeros_like(img)
    match_mask_color = (255,)*channel_count
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def grayscale(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

def draw_lines(img, lines, color=[0,255,0], thickness=3):
    if lines is None:
        return
    
    img = np.copy(img)
    line_img = np.zeros(
        (
            img.shape[0],
            img.shape[1],
            3
        ),
        dtype=np.uint8
    )

    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
    
    img = cv2.addWeighted(img, 0.9, line_img, 1.0, 1.0)

    return img

def preprocessing(img, mode) :
    img = img - 10
    gray_img = grayscale(img)

    kernel_size = 5
    blur_gray_img = cv2.GaussianBlur(gray_img, (kernel_size, kernel_size), 0)

    low_threshold = 150
    high_threshold = 250
    edges_img = cv2.Canny(blur_gray_img, low_threshold, high_threshold)
    return edges_img

def get_Houghlines(img) :
    lines_raw = cv2.HoughLinesP( img,
        rho=6,
        theta=np.pi/120,
        threshold=160,
        lines=np.array([]),
        minLineLength=60,
        maxLineGap=30
    )    
    return lines_raw

def getOneLineArray(raw_line, mode, slope_threshold):
    left_line_x = []
    left_line_y = []
    right_line_x = []
    right_line_y = []

    for line in raw_line:
        for x1, y1, x2, y2 in line:
            slope = (y2-y1)/(x2-x1+0.000000001) # avoid the error of divided by zero
            if math.fabs(slope) < slope_threshold: # only consider extreme slope
                continue
            if slope <=0:
                left_line_x.extend([x1, x2])
                left_line_y.extend([y1, y2])
            else:
                right_line_x.extend([x1, x2])
                right_line_y.extend([y1, y2])

    if mode==mode_leftLine:
        return left_line_x, left_line_y
    elif mode==mode_rightLine:
        return right_line_x, right_line_y

def getLinePolinomial(line_y, line_x, min_y, max_y, mode):
    coefficient = np.polyfit(
        line_y,
        line_x,
        deg=1
    )

    if mode == mode_allLines:
        coefficient_mid_0.append(coefficient[0])
        coefficient_mid_1.append(coefficient[1])
    
    poly_ = np.poly1d(coefficient)
    x_start = int(poly_(max_y))
    x_end = int(poly_(min_y))
    return x_start, x_end, coefficient[0]

def getMidLine(max_y):
    if(len(coefficient_mid_0)==coefficient_queSize):
        coefficient_0_avg = sum(coefficient_mid_0,0.0)/len(coefficient_mid_0)
        coefficient_1_avg = sum(coefficient_mid_1,0.0)/len(coefficient_mid_1)
        coefficient_mid_0.popleft()
        coefficient_mid_0.popleft()
        coefficient_mid_1.popleft()
        coefficient_mid_1.popleft()
    else:
        coefficient_0_avg = sum(coefficient_mid_0,0.0)/(len(coefficient_mid_0)+1)
        coefficient_1_avg = sum(coefficient_mid_1,0.0)/(len(coefficient_mid_1)+1)
    coefficient_mid = [coefficient_0_avg, coefficient_1_avg]
    poly_mid = np.poly1d(coefficient_mid)
    mid_err = (poly_mid(0)+poly_mid(max_y))/2

    # Get Error
    err = width/2 - mid_err

    return err

def pipeline(img):
    global linear_x_que
    global angular_z_que
    global before_err
    global before_avg_slope
    preprocessed_img = preprocessing(img, mode_default)
    
    cropped_img_left = ROI(preprocessed_img, np.array([ROI_vertices_left], np.int32), channel_count)
    cropped_img_right = ROI(preprocessed_img, np.array([ROI_vertices_right], np.int32), channel_count)

    raw_line_left = get_Houghlines(cropped_img_left)
    raw_line_right = get_Houghlines(cropped_img_right)

    left_line_x = []
    left_line_y = []
    right_line_x = []
    right_line_y = []
    min_y, max_y, slope_threshold = getParameters(mode_default)

    nolines = False
    leftline = False
    rightline = False

    #################### Detect Lane ####################
    ### there's no any line
    if raw_line_left is None and raw_line_right is None:
        nolines = True
    
    ### if there're some lines in the left part
    if raw_line_left is not None:
        left_line_x, left_line_y = getOneLineArray(raw_line_left, mode_leftLine, slope_threshold)
        ### it's a lane
        if len(left_line_x):
            leftline = True
        ### it's not a lane
        else:
            leftline = False

    ### if there're some lines in the right part
    if raw_line_right is not None:
        right_line_x, right_line_y = getOneLineArray(raw_line_right, mode_rightLine, slope_threshold)
        ### it's a lane
        if len(right_line_x):
            rightline = True
        ### it's not a lane
        else:
            rightline = False 
    ######################################################


    ################## Drive Algorithm ##################
    ### NO LINES
    if nolines==True or (leftline==False and rightline==False):
        coefficient_mid_0.clear()
        coefficient_mid_1.clear()
        cv2.putText(img, "no lines", (200,240), cv2.FONT_HERSHEY_PLAIN, 3, (0,255,0))
        cropped_img = ROI(preprocessed_img, np.array([ROI_vertices_noLine], np.int32), 1)
        raw_line_curve = cv2.HoughLinesP( cropped_img,
            rho=6,
            theta=np.pi/120,
            threshold=130,
            lines=np.array([]),
            minLineLength=70,
            maxLineGap=200
        )
        if raw_line_curve is None:
            velocity = -0.06
            err = 0
            linear_x_que.append(velocity)
            angular_z_que.append(err)
            print('no line = %f' %(err/5))
        else :
            sum_slope = 0
            for line in raw_line_curve:
                for x1, y1, x2, y2 in line:
                    x1,y1,x2,y2 = line[0]
                    sum_slope = sum_slope + (y2-y1)/(x2-x1+0.000000001)
                    cv2.line(img,(x1,y1),(x2,y2),(0,255,255),5)
            avg_slope = sum_slope/len(line)
            velocity = -0.09

            if abs(avg_slope - before_avg_slope) > 2.8:
                avg_slope = before_avg_slope
            
            if 1 < avg_slope:
                err = 0.0523 * pow(avg_slope, 4) - 0.7409 * pow(avg_slope, 3) + 3.7811 * pow(avg_slope, 2) + 8.0639 + 6.9417
            elif 0 < avg_slope <= 1:
                err = 1.3903 *pow(avg_slope, 2) - 2.4964 * avg_slope + 1.1662
            elif avg_slope < -1:
                err = -0.0523 * pow(avg_slope, 4) + 0.7409 * pow(avg_slope, 3) - 3.7811 * pow(avg_slope, 2) + 8.0639 * avg_slope - 6.9417
            else:
                err = -1.3903 * pow(avg_slope, 2) + 2.4964 * avg_slope - 1.1662

            before_avg_slope = avg_slope

            linear_x_que.append(velocity)
            angular_z_que.append(err)
            print('no line = %f, slope= %f' %(err, avg_slope))

    ### ONLY LEFT   
    elif leftline==True and rightline==False:
        coefficient_mid_0.clear()
        coefficient_mid_1.clear()
        left_x_start, left_x_end, slope = getLinePolinomial(left_line_y, left_line_x, min_y, max_y, mode_leftLine)
        img = draw_lines(img, 
           [[
               [left_x_start, max_y, left_x_end, min_y]
           ]],
           thickness=5
        )
        if abs(slope)>0.9:
            err = slope*0.9
            velocity = -0.09
            print("!!! no right line = %f, extreme slope = %f !!!" %(err/5, slope))
        else :
            err = slope*0.5
            velocity = -0.09
            print("no right line = %f, slope = %f" %(err, slope))
        linear_x_que.append(velocity)
        angular_z_que.append(err)
        
        # cv2.putText(img, "no right line", (200,240), cv2.FONT_HERSHEY_PLAIN, 3, (0,255,0))

    ### ONLY RIGHT
    elif leftline==False and rightline==True:
        coefficient_mid_0.clear()
        coefficient_mid_1.clear()
        right_x_start, right_x_end, slope = getLinePolinomial(right_line_y, right_line_x, min_y, max_y, mode_rightLine)
        img = draw_lines(img, 
            [[
                [right_x_start, max_y, right_x_end, min_y]
            ]],
            thickness=5
        )
        if abs(slope)>0.9:
            err = slope*0.9
            velocity = -0.09
            print("!!! no left line = %f, extreme slope = %f !!!" %(err/5, slope))
        else:
            err = slope*0.5
            velocity = -0.09
            print("no left line = %f, slope = %f" %(err, slope))
        linear_x_que.append(velocity)
        angular_z_que.append(err)
        # cv2.putText(img, "no left line", (200,240), cv2.FONT_HERSHEY_PLAIN, 3, (0,255,0))

    ### ALL LINES
    elif leftline==True and rightline==True:
        if len(left_line_x):
            left_x_start, left_x_end, left_slope = getLinePolinomial(left_line_y, left_line_x, min_y, max_y, mode_allLines)
        else:
            left_x_start = 0
            left_x_end = 0
        if len(right_line_x):
            right_x_start, right_x_end, right_slope = getLinePolinomial(right_line_y, right_line_x, min_y, max_y, mode_allLines)
        else:
            right_x_start = 0
            right_x_end = 0
        #img = draw_lines(img, 
        #    [[
        #        [left_x_start, max_y, left_x_end, min_y],
        #        [right_x_start, max_y, right_x_end, min_y]
        #    ]],
        #    thickness=5
        #)        
        velocity = -0.13
        err = getMidLine(max_y)/800
        linear_x_que.append(velocity)
        angular_z_que.append(err)
        print("all err = %f, left slope = %f, right slope = %f" %(err, left_slope, right_slope))
        # cv2.putText(img, "All lines are deteced", (200,240), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
    ######################################################
    return img, preprocessed_img 


########## ROS INIT ##########
twist = Twist()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
rospy.init_node('line_detect')
##############################

before_err = 0
err = 0
def callback(frame):    # image_msg
    rospy.loginfo('line detect mode')
    global before_err
    global err
    # bridge = CvBridge()
    # frame = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    # frame = cv2.flip(frame, 0)
    new_frame, preprocessed_img = pipeline(frame)

    velocity = linear_x_que.popleft()
    rospy.loginfo("vel pop")
    err = angular_z_que.popleft()
    rospy.loginfo("err pop")

    err = err/5

    # rospy.loginfo("err - before_error : %f" %abs(err-before_err))

    # if abs(err - before_err) > 0.19 :
    #     # rospy.loginfo("before_error : %f" %before_err)
    #     err = before_err
        
    if err > anagularMAX:
        err = anagularMAX
    elif err < -anagularMAX:
        err = -anagularMAX
    # rospy.loginfo("before_error : %f" %before_err)
    before_err = err

    twist.linear.x = velocity
    twist.angular.z = err
    pub.publish(twist)

    # draw fixed line
    new_frame = cv2.line(new_frame, (int(width/2), int(height/3)), (int(width/2), int(height/6)), color=[0,0,255], thickness=3)
    # draw err line
    new_frame = cv2.line(new_frame, (int((width/2)+err*10), int(height/3)), (int((width/2)+err*10), int(height/6)), color=[0,255,0], thickness=3)
    # draw fixed line to err line
    new_frame = cv2.line(new_frame, (int(width/2), int(height/4)), (int((width/2)+err*10), int(height/4)), color=[0,255,0], thickness=3)

    cv2.imshow('new_img', new_frame)
    cv2.imshow('preprocessed', preprocessed_img)
    cv2.waitKey(1)
    writer.write(new_frame)

_mode = -1
def mode_func(mode_msg):
    global _mode
    _mode = mode_msg.linear.y 

# main
if __name__ == '__main__':
    # _mode = mode_lineDetect
    before_mode = 100
    rospy.Subscriber('/lineInfo', Twist, mode_func)
    while (not rospy.is_shutdown()):
        # image_msg = rospy.wait_for_message('/image_raw', Image)
        # mode_msg = rospy.wait_for_message('/lineInfo', Twist)      
        # bridge = CvBridge()
        # frame = bridge.imgmsg_to_cv2(image_msg, "bgr8")

        success, frame = cap.read()

        if success==False:
            print("theres no video")
            break
        frame = cv2.flip(frame, -1)

        if _mode == mode_lineDetect:
            ###################### Set Queue ######################
            # if mode changed, set queue
            # when start
            if before_mode == 100:
                rospy.loginfo("!!!!! START !!!!!")
                linear_x_que = deque([-0.11 for i in range(twist_queSize-1)])
                angular_z_que = deque([0 for i in range(twist_queSize-1)])
            # when after parking, avoiding obstacle (curve)
            elif before_mode != _mode:
                rospy.loginfo("!!!! Mode Changed !!!!")
                linear_x_que = deque([])
                angular_z_que = deque([])
            #######################################################
            callback(frame)

        else:
            rospy.loginfo('mode is %d: '%_mode)
        before_mode = _mode

    if rospy.is_shutdown():
        writer.release()
        cv2.destroyAllWindows()