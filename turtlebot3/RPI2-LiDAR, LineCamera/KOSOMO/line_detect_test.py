"""
line_detect_rev9.cpp
>> good straight driving (even when there are no lines) 
>> omo-r1 test good
""" 
########## ROS ##########
# import rospy
# from geometry_msgs.msg import Twist
#########################
import cv2
import numpy as np
import math
import time #for FPS

# cap = cv2.VideoCapture("/home/miseon/catkin_ws/src/KOSOMO/src/testVideo.mp4")
cap = cv2.VideoCapture(0)

if cap.isOpened()==False :
    print("Can\'t open the Video")
    exit()

fourcc = cv2.VideoWriter_fourcc(*'XVID')

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# save as color
writer = cv2.VideoWriter('output.avi', fourcc, 30.0, (width, height))

#################### MACRO #####################
mode_default = 0
mode_noLine = 1
mode_leftLine = 2
mode_rightLine = 3
mode_allLines = 4

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
    (0,0),
    (width, 0),
    (width, int(height/2)),
    (0, int(height/2))
]

coefficient_mid_0 = []
coefficient_mid_1 = []
right_curve_err = []
coefficient_queSize = 3

def getParameters(mode):
    if mode == mode_default:
        min_y = 0
        max_y = int(height)
        slope_threshold = 0.4
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
    gray_img = grayscale(img)

    kernel_size = 5
    blur_gray_img = cv2.GaussianBlur(gray_img, (kernel_size, kernel_size), 0)

    low_threshold = 280
    high_threshold = 350
    edges_img = cv2.Canny(blur_gray_img, low_threshold, high_threshold)
    return edges_img

def get_Houghlines(img) :
    lines_raw = cv2.HoughLinesP( img,
        rho=6,
        theta=np.pi/120,
        threshold=160,
        lines=np.array([]),
        minLineLength=40,
        maxLineGap=25
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
        coefficient_mid_0.pop(0)
        coefficient_mid_0.pop(0)
        coefficient_mid_1.pop(0)
        coefficient_mid_1.pop(0)
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
    ### there's any line
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
        cv2.putText(img, "no lines", (200,240), cv2.FONT_HERSHEY_PLAIN, 3, (0,255,0))
        print("no line\n")
        velocity = 0
        err = 0
        print("noline err = %f" %err)
    ### ONLY LEFT
    elif leftline==True and rightline==False:
        left_x_start, left_x_end, slope = getLinePolinomial(left_line_y, left_line_x, min_y, max_y, mode_leftLine)
        img = draw_lines(img, 
            [[
                [left_x_start, max_y, left_x_end, min_y]
            ]],
            thickness=5
        )
        velocity = 0.03
        err = slope/2.
        print("left err = %f" %err)
        cv2.putText(img, "no right line", (200,240), cv2.FONT_HERSHEY_PLAIN, 3, (0,255,0))
    ### ONLY RIGHT
    elif leftline==False and rightline==True:
        right_x_start, right_x_end, slope = getLinePolinomial(right_line_y, right_line_x, min_y, max_y, mode_rightLine)
        img = draw_lines(img, 
            [[
                [right_x_start, max_y, right_x_end, min_y]
            ]],
            thickness=5
        )
        velocity = 0.03
        err = slope/2.
        print("right err = %f" %err)
        cv2.putText(img, "no left line", (200,240), cv2.FONT_HERSHEY_PLAIN, 3, (0,255,0))
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
        img = draw_lines(img, 
            [[
                [left_x_start, max_y, left_x_end, min_y],
                [right_x_start, max_y, right_x_end, min_y]
            ]],
            thickness=5
        )        
        velocity = 0.05
        err = getMidLine(max_y)
        ### when Z lane
        if err > 2.5:
            coefficient_mid_0.clear()
            coefficient_mid_1.clear()
            err = right_slope/2.
        elif err < -2.5:
            coefficient_mid_0.clear()
            coefficient_mid_1.clear()
            err = left_slope/2.
        print("all err = %f" %err)
        cv2.putText(img, "All lines are deteced", (200,240), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0))
    ######################################################

    return velocity, err, img, preprocessed_img 

########## ROS INIT ##########
# twist = Twist()
# pub = rospy.Publisher('/control', Twist, queue_size=20)
# rospy.init_node('control')
##############################

before_err = 0
# main
while(1):
    success, frame = cap.read()

    if success==False :
        print("theres no video")
        break
    
    ########## Real Camera Setting ##########
    frame = cv2.flip(frame, 1)
    
    ######### Video Test Setting ##########
    # frame_left = cv2.flip(frame_left, 1)
    # frame_right = cv2.flip(frame_right, 1)

    velocity, err, new_frame, preprocessed_img = pipeline(frame)
    if abs(err - before_err) > 5. :
        err = before_err

    # fixed line
    new_frame = cv2.line(new_frame, (int(width/2), int(height/3)), (int(width/2), int(height/6)), color=[0,0,255], thickness=3)
    # err line
    new_frame = cv2.line(new_frame, (int((width/2)+err*10), int(height/3)), (int((width/2)+err*10), int(height/6)), color=[0,255,0], thickness=3)
    # fixed line to err line
    new_frame = cv2.line(new_frame, (int(width/2), int(height/4)), (int((width/2)+err*10), int(height/4)), color=[0,255,0], thickness=3)
    
    before_err = err

    ########## ROS ##########
    # if(not rospy.is_shutdown()):
    #     twist.angular.z = err/10.
    #     pub.publish(twist)
    #########################

    fps = cap.get(cv2.CAP_PROP_FPS)

    str = "FPS : {0:0.1f}, ERROR : {1:0.2f}".format(fps, err)
    cv2.putText(new_frame, str, (0,100), cv2.FONT_HERSHEY_PLAIN, 1, (255,255,255))

    #cv2.imshow('OUTPUT', new_frame)
    #cv2.imshow('preprocessed', preprocessed_img)
    writer.write(new_frame)

    # exit when press the ESC
    if cv2.waitKey(1)&0xFF == 27:
        break

cap.release()
writer.release()
cv2.destroyAllWindows()