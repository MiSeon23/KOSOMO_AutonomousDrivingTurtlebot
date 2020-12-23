#!/usr/bin/env python
'''
This is about post-sign recognition behavior.
writer : Miseon Kim, Jaehoon Jeong
last update : 2020.11.15
'''
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

############ SYSTEM MODE #############
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

############## ROS INIT ##############
twist = Twist()
twist2 = Twist()
pubToCore = rospy.Publisher('/cmd_vel', Twist, queue_size=20)
pub = rospy.Publisher('/parkingReadyFlag', Twist, queue_size=5)
rospy.init_node('signP')
######################################

first_flag = False
second_flag = False

def moving(velocity, ang):
    twist2.linear.x = velocity
    twist2.angular.z = ang
    pubToCore.publish(twist2)

def checkMode(twist_msg):
    global first_flag
    global second_flag
    if second_flag == True and twist_msg.linear.y == mode_Stop:
        print('Stop Mode: %d' %twist_msg.linear.y)
        for j in range(1000):
            rospy.loginfo('stop %d' %j)
            moving(0, 0)
        twist.linear.y = mode_Stop
        pub.publish(twist)

# main
if __name__ == '__main__':
    mode_before = mode_lineDetect

    rospy.Subscriber('/stopFlag', Twist, checkMode)
    while (not rospy.is_shutdown()):
        mode_msg = rospy.wait_for_message('/lineInfo', Twist)
        mode_ = mode_msg.linear.y

        # First Mission : Stop Sign
        if mode_before==mode_lineDetect and mode_==mode_parkingSign:
            print('Parking Mode: %d' %mode_)
            for i in range(1300):
                rospy.loginfo('1_3 %d' %i)
                moving(-0.07, 0)
            for j in range(4400):
                rospy.loginfo('2_3 %d' %j)
                moving(-0.07, 0.35)
            for l in range(8000):
                rospy.loginfo('3_3 %d' %l)
                moving(-0.07, 0)
            for k in range(35000):
                rospy.loginfo('wait')            
            twist.linear.y = mode_parking
            pub.publish(twist)
            print('Parking Mode END: ')

        # Second Mission : Right sign on first crossroad
        elif first_flag == False and mode_==mode_rightSign:
            print('RightSign Mode1: %d' %mode_)
            for i in range(1300):
                rospy.loginfo('1_3 %d' %i)
                moving(-0.07, 0)
            for j in range(8800):
                rospy.loginfo('2_3 %d' %j)
                moving(-0.07, -0.31)
            for k in range(5000):
                rospy.loginfo('wait') 
                moving(0,0)
            twist.linear.y = mode_Sign_end
            pub.publish(twist)
            print('RightSign Mode1 END: ') 
            first_flag = True

        # Third Misseion : Right sign on second crossroad
        elif first_flag == True and mode_==mode_rightSign:
            print('RightSign Mode2: %d' %mode_)
            for i in range(550):
                rospy.loginfo('1_3 %d' %i)
                moving(-0.07, 0)
            for j in range(4400):
                rospy.loginfo('2_3 %d' %j)
                moving(-0.065, -0.31)
            for k in range(15000):
                rospy.loginfo('wait') 
                moving(0,0)
            twist.linear.y = mode_Sign_end
            pub.publish(twist)
            print('RightSign Mode2 END: ')     
            second_flag = True

        # Last Mission : Stop sign
        elif mode_before==mode_lineDetect and mode_==mode_leftSign:
            print('LeftSign Mode: %d' %mode_)

        # works like mode trigger
        mode_before = mode_