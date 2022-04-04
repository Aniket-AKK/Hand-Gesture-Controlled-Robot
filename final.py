#!/usr/bin/env python

import cv2
import numpy as np
import math
import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

capture = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_SIMPLEX
n=0


# msg at starting
msg1 = """
Control Your TurtleBot3!
---------------------------
Moving around:
        1/6
   4/9   0    3/8
        2/7

1 / 6  : increase linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
2 / 7  : decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
3 / 8  : increase angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)
4 / 9  : decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)
0      : stop

CTRL-C to quit
"""


# msg aafter each pair of 100
msg2 = """

------------------------------
reached 100 
------------------------------

"""

e = """
Communications Failed
"""

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s\t n = %d" % (target_linear_vel,target_angular_vel,n)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    try:
        print(msg1)
        while(1):

            #frame
            ret, frame  = capture.read()

            #croping required part
            cv2.rectangle(frame , (100,100), (600,600),(0,255,0), 0)
            crop_image = frame[100:600, 100:600]

            #blurring
            blur = cv2.GaussianBlur(crop_image, (3, 3), 0)

            #bgr to hsv
            hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
            #cv2.imshow('hsv',hsv)

            #mask for skin color
            mask = cv2.inRange(hsv, np.array([0,0,0]), np.array([20,255,255]))
            #cv2.imshow('mask',mask)

            #kernel
            kernel = np.ones((5,5))

            #filter background noice
            dilation = cv2.dilate(mask, kernel, iterations=2)
            erosion = cv2.erode(dilation, kernel, iterations=2)

            #thresholding
            blur2 = cv2.GaussianBlur(erosion, (5, 5), 0)
            ret, thresh = cv2.threshold(blur2, 127,255,0)
            #cv2.imshow("Threshold" , thresh)

            #contour
            contours,hierarchy= cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)


            #Find Max area
            contour = max(contours, key=lambda x: cv2.contourArea(x))

            #approx the contour a little
            epsilon = 0.0005*cv2.arcLength(contour,True)
            approx= cv2.approxPolyDP(contour,epsilon,True)

            #rectangle aaround the contour
            x,y,w,h = cv2.boundingRect(contour)
            cv2.rectangle(crop_image, (x,y), (x+w,y+h), (0,0,225),0)

            #hull
            hull = cv2.convexHull(contour)

            #extent
            area_cntour = cv2.contourArea(contour)
            rectangle_area = w*h
            extent = float(area_cntour)/rectangle_area

            #draw contour
            drawing = np.zeros(crop_image.shape, np.uint8)
            cv2.drawContours(drawing, [contour], -1,(0,0,255), 2)
            cv2.drawContours(drawing, [hull], -1,(127,127,127), 2)

            #convexity defects
            hull = cv2.convexHull(contour, returnPoints = False)
            defects = cv2.convexityDefects(contour, hull)
            con_defects = 0
            for i in range(defects.shape[0]):
                s,e,f,d = defects[i,0]
                start = tuple(contour[s][0])
                end = tuple(contour[e][0])
                far = tuple(contour[f][0])
                pt= (100,180)

                # find length of all sides of triangle
                a = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
                b = math.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
                c = math.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)
                s = (a+b+c)/2
                ar = math.sqrt(s*(s-a)*(s-b)*(s-c))

                #distance between point and convex hull
                d=(2*ar)/a

                # angle
                angle = (math.acos((b**2 + c**2 - a**2)/(2*b*c)) *180) /3.14 


                # ignore angles > 80 and ignore points very close to convex hull(they generally come due to noise)
                if angle <= 75 and d>30 :
                    con_defects += 1
                    cv2.circle(crop_image, far, 3, [0,255,0], -1)

                #draw lines around hand
                cv2.line(crop_image,start, end, [255,0,0], 2)


            if con_defects==0 and extent > 0.65:
                cv2.putText(crop_image,'0',(0,50), font, 2, (0,255,255), 3, cv2.LINE_AA)
                n=0
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))   

            elif con_defects==0 and extent<0.65:
                if extent >0.55:
                    cv2.putText(crop_image,'6',(0,50), font, 2, (0,255,255), 3, cv2.LINE_AA)
                    n=6
                else:
                    cv2.putText(crop_image,'1',(0,50), font, 2, (0,255,255), 3, cv2.LINE_AA)
                    n=1 
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                #checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))

            elif con_defects==1:
                if extent >0.44:
                    cv2.putText(crop_image,'2',(0,50), font, 2, (0,255,255), 3, cv2.LINE_AA)
                    n=2
                else:
                    cv2.putText(crop_image,'7',(0,50), font, 2, (0,255,255), 3, cv2.LINE_AA)
                    n=7
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))

            elif con_defects==2:
                if extent >0.54:
                    cv2.putText(crop_image,'3',(0,50), font, 2, (0,255,255), 3, cv2.LINE_AA)
                    n=3
                else:
                    cv2.putText(crop_image,'8',(0,50), font, 2, (0,255,255), 3, cv2.LINE_AA)
                    n=8
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))

            elif con_defects==3:
                if extent >0.5:
                    cv2.putText(crop_image,'9',(0,50), font, 2, (0,255,255), 3, cv2.LINE_AA)
                    n=9
                else:
                    cv2.putText(crop_image,'4',(0,50), font, 2, (0,255,255), 3, cv2.LINE_AA)
                    n=4
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))

            elif con_defects==4:
                cv2.putText(crop_image,'5',(0,50), font, 2, (0,255,255), 3, cv2.LINE_AA)
                n=5
                target_angular_vel = 0.2
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))

            else :
                cv2.putText(crop_image,'error',(10,50), font, 2, (0,0,255), 3, cv2.LINE_AA)
                n=0

            #cv2.imshow("Frame" , frame)   
            #final_image = np.hstack((crop_image,drawing))
            #cv2.imshow("Task 2" , final_image)
            cv2.imshow("Task 2" , crop_image)
            k = cv2.waitKey(1) & 0xFF
            
            if k == 27:
                break

            if status == 100 :
                print(msg2)
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

capture.release()  
cv2.destroyAllWindows()
