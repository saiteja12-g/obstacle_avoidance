#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist
import numpy as np

def callback(img_data):
    # Process depth image dataan
    cv_image = bridge.imgmsg_to_cv2(img_data, "passthrough")
    img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 59, 0])
    upper_red = np.array([179, 255, 255])
    imagemask = cv2.inRange(img, lower_red, upper_red)
    result = cv2.resize(imagemask, (640, 480))
    result = cv2.line(result, (320, 0), (320, 480), (0, 255, 0), 2)
    

    done = False
    while not done:
        contours, _ = cv2.findContours(result, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print("Number of contours found =", len(contours))
        if len(contours) >= 2:
            area1 = cv2.contourArea(contours[0])
            area2 = cv2.contourArea(contours[1])
            area = area1 + area2
            print("Contour 1 area:", area1)
            print("Contour 2 area:", area2)
            cv2.imshow('After HSV conversion', result)
            cv2.waitKey(1)

            if area1 > 0 or area2 > 0:
                if area1 > area/2:
                    print("Right")
                    move.angular.z = 0.1
                else:
                    print("Left")
                    move.angular.z = -0.1
                pub.publish(move)
            else:
                print("Both contours disappeared, stopping the robot")
                move.angular.z = 0.0
                pub.publish(move)
                done = True
        else:
            print("Contours not found, stopping the robot")
            move.angular.z = 0.0
            pub.publish(move)
            done = True
    move.angular.z = 0.0
    pub.publish(move)

move = Twist()


rospy.init_node('obstacle_avoidance_node')

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

sub_depth = rospy.Subscriber("/camera/rgb/image_raw", Image, callback, queue_size=1)

rospy.spin()
