#!/usr/bin/env python3
import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist


def callback(scan_msg):
    print ('-------------------------------------------')
    print ('Obstacle distance at 0 deg:   {}'.format(scan_msg.ranges[0]))
    print ('Obstacle distance at 15 deg:  {}'.format(scan_msg.ranges[15]))
    print ('Obstacle distance at 345 deg: {}'.format(scan_msg.ranges[345]))
    print ('-------------------------------------------')
    thr1 = 0.8 # Laser scan range threshold
    
    if scan_msg.ranges[0]>thr1 and scan_msg.ranges[15]>thr1 and scan_msg.ranges[-15]>thr1:
        move.linear.x = 0.01
        move.angular.z = 0.0
    else:
        move.linear.x = 0.0 
        move.angular.z = 0.01
        if scan_msg.ranges[0]>thr1 and scan_msg.ranges[15]>thr1 and scan_msg.ranges[-15]>thr1:
            move.linear.x = 0.01
            move.angular.z = 0.0
    pub.publish(move) 


move = Twist() 
rospy.init_node('obstacle_avoidance_node') # Initializes a node
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
sub = rospy.Subscriber("/scan", LaserScan, callback)
rospy.spin() 