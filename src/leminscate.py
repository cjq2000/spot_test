#! /usr/bin/env python
import rospy

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

import numpy as np


class lemniscate:
    def __init__(self):
        #Start ROS node
        rospy.init()

        #Start Publisher and Subscriber to /cmd_vel and /odom
        self.odom_sub = rospy.Subscriber('/spot/odom', Pose, odom_callback, queue_size=1)
        self.move_pub = rospy.Publisher('/spot/cmd_vel', Twist, queue_size=1)

        #initialise variables
        self.radius = rospy.get_param(radius)
        self.speed = rospy.get_param(speed)
        self.turn = False
        self.completed_segs = 0
        self.tol = 0.5

        #initialise msgs
        self.side_start = Pose()
        self.move_cmd = Twist()
        self.move_cmd.linear.x = self.speed

    
    def odom_callback(self, msg):

        #This is written in 20 mins and not elegant. Reliant on pretty good odom. Get fused odom from gps?
        #Use spot_goto for target?
        if self.completed_segs == 0:
            target = [self.radius, 0]
            dist = np.sqrt((msg.position.x - target[0])**2 + (msg.position.y - target[1])**2)
            
            if dist <= self.tol:
                self.completed_segs += 1
                self.move_cmd.angular.z = self.speed/self.radius
        
        elif self.completed_segs == 1:
            target = [0, self.radius]
            dist = np.sqrt((msg.position.x - target[0])**2 + (msg.position.y - target[1])**2)

            if dist >= self.tol:
                self.completed_segs += 1
                self.move_cmd.angular.z = 0

        elif self.completed_segs == 2:
            target = [0,-self.radius]
            dist = np.sqrt((msg.position.x - target[0])**2 + (msg.position.y - target[1])**2)
            
            if dist <= self.tol:
                self.completed_segs += 1
                self.move_cmd.angular.z = -self.speed/self.radius

        elif self.completed_segs == 3:
            target = [-self.radius, 0]
            dist = np.sqrt((msg.position.x - target[0])**2 + (msg.position.y - target[1])**2)

            if dist >= self.tol:
                self.completed_segs += 1
                self.move_cmd.angular.z = 0

        elif self.completed_segs == 4:
            target = [0, 0]
            dist = np.sqrt((msg.position.x - target[0])**2 + (msg.position.y - target[1])**2)

            if dist >= self.tol:
                self.completed_segs += 1
                self.move_cmd.linear.x = 0
                self.move_cmd.angular.z = 0

        #Publish move_cmd
        if not rospy.is_shutdown():
            self.move_pub.pub(self.move_cmd)

if __name__ == "__main__":
    move_lemon = lemniscate()
    rospy.spin()