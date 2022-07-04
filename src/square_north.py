#! /usr/bin/env python
import rospy

from geometry_msgs.msg import Pose, Twist
import numpy as np

class Move: 
    def __init__(self):
        #Start ROS node
        rospy.init()

        #Start Publisher and Subscriber to /cmd_vel and /odom
        self.odom_sub = rospy.Subscriber('/spot/odom', Pose, odom_callback, queue_size=1)
        self.move_pub = rospy.Publisher('/spot/cmd_vel', Twist, queue_size=1)

        #initialise variables
        self.side = rospy.get_param(side)
        self.speed = rospy.get_param(speed)
        self.turn = False
        self.completed_sides = 0

        #initialise msgs
        self.side_start = Pose()
        self.move_cmd = Twist()
        self.move_cmd.linear.x = self.speed
        

    def odom_callback(self, msg):

        #Check square is not complete
        if self.completed_sides < 4:
            
            #Check distance walked
            dist = sqrt((msg.position.x - self.sidestart.position.x)**2 + (msg.position.y - self.sidestart.position.y)**2)
            
            #Check if total distance has been walked and now need to turn
            if dist >= self.side:
                #Change move_cmd 
                if self.completed_sides == 1:
                    self.move_cmd.linear.x = 0
                    self.move_cmd.linear.y = self.speed
                elif self.completed_sides == 2:
                    self.move_cmd.linear.x = -self.speed
                    self.move_cmd.linear.y = 0
                elif self.completed_sides == 3:
                    self.move_cmd.linear.x = 0
                    self.move_cmd.linear.y = -self.speed
                    
                self.side_start = msg


            #Publish move_cmd
            if not rospy.is_shutdown():
                self.move_pub.pub(self.move_cmd)

if __name__ == "__main__":
    move_square = Move()
    rospy.spin()