#! /usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
from std_srvs.srv import Trigger

class Move:
    def __init__(self):
        #Start ROS node
        rospy.init_node('spot_test')

        self.claim = rospy.ServiceProxy('/spot/claim', Trigger)
        self.power_on = rospy.ServiceProxy('spot/power_on', Trigger)
        self.stand = rospy.ServiceProxy('spot/stand', Trigger)

        rospy.wait_for_service('/spot/claim')
        rospy.wait_for_service('/spot/power_on')
        rospy.wait_for_service('/spot/stand')

        try:
            rospy.loginfo(self.claim())
            rospy.loginfo(self.power_on())
            rospy.loginfo(self.stand())

        except:
            print("Service failed: %s"%e)

        #Start Publisher and Subscriber to /cmd_vel and /odom

        #initialise variables
        self.side = rospy.get_param("test_spot/side")
        self.speed = rospy.get_param("test_spot/speed")
        self.turn = False
        self.completed_sides = 0

        #initialise msgs
        self.odom_set = False
        self.sidestart = Odometry()
        self.move_cmd = Twist()
        self.move_cmd.linear.x = self.speed
        self.move_cmd.linear.y = -self.speed
        # Y: +ve is left, -ve is right walk
        # Z: +ve is right, -ve is left turn
	    
        check = raw_input("Start walk test? y or n")
        if str(check) == "y":
            self.odom_sub = rospy.Subscriber('/spot/odometry', Odometry, self.odom_callback, queue_size=1)
            self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def odom_callback(self, msg):
        if self.odom_set == False:
            self.sidestart = msg
            self.odom_set = True

        #Check square is not complete
        if self.completed_sides < 1:

            #Check distance walked
            dist = np.sqrt((msg.pose.pose.position.x - self.sidestart.pose.pose.position.x)**2 + (msg.pose.pose.position.y - self.sidestart.pose.pose.position.y)**2)

            #Check if total distance has been walked and now need to turn
            if dist >= self.side:
                self.completed_sides += 1
                #Change move_cmd to turn

                self.move_cmd.linear.x = 0
                self.move_cmd.linear.y = self.speed
                #Update start of side position
                self.side_start = msg


        else:
            self.move_cmd.linear.x = 0
            self.move_cmd.linear.y = 0
        
        #Publish move_cmd
        if not rospy.is_shutdown():
            self.move_pub.publish(self.move_cmd)

if __name__ == "__main__":
    move_square = Move()
    rospy.spin()