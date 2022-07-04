#! /usr/bin/env python
import rospy

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import numpy as np
from std_srvs.srv import Trigger

class Move:
    def __init__(self):
        #Start ROS node
        rospy.init()

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
       
        
"""
    def odom_callback(self, msg):

        #Check square is not complete
        if self.completed_sides < 2:
    
            #Check distance walked
            dist = np.sqrt((msg.position.x - self.sidestart.position.x)**2 + (msg.position.y - self.sidestart.position.y)**2)
            
            #Check if total distance has been walked and now need to turn
            if dist >= self.side:
                self.completed_sides += 1
                #Change move_cmd to turn

                self.move_cmd.linear.x = 0
                self.move_cmd.linear.y = self.speed
                #Update start of side position
                self.side_start = msg

            #Publish move_cmd
            if not rospy.is_shutdown():
                self.move_pub.pub(self.move_cmd)
"""
if __name__ == "__main__":
    move_square = Move()
    try:
        rospy.spin()
    except:
        KeyboardInterrupt("Keyboard Interupt: Shutdown")