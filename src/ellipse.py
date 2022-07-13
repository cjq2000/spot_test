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

        #Claim control of spottherobot and stand
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

        #initialise variables
        self.odom_set = False
        self.side = rospy.get_param(side)
        self.radius = rospy.get_param(radius)
        self.speed = rospy.get_param(speed)
        self.ang_vel = rospy.get_param(angular_vel)
        self.reps = rospy.get_param(reps)
        self.comp_reps = 0
        self.turn = False
        self.target_set = False
        self.completed_sides = 0

        #initialise msgs
        self.sidestart = Pose()
        self.move_cmd = Twist()
        self.move_cmd.linear.x = self.speed

        #Start Publisher and Subscriber to /cmd_vel and /odom
        check = raw_input("Start ellipse test? y or n")
        if str(check) == "y":
            self.odom_sub = rospy.Subscriber('/spot/odometry', Pose, self.odom_callback, queue_size=1)
            self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        
    def odom_callback(self, msg):
        #Check if Start position has been set
        if self.odom_set == False:
            self.sidestart = msg
            self.odom_set = True
        
        if self.comp_reps < self.reps:

            #Check square is not complete
            if self.completed_sides < 2:
            
                #Check if the side needs to be walked
                if self.turn == False:
                    #Check distance walked
                    dist = np.sqrt((msg.position.x - self.sidestart.position.x)**2 + (msg.position.y - self.sidestart.position.y)**2)
                    
                    #Check if total distance has been walked and now need to turn
                    if dist >= self.side:
                        self.turn = True
                        #Change move_cmd to turn
                        self.move_cmd.linear.x = self.speed
                        self.move_cmd.angular.z = self.speed/self.radius
                        #Update start of side position
                        self.sidestart = msg
                        
                #Check if robot is turning
                elif self.turn == True:
                    if self.target_set == False:
                        #Set target angle to 1 more, modulo 2 and translate by -1 (spot angular pose is between -1 and 1)
                        self.target = ((self.sidestart.angular.z + 1.0 )%2) - 1
                        self.target_set = True

                    if (msg.angular.z < self.target + 0.05) and (msg.angular.z > self.target - 0.05):
                        self.turn = False
                        self.target_set = False
                        #Record new completed side
                        self.completed_sides += 1
                        #Update move_cmd
                        self.move_cmd.linear.x = self.speed
                        self.move_cmd.angular.z = 0
                        #Update sidestart
                        self.sidestart = msg

            else:
                #Square completed - reset variables to start again
                self.comp_reps += 1
                self.completed_sides = 0
                self.odom_set = False

                self.move_cmd.linear.x = 0
                self.move_cmd.angular.z = 0

        #Publish move_cmd
        if not rospy.is_shutdown():
            self.move_pub.pub(self.move_cmd)

if __name__ == "__main__":
    move_square = Move()
    rospy.spin()