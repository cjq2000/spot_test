#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist

class circle_walk:
    def __init__(self):
        rospy.init_node('circle')

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

        check = raw_input("Start square test? y or n")
        

        radius = rospy.get_param("~radius")
        speed = rospy.get_param("~speed")
        
        self.move_cmd = Twist()
        self.move_cmd.linear.x = speed
        self.move_cmd.angular.z = speed/radius

        if str(check) == "y":
            self.movement = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        while not rospy.is_shutdown():
            self.movement.publish(self.move_cmd)
            rospy.sleep(0.2)

if __name__ == '__main__':
    walk = circle_walk()