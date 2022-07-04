#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist

class circle_walk:
    def __init__(self):
        rospy.init_node('circle')
        self.movement = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        radius = rospy.get_param("~radius")
        speed = rospy.get_param("~speed")
        
        self.move_cmd = Twist()
        self.move_cmd.linear.x = speed
        self.move_cmd.angular.z = speed/radius

        while not rospy.is_shutdown():
            self.movement.publish(self.move_cmd)
            rospy.sleep(0.2)

if __name__ == '__main__':
    walk = circle_walk()