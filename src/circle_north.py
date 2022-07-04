#! /usr/bin/env python

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, Pose

class circle_walk:
    
    def __init__(self):
        rospy.init_node('circle_north')
        self.movement = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom = rospy.Subscriber('/spot/odom', Pose, odom_callback, queue_size=1)

        radius = rospy.get_param("~radius")
        self.speed = rospy.get_param("~speed")
        self.centre = [0, radius]

        self.move_cmd = Twist()
        self.move_cmd.linear.x = self.speed

        #while not rospy.is_shutdown():
        #    self.movement.publish(self.move_cmd)
        #    rospy.sleep(0.2)

    def odom_callback(self, msg):
        rad_grad = (msg.position.y - self.centre[1]) / (msg.position.x / self.centre[0])
        tangent = -1 / rad_grad

        y_vel = np.sqrt((self.speed**2) / (1 + tangent**2))
        x_vel = np.sqrt((self.speed**2) - (x_vel**2))

        self.move_cmd.linear.x = x_vel
        self.move_cmd.linear.y = y_vel

        if not rospy.is_shutdown():
            self.movement.publish(self.move_cmd)


if __name__ == '__main__':
    walk = circle_walk()
    rospy.spin()