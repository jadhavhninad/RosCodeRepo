#!/usr/bin/env python
'''
lawn mower motion for turtlebot
'''

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from tf2_msgs.msg import TFMessage
import tf
import time
import numpy as np
import re
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion




class Turtlebot_Mover:
    def __init__(self):
        self.x_pos=0
        self.y_pos=0
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.position = Point()
        self.move_cmd = Twist()
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

    def pos_callback(self,msg):
        #rospy.loginfo(rospy.get_caller_id())
        #rospy.loginfo(type(msg))
        data = str(msg.transforms[0])
        print(data)

        #vals = re.findall('\d+\.\d', data)
        vals = [float(s) for s in re.findall(r'-?\d+\.?\d*', data)]
        print(vals)

        self.x_pos = vals[3]
        self.y_pos = vals[4]

        print("initial pos = [", self.x_pos, "," , self.y_pos, "]")
        #signal shutdown
        rospy.signal_shutdown("Initial Pos extracted")


    def getPos(self):
        #while not rospy.is_shutdown():
        rospy.Subscriber("tf", TFMessage, self.pos_callback)

    def move_x(self):
        distance = 1.8
        (position, rotation) = self.get_odom()
        goal_x = position.x + distance
        goal_y = position.y + distance

        while distance > 0.02:
            (position, rotation) = self.get_odom()
            distance = sqrt(pow((goal_x - position.x), 2) + pow((goal_y - position.y), 2))
            self.move_cmd.linear.x = min(1 * distance, 0.1)
            self.cmd_vel.publish(self.move_cmd)


        print("Moved to the end of grid. Now to rotate")
        #rospy.signal_shutdown("Initial Pos extracted")


    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])


if __name__ == '__main__':
    rospy.init_node('turtlebot3_obstacle')
    tb_obj = Turtlebot_Mover()
    try:
        #tb_obj.getPos()
        #The program keeps running till node does not shutdown.
        tb_obj.move_x()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
