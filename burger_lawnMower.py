#!/usr/bin/env python
'''
lawn mower motion for turtlebot
'''

import rospy
from gotopoint import GotoPoint
from geometry_msgs.msg import Twist, Point, Quaternion
from tf2_msgs.msg import TFMessage
import tf
import time
import numpy as np
import re
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion



class TurtlebotMover():

    def __init__(self, goal_x, goal_y, gird_points):
        self.goal_X = goal_x
        self.goal_Y = goal_y
        self.grid_points = gird_points
        self.odom_frame = 'odom'
        self.tf_listener = tf.TransformListener()
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

    def MoveBot(self):

        (position, rotation) = self.get_odom()
        init_y = position.y

        while(init_y < self.goal_Y):

            #Logic for generating sub goals and calling GotoPoint
            (position, rotation) = self.get_odom()
            GotoPoint(self.goal_X, position.y, 0)

            (position, rotation) = self.get_odom()
            GotoPoint(position.x, self.goal_Y/self.grid_points, 0)
            init_y = init_y + self.goal_Y/self.grid_points

            (position, rotation) = self.get_odom()
            GotoPoint(-self.goal_X, position.y, 0)

            (position, rotation) = self.get_odom()
            GotoPoint(position.x, self.goal_Y / self.grid_points, 0)
            init_y = init_y + self.goal_Y / self.grid_points


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
    tb_obj = TurtlebotMover(1.2,1.2,4)
    try:
        #tb_obj.getPos()
        #The program keeps running till node does not shutdown.
        tb_obj.MoveBot()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass



















"""
class Turtlebot_Mover:

    def __init__(self, gird_points):
        self.x_pos=0
        self.y_pos=0
        self.distance_x = 0.2 * gird_points
        self.distance_y = 0.2 * gird_points
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.position = Point()
        self.move_cmd = Twist()
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        self.r=rospy.Rate(10)
        self.last_rotation = 0
        self.linear_speed = 1
        self.angular_speed = 1

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

        #Reset Velocities
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0
        self.cmd_vel.publish(self.move_cmd)

    def move_x(self):
        #Moving along the X-Axis direction
        distance_Y = 0.2
        turn=1
        angle=180

        (position, rotation) = self.get_odom()
        current_y_pos = position.y
        iteration=0

        while (current_y_pos < self.goal_y):

            #Moving along the X-axis
            distance = self.distance_x
            (position, rotation) = self.get_odom()
            goal_x = position.x + distance*turn
            goal_y = position.y

            while distance > 0.1:
                (position, rotation) = self.get_odom()
                print(position.x, ",", position.y)

                distance = sqrt(pow((goal_x - position.x), 2) + pow((goal_y - position.y), 2))

                print(distance)
                self.move_cmd.linear.x = min(1 * distance, 0.1)
                self.move_cmd.angular.z = 0.0
                self.cmd_vel.publish(self.move_cmd)
                self.r.sleep()
                #Path angle has been ignored for now. Motion only in straight line.


            print("X-axis motion done")
            self.cmd_vel.publish(Twist())
            age = input("Starting next Iteration")

            #Rotating 90 degrees
            (position, rotation) = self.get_odom()
            goal_z = 90
            goal_z = np.deg2rad(goal_z)
            while abs(rotation - goal_z) >0.05:
                (position, rotation) = self.get_odom()
                #print("rotation = ", rotation)
                #print("goal_z = ", goal_z)
                if goal_z >= 0:
                    if rotation <= goal_z and rotation >= goal_z - pi:
                        self.move_cmd.linear.x = 0.00
                        self.move_cmd.angular.z = 0.5
                    else:
                        self.move_cmd.linear.x = 0.00
                        self.move_cmd.angular.z = -0.5
                else:
                    if rotation <= goal_z + pi and rotation > goal_z:
                        self.move_cmd.linear.x = 0.00
                        self.move_cmd.angular.z = -0.5
                    else:
                        self.move_cmd.linear.x = 0.00
                        self.move_cmd.angular.z = 0.5
                self.cmd_vel.publish(self.move_cmd)
                self.r.sleep()

            print(" 1st rotation done")
            self.cmd_vel.publish(Twist())

            age = input("Starting Y movement")

            #Moving along the Y-axis
            (position, rotation) = self.get_odom()
            distance = distance_Y
            goal_y = position.y + distance
            goal_x = position.x

            while distance > 0.1:
                (position, rotation) = self.get_odom()
                print(position.x, ",", position.y)
                #age = input("Press Enter")
                distance = sqrt(pow((goal_x - position.x), 2) + pow((goal_y - position.y), 2))

                print(distance)
                self.move_cmd.linear.x = min(1 * distance, 0.1)
                self.move_cmd.angular.z = 0.0
                self.cmd_vel.publish(self.move_cmd)
                self.r.sleep()
                #Path angle has been ignored for now. Motion only in straight line.
            print("Y-axis motion done")
            self.cmd_vel.publish(Twist())

            age = input("Starting 2nd rotation")

            (position, rotation) = self.get_odom()
            goal_z = angle
            goal_z = np.deg2rad(goal_z)


            while abs(rotation - goal_z) >0.05:
                (position, rotation) = self.get_odom()
                #print("rotation = ", rotation)
                #print("goal_z = ", goal_z)
                if goal_z >= 0:
                    if rotation <= goal_z and rotation >= goal_z - pi:
                        self.move_cmd.linear.x = 0.00
                        self.move_cmd.angular.z = 0.5
                    else:
                        self.move_cmd.linear.x = 0.00
                        self.move_cmd.angular.z = -0.5
                else:
                    if rotation <= goal_z + pi and rotation > goal_z:
                        self.move_cmd.linear.x = 0.00
                        self.move_cmd.angular.z = -0.5
                    else:
                        self.move_cmd.linear.x = 0.00
                        self.move_cmd.angular.z = 0.5
                self.cmd_vel.publish(self.move_cmd)
                self.r.sleep()

            print(" 2nd rotation done")
            self.cmd_vel.publish(Twist())

            age = input("Starting next Iteration")

            #Do this for flipping the X-axis motion and Angle of rotation on alternate movement
            turn *= -1
            if turn == 1:
                angle = 180
            else:
                angle = 0

            (position, rotation) = self.get_odom()
            final_y = position.y

            age = input("Starting next Iteration")
            iteration+=1
            print("============================ ", iteration, "===============")



        #for Stopping the current motion.
        self.cmd_vel.publish(Twist())

        #rospy.signal_shutdown("Initial Pos extracted")


    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])

"""