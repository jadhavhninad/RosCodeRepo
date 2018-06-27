#!/usr/bin/env python
'''
lawn mower motion for turtlebot
'''

import rospy
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
import time
import numpy as np
import re



def pos_callback(msg):
    #rospy.loginfo(rospy.get_caller_id())
    #rospy.loginfo(type(msg))
    data = str(msg.transforms[0])
    #print(data)

    #vals = re.findall('\d+\.\d', data)
    vals = [float(s) for s in re.findall(r'-?\d+\.?\d*', data)]
    #print(vals)

    x_pos = vals[3]
    y_pos = vals[4]

    print("initial pos = [", x_pos, "," , y_pos, "]")
    #signal shutdown
    rospy.signal_shutdown("Initial Pos extracted")


def getPos():
    #while not rospy.is_shutdown():
    rospy.Subscriber("tf", TFMessage, pos_callback)

if __name__ == '__main__':
    rospy.init_node('turtlebot3_obstacle')
    try:
        getPos()
        #The program keeps running till node does not shutdown.
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
