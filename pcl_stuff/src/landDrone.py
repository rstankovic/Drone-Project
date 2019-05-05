#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import numpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Empty


class takeOff:
    def __init__(self):
        self.stop_flight_plan_pub = rospy.Publisher('bebop/autoflight/stop', Empty, queue_size = 1)
        self.land_drone_pub = rospy.Publisher('bebop/land', Empty, queue_size = 1)


rospy.init_node('takeOff')
takeOff = takeOff()
takeOff.stop_flight_plan_pub.publish()
takeOff.land_drone_pub.publish()
