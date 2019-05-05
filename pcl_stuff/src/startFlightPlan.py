#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import numpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import String


class takeOff:
    def __init__(self):
        self.start_flight_plan_pub = rospy.Publisher('bebop/autoflight/start', String, queue_size = 1)
        self.start_flight_plan_pub.publish("flightPlan.mavlink")


rospy.init_node('takeOff')
takeOff = takeOff()
