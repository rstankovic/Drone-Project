#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge, numpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from std_msgs.msg import Empty
from std_msgs.msg import String


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window",1)
        self.image_sub = rospy.Subscriber('bebop/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 1)
        self.camera_control_pub = rospy.Publisher('bebop/camera_control', Twist, queue_size = 1)
        self.objectCentered_pub = rospy.Publisher('bebop/ObjectCentered', Bool, queue_size = 1)
        self.do_a_flip_pub = rospy.Publisher('bebop/flip', UInt8, queue_size = 1)
        self.take_off_pub = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1)
        self.land_pub = rospy.Publisher('bebop/land', Empty, queue_size = 1)
        self.start_flight_plan_pub = rospy.Publisher('bebop/autoflight/start', String, queue_size = 1)
        self.pause_flight_plan_pub = rospy.Publisher('bebop/autoflight/pause', Empty, queue_size = 1)
        self.twist = Twist()



    def image_callback(self,msg):
        image = self.bridge.imgmsg_to_cv2(msg) #image_raw to cv2
        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([100,140,200])
        upper_yellow = numpy.array([150,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        masked = cv2.bitwise_and(image, image, mask=mask)
        h, w, d = image.shape
        #search_top = h/2
        #search_bot = search_top + 20
        #mask[0:search_top, 0:w] = 0
        #mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.line(image, (cx + 12, cy), (cx - 12, cy), (0, 0, 150), 1)
            cv2.line(image, (cx, cy + 12), (cx, cy - 12), (0, 0, 150), 1)
            cv2.circle(image, (cx, cy), 2, (0, 0, 150), -1)
            cv2.circle(image,(cx, cy), 10, (0, 0, 150), 1)
            cv2.putText(image, 'TARGET LOCKED', (cx + 20, cy + 20),cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 150))
            #cv2.line(image, (w/4,0),(w/4,h),(0,0,150),1)
            #cv2.line(image, (3*w/4,0),(3*w/4,h),(0,0,150),1)
            errx = cx - 410
            erry = cy - 225
            r = rospy.Rate(.2)
            #err_hi = cy - h/2
            #self.twist.linear.z = -numpy.tanh(err)/10
            #print('errx = ', errx)
            #print('erry = ', erry)
            #if errx > -50 and errx < 50 and erry > -30 and erry < 30:
                #self.objectCentered_pub.publish(True)
                #self.do_a_flip_pub.publish(1)
                #rospy.sleep(3)
                #self.start_flight_plan_pub.publish("flightPlan.mavlink")
                #rospy.sleep(3)

            #else:
            self.twist.angular.z = -numpy.tanh(errx/130)/2
            self.twist.angular.y = -numpy.tanh(erry)
            print("rotating speed: ", self.twist.angular.z) #just want to see how fast it is turning
            self.cmd_vel_pub.publish(self.twist)
            self.twist.angular.z = numpy.tanh(errx)
            print("camera speed y: ", self.twist.angular.y, "| camera speed z: ", self.twist.angular.z)
            self.camera_control_pub.publish(self.twist)
            self.objectCentered_pub.publish(False)


        #cv2.imshow("mask",mask)
        cv2.imshow("image", image)
        #cv2.imshow("masked",masked)
        cv2.waitKey(6)



rospy.init_node('follower')
follower = Follower()
rospy.spin()

'''    _, frame = cap.read()
    cv2.imshow("window",frame)

    k = cv2.waitKey(5) & 0xff
    if k == 27:
        break
        '''
