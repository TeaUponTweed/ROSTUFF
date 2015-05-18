#!/usr/bin/env python

from mimason.srv import *
import rospy
import os
import cv2
import scipy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class Recognizer:
    def __init__(self):
        self.bridge = CvBridge()
        # rospy.init_node('recognizer')
        rospy.Subscriber("redo_recognition", bool, self.update_redo)
        rospy.Subscriber("cloud", sensor_msgs/PointCloud, self.update_point_cloud)
        rospy.Subscriber("image_raw", sensor_msgs/Image, self.update_rgb)
    def update_redo(self,update):
        self.do_recognition = update
    def update_point_cloud(self,update):
        if(self.do_recognition):
            self.cloud = update
    def update_rgb(self,im_message):
        if(self.do_recognition):
            try:
                self.cv_image = bridge.imgmsg_to_cv2(im_message, "bgr8")
            except CvBridgeError, e:
                print e
    def recognize(self,req):


def main(args):
  r = Recognizer()
  rospy.init_node('recognizer_server')
  s = rospy.Service('recognizer', recognizeObjects, r.recognize)
  print "Ready to recognize"

if __name__ == '__main__':
    main(sys.argv)
