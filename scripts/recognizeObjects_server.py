#!/usr/bin/env python

from mimason.srv import *
from mimason.msg import *

import rospy
import os
import cv2
import scipy.io as sio
import time
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud


class Recognizer:
    def __init__(self):
        self.bridge = CvBridge()
        self.update_im = False
        self.update_pc = False
        self.cv_image = None
        self.cloud = None
        self.mask = -1*np.ones((480,640))
        self.categories = []
        self.scores = []
        self.count = 0
        self.do_recognition = False
        self.pub_mask = rospy.Publisher('recognition_mask_image',Image,queue_size = 1)
        self.pub_rec_info = rospy.Publisher('recognition_information',rec_info,queue_size = 1)
        self.pub_cloud = rospy.Publisher('recognition_pcl',PointCloud,queue_size = 1)

        # rate = rospy.Rate(1)
        # rospy.init_node('recognizer')
        # rospy.Subscriber("redo_recognition", bool, self.update_redo)
        rospy.Subscriber("cloud", PointCloud, self.update_point_cloud)
        rospy.Subscriber("/rgb/image_raw", Image, self.update_rgb)


    def update_point_cloud(self,update):
        # if(self.update_pc):
        print "need correct topic name!"
        self.cloud = update
        # self.update_pc = False
    def update_rgb(self,im_message):
        # print 'ere'
        # if(self.update_im):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(im_message, "bgr8")
            # self.update_im = False
        except CvBridgeError, e:
            print e
        if self.do_recognition:
            self.recognize()
    def recognize(self):
        self.do_recognition = False

        #get updated image and point cloud
        # self.update_im = True
        # self.update_pc = True



        #wait some time amd ensure image and PointCloud updated
        # while not (self.update_pc and self.update_im):
            # print 'in loop'
            # time.sleep(.1)
        cv2.imwrite('/home/hcrws1/Documents/Toolbox/sds_eccv2014/image_to_recognize.jpg',self.cv_image)
        # os.system("matlab demo_sds.m")
        # os.system(r"""cd /home/hcrws1/Documents/Toolbox/sds_eccv2014 && matlab -nodisplay -nosplash -nodesktop -r "run('startup_sds.m');run('demo_apc.m');exit;" """)
        os.system(r"""cd /home/hcrws1/Documents/Toolbox/sds_eccv2014 && matlab -r "run('startup_sds.m');run('demo_apc.m');exit;" """)
        rr = sio.loadmat('/home/hcrws1/Documents/Toolbox/sds_eccv2014/recognition_results.mat')
        self.mask = rr['mask']
        # mask = np.zeros((8,8))
        # print np.shape(mask)
        self.categories = rr['det2cat']
        self.scores = rr['det2scores']
        # return recognizeObjectsResponse(self.bridge.cv2_to_imgmsg(mask, "mono8"),categories,confidence)
        self.pub_mask.publish(self.bridge.cv2_to_imgmsg(self.mask,"mono8"))
        self.pub_rec_info.publish(rec_info(len(self.scores), ["to do"]*len(self.scores),self.categories,self.scores))
        # self.pub_cloud.publish(self.cloud)
    def acknowledge(self,req):
        self.do_recognition = True
        self.count += 1
        return self.count

def main(args):
  r = Recognizer()
  print "initializing"
  rospy.init_node('recognizeObjects_server')
  s = rospy.Service('recognizeObjects', recognizeObjects, r.acknowledge)
  print "Ready to recognize"
  rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
