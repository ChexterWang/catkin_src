#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

br = CvBridge()




if __name__ == '__main__':
    
    pub = rospy.Publisher('/camera/rgb/image_raw', Image,queue_size=10)
    rospy.init_node("test_publiser", anonymous=True)
    image = cv2.imread("/home/brian/catkin_ws/src/cube_slam/detect_3d_cuboid/data/0000_rgb_raw.jpg");
    print(type(image))
    # vidcap = cv2.VideoCapture('/home/brian/Downloads/test.mp4')
    # success,image = vidcap.read()
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    rate = rospy.Rate(10) # 10hz
    while not (rospy.is_shutdown() and success):
        msg = br.cv2_to_imgmsg(image, "rgb8")
        pub.publish(msg)
        # success,image = vidcap.read()
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        rate.sleep()
