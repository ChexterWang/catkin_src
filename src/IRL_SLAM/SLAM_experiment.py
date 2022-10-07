#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

import rospy
import numpy as np
import cv2


from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import glob
import sys
from line_lbd.msg import final_pose
from line_lbd.msg import combined_boxes
from line_lbd.msg import My_image
from line_lbd.msg import updateServer

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import Virtual_Angle as VA


br = CvBridge()


def main():
        print("data path: ",sys.argv[1])
        rospy.init_node("experiment_publiser", anonymous=True)
        pub = rospy.Publisher('/camera/rgb/image_raw', Image,queue_size=10)
        my_pub = rospy.Publisher('/camera/rgb/My_image', My_image,queue_size=10)
        # pub = rospy.Publisher('/camera/image_raw', Image,queue_size=10)
        
        file_path = sys.argv[1]+"rgb.txt"
        folder_name = sys.argv[1]

        with open(file_path) as txt:
            for i in range(3): #header
                header = txt.readline()
                print(header)
            line = txt.readline().split(" ")
            # print(type(rospy.Time.now()))
            id_ = 0
            while(line and not rospy.is_shutdown()):
                print(line)
                image = cv2.imread(folder_name+line[1].strip())
                cv2.imshow("image",image)
                cv2.waitKey(40)
                msg = br.cv2_to_imgmsg(image, "bgr8")
                t = rospy.Time.from_sec(float(line[0].strip()))
                msg.header.stamp = t
                pub.publish(msg)
                My_msg = My_image()
                My_msg.id = str(id_)
                id_+=1
                My_msg.image_now = msg
                
                my_pub.publish(My_msg)

                line = txt.readline().split(" ")


if __name__ == '__main__':

    main()
    