#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import os
import cv2
import numpy as np
import time


def gen_images(folder_path):
    for fname in sorted(os.listdir(folder_path)):
        if os.path.splitext(fname)[1].lower() in ['.jpg', '.jpeg', '.png', '.bmp']:
            full_fname = os.path.join(folder_path, fname)
            img = cv2.imread(full_fname)
            assert img.dtype == np.uint8
            yield img

def main():
    rospy.init_node('image_folder_player')

    folder_path = rospy.get_param('~image_folder_path')
    img_pub_topic = rospy.get_param('~publish_topic')
    publish_rate = rospy.get_param('~publish_rate')

    publisher = rospy.Publisher(img_pub_topic, Image, queue_size=1)

    rate = rospy.Rate(publish_rate)
    bridge = CvBridge()
    img_generator = gen_images(folder_path)

    while not rospy.is_shutdown():
        try:
            cv_image = img_generator.next()
            publisher.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            rate.sleep()
        except StopIteration:
            img_generator = gen_images(folder_path)

if __name__ == '__main__':
    main()
