import numpy as np
import pandas as pd
import cv2
import os
import glob
import matplotlib.pyplot as plt
import pickle
import math
from sklearn.metrics import r2_score
from sklearn.metrics import mean_squared_error
import imutils
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import warnings

warnings.simplefilter('ignore', np.RankWarning)


def splitImage(img):
    leftHalf = img.copy()
    rightHalf = img.copy()
    cv2.rectangle(leftHalf, (width / 2, height), (width, 0), 0, -1)
    cv2.rectangle(rightHalf, (0, height), (width / 2, 0), 0, -1)
    return leftHalf, rightHalf


def findLoweredContour(side, frame):
    im2, contours, hierarchy = cv2.findContours(frame, 1, 2)
    pointList = cv2.findNonZero(side)
    if pointList is not None:
        for contour in contours:
            bottomPoint = tuple(pointList[-1][-1])
            if cv2.pointPolygonTest(contour, bottomPoint, False) >= 0:
                return contour
    return None


def callback(data):
    bridge = CvBridge()
    try:
        frame = bridge.imgmsg_to_cv2(data)
    except CvBridgeError as e:
        print(e)
        return None

    t0 = time.time()

    oldHeight, oldWidth = frame.shape[:2]
    global height, width
    height, width = 500, 500

    # resize image
    frame = cv2.resize(frame, (height, width))
    frame = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    # Three channels
    debug = np.stack([frame] * 3, axis=-1)
    lanes = np.zeros((height, width, 1), np.uint8)

    leftHalf, rightHalf = splitImage(frame)
    leftBottomContour = findLoweredContour(leftHalf, frame)
    # left_lane = Find related if present

    if leftBottomContour is not None:
        cv2.drawContours(rightHalf, [leftBottomContour], 0, 0, 1)
        cv2.drawContours(lanes, [leftBottomContour], -1, 255, -1)
        cv2.drawContours(debug, [leftBottomContour], -1, (255, 0, 0), -1)


    rightBottomContour = findLoweredContour(rightHalf, frame)
    # right_lane = Find related if present
    if rightBottomContour is not None:
        cv2.drawContours(lanes, [rightBottomContour], -1, 255, -1)
        cv2.drawContours(debug, [leftBottomContour], -1, (0, 0, 255), -1)


    # if have both draw middle
    # if one do offset
    # if zero straight

    t1 = time.time()
    print("time", t1 - t0)
    print("-----------------------------------------------")
    # msg = bridge.cv2_to_imgmsg(debug, encoding="bgr8")
    msg = bridge.cv2_to_imgmsg(lanes, encoding="mono8")
    debug_publisher.publish(msg)


def listener():
    global debug_publisher
    rospy.init_node("lane_detector")
    rospy.Subscriber("/lines_detection_img_transformed", Image, callback, buff_size=10 ** 8)
    debug_publisher = rospy.Publisher("de_image", Image, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    listener()
