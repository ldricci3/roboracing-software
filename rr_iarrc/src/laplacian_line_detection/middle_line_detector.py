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
from sympy import *

warnings.simplefilter('ignore', np.RankWarning)


def getRelatedLines(img):
    height = np.size(img, 0)
    width = np.size(img, 1)
    # cv2.line(img, (width/2, 0), (width/2, height), 0, 2)
    # cv2.line(img, (width / 3, 0), (width / 3, height), 0, 2)
    # cv2.line(img, (width *2 / 3, 0), (width *2 / 3, height), 0, 2)

    im2, contours, hierarchy = cv2.findContours(img, 1, 2)
    skel = skeletonize(img)
    im_debug = np.stack([skel] * 3, axis=-1)
    blank_frame = np.zeros(img.shape, np.uint8)
    list = np.zeros([1, 3])
    line1_coef_list = np.zeros([1, 2])
    line2_coef_list = np.zeros([1, 2])
    pointA_list = np.zeros([1, 2])
    pointB_list = np.zeros([1, 2])
    pointC_list = np.zeros([1, 2])
    pointD_list = np.zeros([1, 2])

    for contour in contours:
        blank_frame = np.zeros(img.shape, np.uint8)
        cv2.drawContours(blank_frame, [contour], 0, 255, -1)
        contourSkel = cv2.bitwise_and(blank_frame, skel)
        contourSkelPoints = cv2.findNonZero(contourSkel)
        if contourSkelPoints is not None:
            contourSkelPoints = np.vstack(contourSkelPoints).squeeze()
            if contourSkelPoints.ndim == 2:
                im_debug, a1_coefs, line_coefs1, endPnt1, startPnt1, line_coefs2, endPnt2, startPnt2 = getEquestions(im_debug,
                                                                                               contourSkelPoints)
                list = np.vstack((list, a1_coefs))
                line1_coef_list = np.vstack((line1_coef_list, line_coefs1))
                line2_coef_list = np.vstack((line2_coef_list, line_coefs2))
                pointA_list = np.vstack((pointA_list, endPnt1))
                pointB_list = np.vstack((pointB_list, endPnt2))
                pointD_list = np.vstack((pointD_list, startPnt1))
                pointC_list = np.vstack((pointC_list, startPnt2))
    list = list[1:]
    line1_coef_list = line1_coef_list[1:]
    line2_coef_list = line2_coef_list[1:]
    pointA_list = pointA_list[1:]
    pointB_list = pointB_list[1:]


    print line1_coef_list
    print line2_coef_list
    print len(contours)

    for i in range(len(line1_coef_list)):
        for j in range(i + 1, len(line1_coef_list)):
            print i, "to", j
            im_debug = isConnectedTo(im_debug, line1_coef_list[i], line2_coef_list[j], pointA_list[i], pointD_list[i],
                                     pointB_list[j], pointC_list[j])
            im_debug = isConnectedTo(im_debug, line2_coef_list[i], line2_coef_list[j], pointB_list[i], pointC_list[i],
                                     pointA_list[j], pointD_list[j])

    # print list[1:]

    return im_debug


def skeletonize(img):
    size = np.size(img)
    skel = np.zeros(img.shape, np.uint8)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    done = False

    while (not done):
        eroded = cv2.erode(img, element)
        temp = cv2.dilate(eroded, element)
        temp = cv2.subtract(img, temp)
        skel = cv2.bitwise_or(skel, temp)
        img = eroded.copy()

        zeros = size - cv2.countNonZero(img)
        if zeros == size:
            done = True

    return skel


def intersect(A, B, C, D):
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)


def ccw(A, B, C):
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])


def getEquestions(img, a1):
    min_a1_x, max_a1_x = min(a1[:, 0]), max(a1[:, 0])
    new_a1_x = np.linspace(min_a1_x, max_a1_x, 100)
    a1_coefs1, residuals1, extra1, extra2, extra3 = np.polyfit(a1[:, 0], a1[:, 1], 2, full=True)
    new_a1_y = np.polyval(a1_coefs1, new_a1_x)

    endPnt1 = (new_a1_x[-1], new_a1_y[-1])
    secEndPnt1 = (new_a1_x[-5], new_a1_y[-5])
    line_coefs1 = np.polyfit([secEndPnt1[0], endPnt1[0]], [secEndPnt1[1], endPnt1[1]], 1)
    newEnd_x = int(endPnt1[0] + 40)
    newEnd_y = int(np.polyval(line_coefs1, newEnd_x))
    secEndPnt1 = tuple(np.array(secEndPnt1, np.int32))
    endPnt1 = tuple(np.array(endPnt1, np.int32))
    cv2.circle(img, endPnt1, 4, (255, 255, 0), -1)
    # cv2.circle(img, secEndPnt, 4, (0, 255, 255), -1)
    # cv2.circle(img, (newEnd_x, newEnd_y), 4, (255, 255, 0), -1)
    cv2.line(img, secEndPnt1, (newEnd_x, newEnd_y), (255, 0, 0), 2)
    startPoint1 = (newEnd_x, newEnd_y)

    endPnt = (new_a1_x[0], new_a1_y[0])
    secEndPnt = (new_a1_x[5], new_a1_y[5])
    line_coefs2 = np.polyfit([secEndPnt[0], endPnt[0]], [secEndPnt[1], endPnt[1]], 1)
    newEnd_x = int(endPnt[0] - 40)
    newEnd_y = int(np.polyval(line_coefs2, newEnd_x))
    secEndPnt = tuple(np.array(secEndPnt, np.int32))
    endPnt = tuple(np.array(endPnt, np.int32))
    cv2.circle(img, endPnt, 4, (0, 255, 255), -1)
    # cv2.circle(img, secEndPnt, 4, (0, 255, 255), -1)
    # cv2.circle(img, (newEnd_x, newEnd_y), 4, (255, 255, 0), -1)
    cv2.line(img, secEndPnt, (newEnd_x, newEnd_y), (255, 0, 0), 2)
    startPoint2 = (newEnd_x, newEnd_y)

    # min_a1_y, max_a1_y = min(a1[:, 1]), max(a1[:, 1])
    # new_a1_y = np.linspace(min_a1_y, max_a1_y, 100)
    # a1_coefs2, residuals2, extra1, extra2, extra3 = np.polyfit(a1[:, 1], a1[:, 0], 3, full=True)
    # print(residuals1, residuals2)

    img = putArrayOnImg(img, new_a1_x, new_a1_y, (0, 255, 0))
    return img, a1_coefs1, line_coefs1, secEndPnt1, startPoint1, line_coefs2, secEndPnt, startPoint2


def isConnectedTo(img, a1, a2, pointA, pointB, pointC, pointD):
    pointA = np.array(pointA, np.int32)
    pointB = np.array(pointB, np.int32)
    pointC = np.array(pointC, np.int32)
    pointD = np.array(pointD, np.int32)
    print pointA, pointB, pointC, pointD
    cv2.circle(img, tuple(pointA), 4, (255, 255, 255), -1)
    cv2.circle(img, tuple(pointB), 4, (255, 255, 255), -1)
    cv2.circle(img, tuple(pointC), 4, (0, 0, 255), -1)
    cv2.circle(img, tuple(pointD), 4, (0, 0, 255), -1)

    if not intersect(pointA, pointB, pointC, pointD):
        print False
        return img
    print True

    new_a1_x = np.linspace(0, 500, 500)
    f = np.polyval(a1, new_a1_x)
    g = np.polyval(a2, new_a1_x)
    idx = np.argwhere(np.diff(np.sign(f - g))).flatten()
    if len(idx) != 0:
        cv2.circle(img, (idx[0], int(f[idx][0])), 4, (100, 100, 100), -1)

    return img


def getMidline(img, a1, a2, poly_deg=3, n_points=100, plot=True):
    min_a1_x, max_a1_x = min(a1[:, 0]), max(a1[:, 0])
    new_a1_x = np.linspace(min_a1_x, max_a1_x, n_points)
    a1_coefs, residuals, rank, singular_values, rcondz = np.polyfit(a1[:, 0], a1[:, 1], poly_deg)
    print(residuals)
    # maxAdd = np.arange(max_a1_x, max_a1_x + 20)
    # minAdd = np.arange(min_a1_x - 20, min_a1_x)
    # new_a1_x = np.append(new_a1_x, maxAdd)
    # new_a1_x = np.append(minAdd, new_a1_x)

    new_a1_y = np.polyval(a1_coefs, new_a1_x)

    min_a2_x, max_a2_x = min(a2[:, 0]), max(a2[:, 0])
    new_a2_x = np.linspace(min_a2_x, max_a2_x, n_points)
    a2_coefs = np.polyfit(a2[:, 0], a2[:, 1], poly_deg)

    new_a2_y = np.polyval(a2_coefs, new_a2_x)

    combined1 = np.vstack((new_a1_x, new_a1_y)).T
    combined1 = np.array(combined1, np.int32)
    combined1 = combined1[combined1[:, 1].argsort()]

    combined2 = np.vstack((new_a2_x, new_a2_y)).T
    combined2 = np.array(combined2, np.int32)
    combined2 = combined2[combined2[:, 1].argsort()]

    midx = [np.mean([combined1[i][0], combined2[i][0]]) for i in range(n_points)]
    midy = [np.mean([combined1[i][1], combined1[i][1]]) for i in range(n_points)]

    img = putArrayOnImg(img, new_a1_x, new_a1_y)
    img = putArrayOnImg(img, new_a2_x, new_a2_y)
    img = putArrayOnImg(img, midx, midy)
    return img


def putArrayOnImg(img, new_a1_x, new_a1_y, color=(0, 255, 0)):
    combined = np.vstack((new_a1_x, new_a1_y)).T
    combined = np.array(combined, np.int32)
    combined = combined.reshape((-1, 1, 2))
    cv2.polylines(img, [combined], False, color)
    return img


def callback(data):
    bridge = CvBridge()
    try:
        img = bridge.imgmsg_to_cv2(data)
    except CvBridgeError as e:
        print(e)
        return None

    t0 = time.time()

    # resize image
    img = cv2.resize(img, (500, 500))
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones((4, 4), np.uint8))

    # skel = getRelatedLines(img)
    skel = skeletonize(img)

    # Three channels
    im_debug = np.stack([img] * 3, axis=-1)

    height = np.size(img, 0)
    width = np.size(img, 1)

    blank = np.zeros((height, width, 1), np.uint8)
    blank_mask = np.zeros((height + 2, width + 2, 1), np.uint8)
    offset1 = 50
    offset2 = 90
    offset_y = 60
    cv2.line(blank, (offset2, 0), (offset2, height), 255, 2)
    cv2.line(blank, (width - offset1, 0), (width - offset1, height), 255, 2)
    cv2.line(blank, (offset2, height - offset_y), (width - offset1, height - offset_y), 255, 2)

    cv2.line(im_debug, (offset2, 0), (offset2, height), (100, 100, 100), 2)
    cv2.line(im_debug, (width - offset1, 0), (width - offset1, height), (100, 100, 100), 2)
    cv2.line(im_debug, (offset2, height - offset_y), (width - offset1, height - offset_y), (100, 100, 100), 2)

    img[img != 0] = 255
    blank_and = cv2.bitwise_and(blank, img)
    # blank_and_dil = cv2.dilate(blank_and, np.ones((3, 3), np.uint8))
    im_debug[blank_and != 0] = (255, 0, 255)

    img_half1 = blank_and.copy()
    img_half2 = blank_and.copy()
    cv2.rectangle(img_half1, (width / 2, height), (width, 0), (0, 0, 0), -1)
    cv2.rectangle(img_half2, (0, height), (width / 2, 0), (0, 0, 0), -1)

    img_copy = img.copy()
    img_copy2 = cv2.dilate(img_copy, np.ones((2, 2), np.uint8))

    rightContour = np.ndarray([0])
    leftContour = np.ndarray([0])

    pixelpoints = cv2.findNonZero(img_half1)
    if pixelpoints is not None:
        point = tuple(pixelpoints[-1][-1])
        cv2.circle(im_debug, point, 2, (0, 0, 255), -1)
        cv2.floodFill(img_copy2, blank_mask, point, 100, 30, 30)
        im_debug[img_copy2 == 100] = (0, 0, 255)

        blankFULL = np.zeros((height, width, 1), np.uint8);
        blankFULL[img_copy2 == 100] = 255
        im2, contours, hierarchy = cv2.findContours(blankFULL, 1, 2)
        leftContour = max(contours, key=cv2.contourArea)
        leftContour = np.vstack(leftContour).squeeze()
        im_debug = getEquestions(im_debug, leftContour)

    pixelpoints2 = cv2.findNonZero(img_half2)
    if pixelpoints2 is not None:
        point = tuple(pixelpoints2[-1][-1])
        print point
        return
        cv2.circle(im_debug, point, 2, (200, 0,), -1)
        cv2.floodFill(img_copy2, blank_mask, point, 200, 50, 50)
        im_debug[img_copy2 == 200] = (200, 0, 0)

        blankFULL1 = np.zeros((height, width, 1), np.uint8);
        blankFULL1[img_copy2 == 200] = 255
        im2, contours, hierarchy = cv2.findContours(blankFULL1, 1, 2)
        rightContour = max(contours, key=cv2.contourArea)
        rightContour = np.vstack(rightContour).squeeze()

    if leftContour is None or leftContour.ndim != 1 and rightContour is None or rightContour.ndim != 1:
        try:
            im_debug = getMidline(im_debug, leftContour, rightContour)
        except:
            print("0")

    im2, contours, hierarchy = cv2.findContours(img, 1, 2)
    cnt = np.vstack(contours[0]).squeeze()

    # for cnt in contours:
    #     cnt = np.vstack(cnt).squeeze()
    #     if cnt.ndim != 1:
    #         im_debug = getEquestions(im_debug, cnt)
    #
    # try:
    #     c1 = contours[-1]
    #     c1 = np.vstack(c1).squeeze()
    #     c2 = contours[-2]
    #     c2 = np.vstack(c2).squeeze()
    #     if c1.ndim != 1 and c2.ndim != 1:
    #         im_debug = getMidline(im_debug, c1, c2)
    # except:
    #     print("0")

    # im_debug = blank

    t1 = time.time()
    print("time", t1 - t0)
    print("-----------------------------------------------")
    msg = bridge.cv2_to_imgmsg(im_debug, encoding="bgr8")
    # msg = bridge.cv2_to_imgmsg(skel, encoding="bgr8")
    debug_publisher.publish(msg)


def listener():
    global debug_publisher
    rospy.init_node("lane_detector")
    rospy.Subscriber("/lines_detection_img2", Image, callback, buff_size=10 ** 8)
    debug_publisher = rospy.Publisher("de_image", Image, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    listener()
