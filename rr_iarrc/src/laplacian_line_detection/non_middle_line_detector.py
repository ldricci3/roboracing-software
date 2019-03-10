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


def getEquestions(img, a1, degree=2, color=(0, 255, 0), inputVisual=False):
    # Find the range of x values in a1
    min_a1_x, max_a1_x = min(a1[:, 0]), max(a1[:, 0])
    # Create an evenly spaced array that ranges from the minimum to the maximum
    # I used 100 elements, but you can use more or fewer.
    # This will be used as your new x coordinates
    new_a1_x = np.linspace(min_a1_x, max_a1_x, 200)
    # Fit a 3rd degree polynomial to your data
    a1_coefs = np.polyfit(a1[:, 0], a1[:, 1], degree)
    # Get your new y coordinates from the coefficients of the above polynomial

    # maxAdd = np.arange(max_a1_x, max_a1_x + 20 * (new_a1_x[1] - new_a1_x[0]))
    # minAdd = np.arange(min_a1_x - 20 *(new_a1_x[1] - new_a1_x[0]), min_a1_x)
    # new_a1_x = np.append(new_a1_x, maxAdd)
    # new_a1_x = np.append(minAdd, new_a1_x)

    new_a1_y = np.polyval(a1_coefs, new_a1_x)

    img = putArrayOnImg(img, new_a1_x, new_a1_y, color)

    return img


def getEquestionsY(img, a1, degree=3, color=(0, 255, 0), inputVisual=False):
    a1 = np.fliplr(a1)
    min_a1_x, max_a1_x = min(a1[:, 0]), max(a1[:, 0])
    new_a1_x = np.linspace(min_a1_x, max_a1_x, 200)
    a1_coefs = np.polyfit(a1[:, 0], a1[:, 1], degree)
    new_a1_y = np.polyval(a1_coefs, new_a1_x)

    img = putArrayOnImg(img, new_a1_y, new_a1_x, color)

    return img


def getMidline(img, a1, a2, poly_deg=3, n_points=100, plot=True):
    min_a1_x, max_a1_x = min(a1[:, 0]), max(a1[:, 0])
    new_a1_x = np.linspace(min_a1_x, max_a1_x, n_points)
    a1_coefs = np.polyfit(a1[:, 0], a1[:, 1], poly_deg)
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

    height, width, dim = img.shape
    point = np.array([width / 2, height - 50])
    anchor = (width / 2, height - 50)
    # print 1
    cv2.circle(img, anchor, 4, (255, 0, 255), -1)
    # print 2
    point = np.vstack((point, [width / 2, height - 50 + 1]))
    point = np.vstack((point, [width / 2, height - 50 - 1]))
    point = np.vstack((point + 1, [width / 2, height - 50]))
    point = np.vstack((point + 1, [width / 2, height - 50]))
    point = np.vstack((point + 1, [width / 2, height - 50 + 1]))
    for i in range(45, 100):
        point = np.vstack((point, [width / 2, height - i]))

    mid = np.vstack((midx, midy)).T
    mid = np.array(mid, np.int32)
    # mid = mid.reshape((-1, 1, 2))
    mid = np.vstack((mid, point))
    img = getEquestionsY(img, mid, 2, (0, 0, 255))

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


def getTopCorner(img, a1, anchor, i):
    print a1
    min_a1_x, max_a1_x = min(a1[:, 0]), max(a1[:, 0])
    new_a1_x = np.linspace(min_a1_x, max_a1_x, 200)
    a1_coefs = np.polyfit(a1[:, 0], a1[:, 1], 1)
    new_a1_y = np.polyval(a1_coefs, new_a1_x)

    new_a1_y = tuple(np.array(new_a1_y, np.int32))
    new_a1_x = tuple(np.array(new_a1_x, np.int32))
    cv2.line(img, anchor, (new_a1_x[i], new_a1_y[i]), (0, 0, 255), 1)

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
    dilation = cv2.dilate(img, np.ones((3, 3)))
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    skel = skeletonize(img)

    # Three channels
    im_debug = np.stack([img] * 3, axis=-1)

    height = np.size(img, 0)
    width = np.size(img, 1)

    anchor = (width / 2, height - 50)
    cv2.circle(im_debug, anchor, 4, (255, 0, 255), -1)

    cv2.line(img, (width / 2, 0), (width / 2, height), (0, 0, 0), 2)


    blank = np.zeros((height, width, 1), np.uint8)
    blank_mask = np.zeros((height + 2, width + 2, 1), np.uint8)
    offset1 = 50
    offset2 = 90
    offset_y = 60
    cv2.line(blank, (offset2, 0), (offset2, height), 255, 2)
    cv2.line(blank, (width - offset1, 0), (width - offset1, height), 255, 2)
    cv2.line(blank, (offset2, height - offset_y), (width - offset1, height - offset_y), 255, 2)

    warp_point = (width / 2, height - 100)
    point1 = (offset2, 0)
    point = (width - offset1, 0)
    cv2.line(blank, warp_point, point1, 255, 2)
    cv2.line(blank, warp_point, point, 255, 2)

    cv2.line(im_debug, warp_point, point1, (100, 100, 100), 2)
    cv2.line(im_debug, warp_point, point, (100, 100, 100), 2)

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
        if contours != []:
            leftContour = max(contours, key=cv2.contourArea)
            leftContour = np.vstack(leftContour).squeeze()
            im_debug = getEquestions(im_debug, leftContour)

    pixelpoints2 = cv2.findNonZero(img_half2)
    if pixelpoints2 is not None:
        point = tuple(pixelpoints2[-1][-1])
        cv2.circle(im_debug, point, 2, (200, 0,), -1)
        cv2.floodFill(img_copy2, blank_mask, point, 200, 50, 50)
        im_debug[img_copy2 == 200] = (200, 0, 0)

        blankFULL1 = np.zeros((height, width, 1), np.uint8);
        blankFULL1[img_copy2 == 200] = 255
        im2, contours, hierarchy = cv2.findContours(blankFULL1, 1, 2)
        if contours != []:
            rightContour = max(contours, key=cv2.contourArea)
            rightContour = np.vstack(rightContour).squeeze()

    print leftContour.ndim, rightContour.ndim
    if leftContour is not None and leftContour.ndim != 1 and rightContour is not None and rightContour.ndim != 1:
        try:
            im_debug = getMidline(im_debug, leftContour, rightContour)
        except:
            print("0")

    elif leftContour is not None and leftContour.ndim != 1:
        im_debug = getTopCorner(im_debug, leftContour, anchor, -1)
    elif rightContour is not None and rightContour.ndim != 1:
        im_debug = getTopCorner(im_debug, rightContour, anchor, 0)
    else:
        cv2.line(im_debug, (width / 2, height - 50), (width / 2, height - 150), (0, 0, 255), 1)

    im2, contours, hierarchy = cv2.findContours(img, 1, 2)
    cnt = np.vstack(contours[0]).squeeze()

    for cnt in contours:
        cnt = np.vstack(cnt).squeeze()
        if cnt.ndim != 1:
            im_debug = getEquestions(im_debug, cnt)

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
    debug_publisher.publish(msg)


def listener():
    global debug_publisher
    rospy.init_node("lane_detector")
    rospy.Subscriber("/lines_detection_img_transformed", Image, callback, buff_size=10 ** 8)
    debug_publisher = rospy.Publisher("de_image", Image, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    listener()
