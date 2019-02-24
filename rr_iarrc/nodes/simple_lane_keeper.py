#! /usr/bin/python

from __future__ import print_function

import warnings
import time

import numpy as np
import cv2

import rospy
import cv_bridge
from sensor_msgs.msg import Image
from rr_platform.msg import steering as Steering
from rr_platform.msg import speed as Speed


dist_max = 2.5
px_per_meter = 50
pursuit_dist = 2.5
turn_max = 0.4


class Controller(object):
    def __init__(self, kP, kI, kD, decay=0.9):
        self.last_error = 0
        self.last_time = time.time()
        self.acc_error = 0
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.decay = decay

    def update(self, error):
        dt = time.time() - self.last_time
        self.acc_error = self.acc_error ** (self.decay * dt)
        error_gradient = (error - self.last_error) / dt

        self.last_time += dt
        self.last_error = error

        return self.kP * error + self.kI * self.acc_error + self.kD * error_gradient


def get_base_position(label_img, i, x, y, w, h):
    points_mask_img = np.zeros_like(label_img, dtype=np.uint8)
    points_mask_img[label_img == i] = 1

    region_mask_img = np.zeros_like(points_mask_img)
    y_bottom = y + h
    cv2.rectangle(region_mask_img, (x, y_bottom - 2), (x + w, y_bottom), color=1, thickness=-1)

    rs, cs = np.where(np.logical_and(points_mask_img, region_mask_img))
    x_mean = float(np.mean(cs) - (label_img.shape[1] / 2)) / px_per_meter
    y_mean = float(label_img.shape[0] - np.mean(rs)) / px_per_meter
    return x_mean, y_mean


def label_base_distance(label_img, stats, i):
    x, y, w, h, _ = stats[i]
    base_x, base_y = get_base_position(label_img, i, x, y, w, h)
    return ((base_x * base_x) + (base_y * base_y)) ** 0.5


def make_kernel(ksize):
    kernel = np.zeros((2*ksize+1, 2*ksize+1), dtype=np.uint8)
    cv2.circle(kernel, (ksize, ksize), ksize/2, color=255, thickness=-1)
    return kernel


# get callable line model
def fit_line(labels_img, i):
    rows, cols = np.where(labels_img == i)
    indices = np.random.random_integers(0, len(rows)-1, 20)
    rows = rows[indices]
    cols = cols[indices]

    ins = np.float32(labels_img.shape[0] - rows) / px_per_meter
    outs = np.float32(cols - (labels_img.shape[1] / 2)) / px_per_meter

    p = np.polyfit(ins, outs, deg=1)
    return np.poly1d(p)


def image_callback(msg):
    bridge = cv_bridge.CvBridge()
    img = bridge.imgmsg_to_cv2(msg)

    img = cv2.dilate(img, make_kernel(11))
    img = cv2.erode(img, make_kernel(9))

    n_labels, labels_img, stats, centroids = cv2.connectedComponentsWithStats(img, connectivity=8, ltype=cv2.CV_32S)

    px_crop = labels_img.shape[0] - int(dist_max * px_per_meter)
    labels_img_cropped = labels_img.copy()
    labels_img_cropped[:px_crop, :] = 0

    valid_labels = []
    for i in range(1, n_labels):
        if stats[i, cv2.CC_STAT_AREA] > 100 and np.count_nonzero(labels_img_cropped == i) > 0:
            valid_labels.append(i)

    left_lines_labels = []
    right_lines_labels = []
    for i in valid_labels:
        line = fit_line(labels_img_cropped, i)
        l = (left_lines_labels if line(0) < 0 else right_lines_labels)
        l.append((line, i))

    def line_label_dist(line_label):
        _, i = line_label
        return label_base_distance(labels_img_cropped, stats, i)

    left_line = None
    right_line = None
    if len(left_lines_labels) > 0:
        left_line, _ = min(left_lines_labels, key=line_label_dist)
    if len(right_lines_labels) > 0:
        right_line, _ = min(right_lines_labels, key=line_label_dist)

    if left_line is not None and right_line is not None:
        target = (left_line(pursuit_dist) + right_line(pursuit_dist)) / 2
    elif left_line is not None:
        target = left_line(pursuit_dist) + 1
    elif right_line is not None:
        target = right_line(pursuit_dist) - 1
    else:
        target = 0

    steer_angle = controller.update(target)
    if steer_angle < -turn_max:
        steer_angle = -turn_max
    elif steer_angle > turn_max:
        steer_angle = turn_max

    steer_msg = Steering()
    steer_msg.angle = steer_angle
    steer_pub.publish(steer_msg)

    speed_msg = Speed()
    # speed_msg.speed = 1.0 - 1.0 * abs(steer_angle)
    speed_msg.speed = 1.25
    speed_pub.publish(speed_msg)

    # ------------------ Draw debug image ----------------
    debug_img = np.stack([img] * 3, axis=-1)

    def x_px(x_real):
        return int((x_real * px_per_meter) + (debug_img.shape[1] / 2))

    def y_px(y_real):
        return int(debug_img.shape[0] - (y_real * px_per_meter))

    cv2.circle(debug_img, (x_px(target), y_px(pursuit_dist)), 3, color=(0, 255, 0), thickness=-1)

    if left_line is not None:
        cv2.line(debug_img,
                 (x_px(left_line(0)), y_px(0)),
                 (x_px(left_line(dist_max)), y_px(dist_max)),
                 color=(255, 0, 0), thickness=2)
    if right_line is not None:
        cv2.line(debug_img,
                 (x_px(right_line(0)), y_px(0)),
                 (x_px(right_line(dist_max)), y_px(dist_max)),
                 color=(0, 0, 255), thickness=2)

    car_length = int(0.65 * px_per_meter)
    car_width = int(0.4 * px_per_meter)
    cv2.rectangle(debug_img,
                  ((debug_img.shape[1] - car_width) / 2, debug_img.shape[0] - car_length),
                  ((debug_img.shape[1] + car_width) / 2, debug_img.shape[0]),
                  color=(0, 255, 0), thickness=3)

    cv2.imshow("debug", debug_img)
    cv2.waitKey(1)


if __name__ == '__main__':
    # global speed_pub, steer_pub, controller

    rospy.init_node("simple_lane_keeper")

    rospy.Subscriber("/lines_detection_img_transformed", Image, image_callback)
    speed_pub = rospy.Publisher("/plan/speed", Speed, queue_size=1)
    steer_pub = rospy.Publisher("/plan/steering", Steering, queue_size=1)

    controller = Controller(0.4, 0, 0)

    warnings.simplefilter("ignore", category=np.RankWarning)

    rospy.spin()
