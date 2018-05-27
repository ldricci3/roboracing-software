#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


last_red = last_green = (0,0,0)
latest_img_msg = None
bridge = CvBridge()


# rate a ROI circle by computing the proportion of the circle that is more green
#   than red or vice cersa
def rate_circle(xyr, channel_mask):
    try:
        x, y, r = xyr
        channel_mask_crop = channel_mask[int(y-r):int(y+r), int(x-r):int(x+r)]
        circle_mask = np.zeros_like(channel_mask_crop)
        r = int(r)
        circle_mask = cv2.circle(circle_mask, (r, r), r, 255, -1)
        rating = np.count_nonzero(np.bitwise_and(circle_mask, channel_mask_crop)) \
                 / float(np.count_nonzero(circle_mask))
        return rating
    except:
        return 0

def is_similar_circle(xyr1, xyr2):
    x1, y1, r1 = xyr1
    x2, y2, r2 = xyr2
    center_dist = ((float(x2)-x1)**2 + (float(y2)-y1)**2) ** 0.5
    return (min(r1, r2) > center_dist)

def pair_score_circle(circle, channel_mask):
    return (rate_circle(circle, channel_mask), tuple(circle))

def proc_channel(img_good_channel, img_bad_channel, circles, last_circle, debug_img, color):
    circles = np.uint32(np.around(circles))[0,:3]
    mask = (img_good_channel > 150) * ((img_good_channel - 80) > img_bad_channel)
    mask = mask.astype(np.uint8) * 255
    score, circle = max(pair_score_circle(p, mask) for p in circles)
    if not is_similar_circle(circle, last_circle):
        score *= 0.3

    debug_img = cv2.circle(debug_img, (circle[0],circle[1]), circle[2], color, 2)
    debug_img = cv2.putText(debug_img, "%.2f"%score, (circle[0],circle[1]),
                            cv2.FONT_HERSHEY_PLAIN, 1.5, color, thickness=2)

    return score, circle, mask, debug_img

def img_callback(img_msg):
    global latest_img_msg
    latest_img_msg = img_msg

# (image) -> {"red", "green", None}
def process_img(im):
    global last_red, last_green

    im_red = im[:,:,2].copy()
    im_green = im[:,:,1].copy()
    im_blue = im[:,:,0].copy()

    im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    im_gray = cv2.medianBlur(im_gray, 7)

    circles = cv2.HoughCircles(im_gray, cv2.HOUGH_GRADIENT, 2, 50,
                        param1=70, param2=50, minRadius=8, maxRadius=75)

    debug_img = np.stack([im_gray]*3, axis=-1)

    if type(circles) != type(None):
        red_score, red_circle, red_mask, debug_img = proc_channel(
                im_red, im_green, circles.copy(), last_red, debug_img, (0,0,255))

        green_score, green_circle, green_mask, debug_img = proc_channel(
                im_green, im_red, circles.copy(), last_green, debug_img, (0,255,0))

        last_red = red_circle
        last_green = green_circle

        cv2.imshow('red', red_mask)
        cv2.imshow('green', green_mask)
        cv2.imshow('detected circles', debug_img)
        cv2.waitKey(1)

        is_red = red_score > 0.8
        is_green = green_score > 0.8

        if not (is_red or is_green):
            return None
        else:
            return 'red' if is_red else 'green'


def main():
    rospy.init_node('stoplight_watcher_py')

    stoplight_topic = rospy.get_param('~stoplight_topic')
    img_topic = rospy.get_param('~img_topic')

    publisher = rospy.Publisher(stoplight_topic, Bool, queue_size=1)
    rospy.Subscriber(img_topic, Image, img_callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if type(latest_img_msg) != type(None) and publisher.get_num_connections() > 0:
            img = bridge.imgmsg_to_cv2(latest_img_msg, desired_encoding='bgr8')

            scale_factor = 480. / img.shape[0]
            if abs(scale_factor - 1.) > 0.1:
                img = cv2.resize(img, None, fx=scale_factor, fy=scale_factor)

            res = process_img(img)
            start_detected = (res == 'green')

            msg = Bool()
            msg.data = start_detected
            publisher.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    main()
