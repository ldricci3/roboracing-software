#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, sys
from glob import glob


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
    circles = np.uint32(np.around(circles))[0,:]
    mask = (img_good_channel > 150) * ((img_good_channel - 80) > img_bad_channel)
    mask = mask.astype(np.uint8) * 255
    score, circle = max(pair_score_circle(p, mask) for p in circles)[:3]
    if not is_similar_circle(circle, last_circle):
        score *= 0.3

    debug_img = cv2.circle(debug_img, (circle[0],circle[1]), circle[2], color, 2)
    debug_img = cv2.putText(debug_img, "%.2f"%score, (circle[0],circle[1]),
                            cv2.FONT_HERSHEY_PLAIN, 1.5, color, thickness=2)

    return score, circle, mask, debug_img

def main():
    rospy.init_node('stoplight_watcher_py')

    assert len(sys.argv) > 1
    path = sys.argv[1]

    last_red = last_green = (0,0,0)

    for fname in sorted(glob(os.path.join(path, '*.jpg'))):
        im = cv2.imread(fname)
        im = cv2.resize(im, (800,450))

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
            # print "red_circle", red_circle, red_score

            green_score, green_circle, green_mask, debug_img = proc_channel(
                im_green, im_red, circles.copy(), last_green, debug_img, (0,255,0))
            # print "green_circle", green_circle, green_score

            last_red = red_circle
            last_green = green_circle

        print "red" if red_score >= green_score else "green"

        cv2.imshow('detected circles', debug_img)
        cv2.imshow('red', red_mask)
        cv2.imshow('green', green_mask)

        k = cv2.waitKey(40)
        if k == ord('q'):
            break

if __name__ == '__main__':
    main()
