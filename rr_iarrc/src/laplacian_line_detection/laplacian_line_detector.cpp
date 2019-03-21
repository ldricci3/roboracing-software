#include <ros/ros.h>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include "nodelet/loader.h"

using namespace std;

cv::Mat kernel(int, int);
cv::Mat fillColorLines(const cv::Mat&, const cv::Mat&);
void cutEnvironment(const cv::Mat&, int offset);
cv::Mat cutSmall(const cv::Mat&, int);
void publishMessage(ros::Publisher, const cv::Mat&, std::string);
cv::Mat overlayBinaryGreen(cv::Mat&, const cv::Mat&);
cv::Mat removeAngels(const cv::Mat& img, int distanceFromEarth);

cv_bridge::CvImagePtr cv_ptr;
ros::Publisher pub, pub1, pub2, pub3;
int blockSky_height, blockWheels_height, blockBumper_height;
int perfect_lines_min_cut, Laplacian_threshold, adaptive_mean_threshold;

void img_callback(const sensor_msgs::ImageConstPtr& msg) {
    //Convert msg to Mat image
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;

    //Store original dimensions and resize
    int originalHeight = frame.rows;
    int originalWidth = frame.cols;
    cv::resize(frame, frame, cv::Size(400, 400));

    //Blur and convert to GrayScale
    cv::Mat frame_gray, frame_blur, detected_edges;
    cv::GaussianBlur(frame, frame_blur, cv::Size(5,5), 0);
    cv::cvtColor(frame_blur, frame_gray, cv::COLOR_BGR2GRAY);

    //Performing Adaptive Threshold to detect high-contrast lines in different lighting conditions
    //Cut image to ROI, erode small noise, and removes any blob less than a minimum area
    //The adaptive image (Little Noise / High Certainty) will represent where we are certain lines are located
    cv::Mat thres;
    cv::adaptiveThreshold(frame_gray, thres, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 5, -1);
    cutEnvironment(thres, 0);
    cv::erode(thres, thres, kernel(3,1));
    cv::Mat cut = cutSmall(thres, perfect_lines_min_cut);

    //Perform Laplacian operation to find strong edges
    //The laplacian image (High Noise / Low Certainty) will allow us to floodfill the whole line in case the adaptive
    //only found a small part of it
    cv::Mat lapl;
    cv::Laplacian(frame_gray, lapl, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT);
    cv::threshold(lapl, lapl, -Laplacian_threshold, 255, 1);
    cv::convertScaleAbs(lapl, lapl);

    //FloodFills the Laplacian at points where we are certain lines are located
    //Perform another cut to remove any small resultant noise
    cv::Mat fill = fillColorLines(lapl, cut);
    fill = cutSmall(fill, perfect_lines_min_cut);

    //Make debug green overlay img and resize resultant image to initial dimensions
    cv::Mat green_lines = overlayBinaryGreen(frame, fill);
    cv::resize(fill, fill, cv::Size(originalWidth, originalHeight));

    //Publish Messages
    publishMessage(pub, fill, "mono8");
    publishMessage(pub1, thres, "mono8");
    publishMessage(pub2, lapl, "mono8");
    publishMessage(pub3, green_lines, "bgr8");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Laplacian");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string subscription_node;
    nhp.param("perfect_lines_min_cut", perfect_lines_min_cut, 200);
    nhp.param("Laplacian_threshold", Laplacian_threshold, 2);
    nhp.param("adaptive_mean_threshold", adaptive_mean_threshold, 1);

    nhp.param("blockSky_height", blockSky_height, 220);
    nhp.param("blockWheels_height", blockWheels_height, 200);
    nhp.param("blockBumper_height", blockBumper_height, 200);

    nhp.param("subscription_node", subscription_node, std::string("/camera/image_color_rect"));

    pub = nh.advertise<sensor_msgs::Image>("/lines/detection_img", 1); //test publish of image
    pub1 = nh.advertise<sensor_msgs::Image>("/lines/debug_adaptive", 1);
    pub2 = nh.advertise<sensor_msgs::Image>("/lines/debug_laplacian", 1);
    pub3 = nh.advertise<sensor_msgs::Image>("/lines/debug_overlay", 1);
    auto img_real = nh.subscribe(subscription_node, 1, img_callback);

    ros::spin();
    return 0;
}

cv::Mat kernel(int x, int y) {
    return cv::getStructuringElement(cv::MORPH_RECT,cv::Size(x,y));
}

cv::Mat fillColorLines(const cv::Mat& lines, const cv::Mat& color_found) {
    cv::Mat color_left, lines_found(lines.rows,lines.cols,CV_8UC1,cv::Scalar::all(0));
    cv::Mat lines_remaining = lines.clone();
    lines.copyTo(color_left, color_found);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(color_left, contours,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    for( int i = 0; i < contours.size(); i++ ) {
        cv::floodFill(lines_remaining, contours[i][0], cv::Scalar(0));
    }
    cv::bitwise_xor(lines, lines_remaining, lines_found);
    return lines_found;
}


void cutEnvironment(const cv::Mat& img, int offset) {
    cv::rectangle(img,
                  cv::Point(0,0),
                  cv::Point(img.cols,img.rows / 3 + blockSky_height - offset),
                  cv::Scalar(0,0,0),CV_FILLED);

    cv::rectangle(img,
                  cv::Point(0,img.rows),
                  cv::Point(img.cols,2 * img.rows / 3 + blockWheels_height + offset),
                  cv::Scalar(0,0,0),CV_FILLED);

    cv::rectangle(img,
                  cv::Point(img.cols/3,img.rows),
                  cv::Point(2 * img.cols / 3, 2 * img.rows / 3 + blockBumper_height + offset),
                  cv::Scalar(0,0,0),CV_FILLED);
}

cv::Mat cutSmall(const cv::Mat& color_edges, int size_min) {
    cv::Mat contours_color(color_edges.rows,color_edges.cols,CV_8UC1,cv::Scalar::all(0));
    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(color_edges, contours,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    for( int i = 0; i < contours.size(); i++ ) {
        if (size_min < cv::arcLength(contours[i], false) ) {
            cv::drawContours(contours_color, contours, i, cv::Scalar(255), CV_FILLED, 8);
        }
    }
    return contours_color;
}

void publishMessage(ros::Publisher pub, const cv::Mat& img, std::string img_type) {
    if (pub.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptr->image = img;
        cv_ptr->encoding = img_type;
        cv_ptr->toImageMsg(outmsg);
        pub.publish(outmsg);
    }
}

cv::Mat overlayBinaryGreen(cv::Mat& frame, const cv::Mat& binary) {
    return frame.setTo(cv::Scalar(0,255,0), binary != 0);
}


cv::Mat removeAngels(const cv::Mat& img, int distanceFromEarth) {
    //Removes anything that extends from the top of the image to the bottom like glare from the sun
    cv::Mat top = img.clone();
    cv::rectangle(top,
                  cv::Point(0,img.rows / 3 + blockSky_height - distanceFromEarth),
                  cv::Point(img.cols,img.rows),
                  cv::Scalar(0),CV_FILLED);

    std::vector<cv::Point> locations;
    cv::findNonZero(top, locations);
    int number_of_angles = 20;
    for (int i = 0; i < number_of_angles; ++i) {
        cv::floodFill(img, locations[i], cv::Scalar(0));
        cv::floodFill(top, locations[i], cv::Scalar(0));
        cv::findNonZero(top, locations);
    }
    return img;
}
