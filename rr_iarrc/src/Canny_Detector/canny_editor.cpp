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

using namespace cv;

ros::Publisher pub;
ros::Publisher debug_pub;
ros::Publisher debug_pub2;
ros::Publisher debug_pub3;
ros::Publisher debug_pub4;

cv::Mat kernel(int, int);
cv::Mat cutEnvironment(cv::Mat);
cv::Mat fillColorLines(cv::Mat, cv::Mat);
cv::Mat findColorLine(cv::Mat, cv::Mat, std::string);
cv::Mat cutSmallCurvedLines(cv::Mat, int, int, bool x = false, double y = 1.0);
cv::Mat findMaximumAboveLimit(cv::Mat, int, int);
cv::Mat ignoreEdge(cv::Mat);
void img_callback(const sensor_msgs::ImageConstPtr& msg);

int min_line_threshold, max_line_threshold;
int blockSky_height, blockWheels_height, blockBumper_height;
int yellow_low_H, yellow_high_H, yellow_low_S, yellow_low_V;
int white_low_H, white_high_H, white_low_S, white_low_V;


int blockSky_height_bar = 150, blockWheels_height_bar = 30, blockBumper_height_bar = 0;

int ylow_H = 20, yhigh_H = 40, ylow_S = 20, ylow_V = 100, color_choice;
int wlow_H = 90, whigh_H = 115, wlow_S = 0, wlow_V = 125 , prev_choice = 2;
int low_H, high_H, low_S, low_V;
int min_line_threshold_bar = 60;
int max_line_threshold_bar = 0;
const char* window_name = "Edge Map";
cv::Mat frame, detected_edges, yellow_found, white_found, combined_edges;
sensor_msgs::ImageConstPtr msg1;

void CannyThreshold(int x, void*) {

    if(x == 0) {
        img_callback(msg1);
    }

    Mat a,b,c;
    frame.copyTo(detected_edges, detected_edges);
    cvtColor(yellow_found,yellow_found,COLOR_GRAY2BGR);
    cvtColor(white_found,white_found,COLOR_GRAY2BGR);
    cvtColor(combined_edges,combined_edges,COLOR_GRAY2BGR);

    hconcat(detected_edges,yellow_found, a);
    hconcat(a, white_found, b);
    hconcat(b, combined_edges, c);

    cv::resizeWindow(window_name, 1800, 900);
    imshow(window_name, c);
    waitKey(100);
}


void img_callback(const sensor_msgs::ImageConstPtr& msg) {
    //convert msg to mat
    msg1 = msg;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    frame = cv_ptr->image;

//    Canny
    cv::Mat frame_cut = cutEnvironment(frame);
    cv::Mat frame_gray, blur_gray, bgr_detected_edges;
    cv::cvtColor(frame_cut, frame_gray, cv::COLOR_BGR2GRAY );
    cv::blur(frame_gray, blur_gray, cv::Size(2,2));
    cv::Canny(blur_gray, detected_edges, min_line_threshold, max_line_threshold*3, 3);
    detected_edges = ignoreEdge(detected_edges);
    cv::dilate(detected_edges, detected_edges, kernel(2,1));

    frame_cut.copyTo(bgr_detected_edges, detected_edges);

    cv::Mat hsv_frame, yellow_combine, white_combine;
    cv::cvtColor(frame_cut, hsv_frame, cv::COLOR_BGR2HSV);

    if (color_choice == 0) {
        if (prev_choice != color_choice) {
            setTrackbarPos("Low H", window_name, ylow_H); setTrackbarPos("High H", window_name, yhigh_H);
            setTrackbarPos("Low S", window_name, ylow_S); setTrackbarPos("Low V", window_name, ylow_V);
            prev_choice = color_choice;
        }
        ylow_H = low_H; yhigh_H = high_H; ylow_S = low_S; ylow_V = low_V;
    } else {
        if (prev_choice != color_choice) {
            setTrackbarPos("Low H", window_name, wlow_H); setTrackbarPos("High H", window_name, whigh_H);
            setTrackbarPos("Low S", window_name, wlow_S); setTrackbarPos("Low V", window_name, wlow_V);
            prev_choice = color_choice;
        }
        wlow_H = low_H; whigh_H = high_H; wlow_S = low_S; wlow_V = low_V;
    }

    cv::inRange(hsv_frame, cv::Scalar(ylow_S, ylow_S, ylow_V), cv::Scalar(yhigh_H, 255, 255), yellow_found);
    cv::inRange(hsv_frame, cv::Scalar(wlow_H, wlow_S, wlow_V), cv::Scalar(whigh_H, 255, 255), white_found);

    cv::Mat yellow_edges = findColorLine(detected_edges.clone(), yellow_found, "yellow");
    cv::Mat white_edges = findColorLine(detected_edges.clone(), white_found, "white");

    Mat color_found;
    bitwise_or(yellow_found, white_found, color_found);
    bitwise_or(yellow_edges, white_edges, combined_edges);
//    cv::erode(combined_edges, combined_edges, kernel(2,1));
    cv::morphologyEx(combined_edges, combined_edges, cv::MORPH_CLOSE,kernel(7,7));
//    cv::erode(combined_edges, combined_edges, kernel(3,1));


//	publish Mask overlapped image
    sensor_msgs::Image outmsg;
    cv_ptr->image = detected_edges;
    cv_ptr->encoding = "mono8";
    cv_ptr->toImageMsg(outmsg);
    pub.publish(outmsg);

    //Debugging just colored Mask
    sensor_msgs::Image outmsg_mask;
    cv_ptr->image = yellow_found;
    cv_ptr->encoding = "mono8";
    cv_ptr->toImageMsg(outmsg_mask);
    debug_pub.publish(outmsg_mask);

    //Debugging just colored Mask
    sensor_msgs::Image outmsg_mask2;
    cv_ptr->image = combined_edges;
    cv_ptr->encoding = "mono8";
    cv_ptr->toImageMsg(outmsg_mask2);
    debug_pub2.publish(outmsg_mask2);

    //Debugging just colored Mask
    sensor_msgs::Image outmsg_mask3;
    cv_ptr->image = white_found;
    cv_ptr->encoding = "mono8";
    cv_ptr->toImageMsg(outmsg_mask3);
    debug_pub3.publish(outmsg_mask3);

    sensor_msgs::Image outmsg_mask4;
    cv_ptr->image = frame_cut;
    cv_ptr->encoding = "bgr8";
    cv_ptr->toImageMsg(outmsg_mask4);
    debug_pub4.publish(outmsg_mask4);

    CannyThreshold(1,0);
}

cv::Mat kernel(int x, int y) {
    return cv::getStructuringElement(cv::MORPH_RECT,cv::Size(x,y));
}

cv::Mat cutEnvironment(cv::Mat img) {
    cv::rectangle(img,
                  cv::Point(0,0),
                  cv::Point(img.cols,img.rows / 3 + blockSky_height),
                  cv::Scalar(0,0,0),CV_FILLED);

    cv::rectangle(img,
                  cv::Point(0,img.rows),
                  cv::Point(img.cols,2*img.rows / 3 + blockWheels_height),
                  cv::Scalar(0,0,0),CV_FILLED);

    cv::rectangle(img,
                  cv::Point(img.cols/3,img.rows),
                  cv::Point(2*img.cols/3,2*img.rows / 3.0 + blockBumper_height),
                  cv::Scalar(0,0,0),CV_FILLED);

    return img;
}

cv::Mat ignoreEdge(cv::Mat img) {
    cv::rectangle(img,
                  cv::Point(0,0),
                  cv::Point(img.cols,img.rows / 3 + blockSky_height + 1),
                  cv::Scalar(0,0,0),CV_FILLED);
    cv::rectangle(img,
                  cv::Point(0,img.rows),
                  cv::Point(img.cols,2*img.rows / 3 + blockWheels_height-1),
                  cv::Scalar(0,0,0),CV_FILLED);
    cv::rectangle(img,
                  cv::Point(img.cols/3,img.rows),
                  cv::Point(2*img.cols/3+1, 2*img.rows / 3 + blockBumper_height-1),
                  cv::Scalar(0,0,0),CV_FILLED);
    return img;
}

cv::Mat fillColorLines(cv::Mat lines, cv::Mat color_found) {
    cv::Mat color_left, lines_found_inv;
    cv::Mat lines_found(lines.rows,lines.cols,CV_8UC1,cv::Scalar::all(0));
    std::vector<cv::Point> locations;

    cv::dilate(lines, lines, kernel(1,2));
    cv::findNonZero(color_found, locations);
    for (int i = 0; i < locations.size(); i++) {
        cv::Point seed = locations[i];
        floodFill(lines, seed, cv::Scalar(100));
        cv::inRange(lines, 1, 254, lines_found);
        cv::bitwise_not(lines_found, lines_found_inv);
        color_left = cv::Scalar::all(0);
        color_found.copyTo(color_left, lines_found_inv);
        cv::findNonZero(color_left, locations);
    }
    return lines_found;
}

cv::Mat findColorLine(cv::Mat detected_edges, cv::Mat color_found, std::string color) {
    cv::Mat color_combine, color_edges, color_foundBigger, color_combine_edge, n;

    cv::erode(color_found, color_found, kernel(3,1));
    cv::dilate(color_found, color_found, kernel(2,2));

    detected_edges.copyTo(color_combine, color_found);
    cv::erode(color_combine, color_combine, kernel(2,1));
    cv::dilate(color_combine, color_combine, kernel(3,3));
    cv::morphologyEx(color_combine, color_combine, cv::MORPH_CLOSE, kernel(10,5));

    cv::Mat color_combine_cutMIN = cutSmallCurvedLines(color_combine, 30, 250000);

    return color_combine_cutMIN;
}

cv::Mat cutSmallCurvedLines(cv::Mat color_edges, int size_min, int size_max, bool useSolidity, double ratio) {
    cv::Mat contours_color(color_edges.rows,color_edges.cols,CV_8UC1,cv::Scalar::all(0));
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    double solidity;

    cv::findContours(color_edges, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point>> hull(contours.size());
    for( int i = 0; i < contours.size(); i++ ) {
        double area = cv::contourArea(contours[i], false);
        if(useSolidity) {
            cv::convexHull(contours[i], hull[i], false);
            solidity = area / (double) cv::contourArea(hull[i], false);
        }
        if((size_min < area && area < size_max) || (useSolidity && solidity > ratio && area > size_min/3)) {
            cv::drawContours(contours_color, contours, i, cv::Scalar(255), CV_FILLED, 8, hierarchy);
        }
    }
    return contours_color;
}

cv::Mat findMaximumAboveLimit(cv::Mat color_edges, int size_min, int size_max) {
    cv::Mat contours_color(color_edges.rows,color_edges.cols,CV_8UC1,cv::Scalar::all(0));
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    int max_index = 0;
    double max_area = 0;
    cv::findContours(color_edges, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point>> hull(contours.size());
    for( int i = 0; i < contours.size(); i++ ) {
        double area = cv::contourArea(contours[i], false);
        if(size_min < area && max_area < area && area < size_max) {
            max_area = area;
            max_index = i;
        }
    }
    if(max_area != 0.0) {
        cv::drawContours(contours_color, contours, max_index, cv::Scalar(255), CV_FILLED, 8, hierarchy);
    }
    return contours_color;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "histogram_detector");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    nhp.param("min_line_threshold", min_line_threshold, 50);
    nhp.param("max_line_threshold", max_line_threshold, 0);

    nhp.param("blockSky_height", blockSky_height, 140);
    nhp.param("blockWheels_height", blockWheels_height, 70);
    nhp.param("blockBumper_height", blockBumper_height, 0);

    nhp.param("yellow_low_H", yellow_low_H, 20);
    nhp.param("yellow_high_H", yellow_high_H, 40);
    nhp.param("yellow_low_S", yellow_low_S, 20);
    nhp.param("yellow_low_V", yellow_low_V, 100);


    nhp.param("white_low_H", yellow_low_H, 90);
    nhp.param("white_high_H", white_high_H, 115);
    nhp.param("white_low_S", white_low_S, 0);
    nhp.param("white_low_V", white_low_V, 100);

    namedWindow( window_name, CV_WINDOW_NORMAL );
    createTrackbar( "Min Threshold:", window_name, &min_line_threshold, 400);
    createTrackbar( "Max Threshold:", window_name, &max_line_threshold, 100 );

    createTrackbar( "BlockSky:", window_name, &blockSky_height, 200 );
    createTrackbar( "BlockCar:", window_name, &blockWheels_height, 200 );
//    createTrackbar( "BlockBumper:", window_name, &blockBumper_height, 200, CannyThreshold );

    createTrackbar("Yellow | White", window_name, &color_choice, 1);
    createTrackbar("Low H", window_name, &low_H, 180);
    createTrackbar("High H", window_name, &high_H, 180);
    createTrackbar("Low S", window_name, &low_S, 255);
    createTrackbar("Low V", window_name, &low_V, 255);

    pub = nh.advertise<sensor_msgs::Image>("/canny_detected_edges", 1); //test publish of image
    debug_pub = nh.advertise<sensor_msgs::Image>("/canny_yellow_found", 1); //test publish of image
    debug_pub2 = nh.advertise<sensor_msgs::Image>("/canny_color_edges", 1); //test publish of image
    debug_pub3 = nh.advertise<sensor_msgs::Image>("/canny_white_found", 1); //test publish of image
    debug_pub4 = nh.advertise<sensor_msgs::Image>("/canny_frameCut", 1); //test publish of image
    pub = nh.advertise<sensor_msgs::Image>("/canny_detected_edges", 1); //test publish of image
//    auto img_sub = nh.subscribe("camera/image_color_rect_flipped", 1, img_callback);   //Newer Videos
	auto img_sub = nh.subscribe("/camera/image_color_rect", 1, img_callback);               //Older Videos

    ros::spin();
    return 0;

}