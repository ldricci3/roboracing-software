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
using namespace std;

cv_bridge::CvImagePtr cv_ptr;
ros::Publisher pub, pub1, pub2;

cv::Mat kernel(int, int);
cv::Mat fillColorLines(cv::Mat, cv::Mat);
void cutEnvironment(cv::Mat);
cv::Mat cutSmall(cv::Mat, int);
void publishMessage(ros::Publisher, Mat, std::string);
Mat overlayBinaryGreen(Mat, Mat);
cv::Mat  getCenter(cv::Mat, double);
cv::Mat  splitBodies(cv::Mat, Mat);

int blockSky_height, blockWheels_height, blockBumper_height;
int low_H = 5, high_H = 25, low_S = 140, low_V = 140;

void img_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    Mat frame = cv_ptr->image;

    cv::Mat hsv_frame, orange_found;
    cv::Mat frame_gray, detected_edges, img;
    Mat frame_cut = frame;
    cutEnvironment(frame_cut);

    cv::cvtColor(frame_cut, hsv_frame, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_frame, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, 255, 255), orange_found);

    GaussianBlur( frame_cut, frame_cut, Size(5,5), 0, 0, BORDER_DEFAULT );
    cv::cvtColor(frame_cut, frame_gray, cv::COLOR_BGR2GRAY );
    bitwise_and(frame_gray, orange_found, frame_gray);
    cv::Canny(frame_gray, detected_edges, 35, 35*3, 3);
    dilate(detected_edges, detected_edges,kernel(2,2));
    floodFill(detected_edges, Point(0,0), cv::Scalar(255));
    bitwise_not(detected_edges, detected_edges);
    morphologyEx(detected_edges, detected_edges, cv::MORPH_OPEN, kernel(3,3));

    Mat sure_back, dist_transform, unknown, markers, stats, marker_centroid, labels;
    cv::morphologyEx(orange_found, orange_found, cv::MORPH_OPEN, kernel(2,2));
//    cv::morphologyEx(orange_found, orange_found, cv::MORPH_CLOSE, kernel(2,2));

    dilate(orange_found, sure_back, kernel(3,3));

//    Mat split = splitBodies(frame, orange_found);
    Mat sure_front = getCenter(detected_edges, .7);


    sure_front.convertTo(sure_front, CV_8UC1);

    subtract(sure_back, sure_front, unknown);

    connectedComponents(sure_front, markers);
    markers += 1;
    markers.setTo(0, unknown == 255);

    watershed(frame, markers);
    frame.setTo(Scalar(0, 255), markers == -1);

    Mat lines = Mat::zeros(frame.size(), CV_8UC1);
    lines.setTo(Scalar(255), markers == -1);
    rectangle(lines, Point(0,0), Point(frame.cols, frame.rows), Scalar(0), 2);

    dilate(lines, lines, kernel(2,1));
    floodFill(lines, Point(0,0), cv::Scalar(255));
    Mat body;
    bitwise_not(lines, body);

    vector<vector<Point> > contours;
    findContours(body, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    vector<vector<Point> > approx(contours.size());
    vector<Rect> rect( contours.size() );
    vector<int> heights;
    Mat drawing = Mat::zeros(frame.size(), CV_8UC3);

    double fy = 111; //586.508911;
    double real_height = .45; //cm

    for( int i = 0; i< contours.size(); i++ ) {

        rect[i] = boundingRect(contours[i]);
        if (rect[i].height < 20 || rect[i].width < 10) {
            continue;
        }

        Moments m = moments(contours[i], true);
        Point center = Point(m.m10 / m.m00, m.m01 / m.m00);

        double distance = (real_height * fy) / rect[i].height;
        double px_horz_dist = abs(frame.cols/2 - (rect[i].tl().x + rect[i].width/2));
        double horz_dist = distance * px_horz_dist / fy;


        cerr << "Box Height, Mid Dist: (" << rect[i].height << ", " << px_horz_dist << ")";
        cerr << "     X, Y Dist: (" << distance << ", " << horz_dist << ") " << endl;

        Scalar color =  Scalar(0,255);
        drawContours( frame, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        rectangle(frame, rect[i].tl(), rect[i].br(), color, 3, 8, 0);
        circle(frame, center, 5,  Scalar(0,0,255), CV_FILLED, 8, 0);
    }



//	publish Images
    publishMessage(pub, frame, "bgr8");
    publishMessage(pub1, orange_found, "mono8");
//    publishMessage(pub2, orange_found, "mono8");
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "coneDetection");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string subscription_node;
//    nhp.param("perfect_lines_min_cut", perfect_lines_min_cut, 200);
//    nhp.param("Laplacian_threshold", Laplacian_threshold, 2);

    nhp.param("blockSky_height", blockSky_height, 220);
    nhp.param("blockWheels_height", blockWheels_height, 200);
    nhp.param("blockBumper_height", blockBumper_height, 200);

    nhp.param("subscription_node", subscription_node, std::string("/camera/image_color_rect"));

    pub = nh.advertise<sensor_msgs::Image>("/overlay_cones", 1); //test publish of image
    pub1 = nh.advertise<sensor_msgs::Image>("/cone_line_debug", 1);
    pub2 = nh.advertise<sensor_msgs::Image>("/cone_orange_found", 1);
    auto img_real = nh.subscribe(subscription_node, 1, img_callback);

    ros::spin();
    return 0;
}

cv::Mat getCenter(cv::Mat img, double thres_value) {
    Mat dist_transform, thres, dist_mask, mask;
    Mat sure_front = Mat::zeros(img.size(), CV_8UC1);
    vector<vector<Point>> contours1;
    double min, max;

    distanceTransform(img, dist_transform, CV_DIST_L2, 3);
    findContours(img, contours1, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    for( int i = 0; i < contours1.size(); i++ ) {
        mask = Mat::zeros(img.size(), CV_8UC1);
        dist_mask = Scalar::all(0);

        drawContours(mask, contours1, i, 255, CV_FILLED);

        dist_transform.copyTo(dist_mask, mask);

        cv::minMaxLoc(dist_mask, &min, &max);

        threshold(dist_mask, thres, max * thres_value, 255, CV_THRESH_BINARY);
        thres.convertTo(thres, CV_8UC1);
        bitwise_or(thres, sure_front, sure_front);
    }

    return sure_front;
}

cv::Mat splitBodies(cv::Mat frame, cv::Mat cones_found) {
    Mat thres, frame_gray, mask, edges, background, forground, body;

    vector<vector<Point> > contours;
    findContours(cones_found, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    for( int i = 0; i< contours.size(); i++ ) {
        drawContours(frame, contours, i, Scalar(255), 1, 8, vector<Vec4i>(), 0, Point());
    }

    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    adaptiveThreshold(frame_gray, thres, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 5, -1);

    cones_found.copyTo(mask, thres);
    cv::morphologyEx(mask, edges, cv::MORPH_CLOSE, kernel(2,2));
    background = edges.clone();
    floodFill(background, Point(0,0), cv::Scalar(255));
    bitwise_not(background, forground);

    floodFill(background, Point(0,0), cv::Scalar(0));
    bitwise_xor(forground, background, body);


    return body;
}



cv::Mat kernel(int x, int y) {
    return cv::getStructuringElement(cv::MORPH_RECT,cv::Size(x,y));
}

cv::Mat fillColorLines(cv::Mat lines, cv::Mat color_found) {
    cv::Mat color_left, lines_found(lines.rows,lines.cols,CV_8UC1,cv::Scalar::all(0));
    Mat lines_remaining = lines.clone();
    std::vector<cv::Point> locations;

    lines.copyTo(color_left, color_found);
    cv::findNonZero(color_left, locations);
    while(!locations.empty()) {
        floodFill(lines_remaining, locations[0], cv::Scalar(0));
        bitwise_xor(lines, lines_remaining, lines_found);
        bitwise_and(lines_remaining, color_left, color_left);
        cv::findNonZero(color_left, locations);
    }
    return lines_found;
}

void cutEnvironment(cv::Mat img) {
    cv::rectangle(img,
                  cv::Point(0,0),
                  cv::Point(img.cols,img.rows / 3 + blockSky_height),
                  cv::Scalar(0,0,0),CV_FILLED);

    cv::rectangle(img,
                  cv::Point(0,img.rows),
                  cv::Point(img.cols,2 * img.rows / 3 + blockWheels_height),
                  cv::Scalar(0,0,0),CV_FILLED);

    cv::rectangle(img,
                  cv::Point(img.cols/3,img.rows),
                  cv::Point(2 * img.cols / 3, 2 * img.rows / 3 + blockBumper_height),
                  cv::Scalar(0,0,0),CV_FILLED);
}

cv::Mat cutSmall(cv::Mat color_edges, int size_min) {
    cv::Mat contours_color(color_edges.rows,color_edges.cols,CV_8UC1,cv::Scalar::all(0));
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(color_edges, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    for( int i = 0; i < contours.size(); i++ ) {
        if (size_min < cv::arcLength(contours[i], false) ) {
            cv::drawContours(contours_color, contours, i, cv::Scalar(255), CV_FILLED, 8, hierarchy);
        }
    }
    return contours_color;
}

void publishMessage(ros::Publisher pub, Mat img, std::string img_type) {
    if (pub.getNumSubscribers() > 0) {
        sensor_msgs::Image outmsg;
        cv_ptr->image = img;
        cv_ptr->encoding = img_type;
        cv_ptr->toImageMsg(outmsg);
        pub.publish(outmsg);
    }
}

Mat overlayBinaryGreen(Mat frame, Mat binary) {
    Mat green(frame.rows,frame.cols,CV_8UC3,cv::Scalar::all(3));
    Mat green_edges, weight;

    green = cv::Scalar(0,255,0);
    green.copyTo(green_edges, binary);
    addWeighted(frame, .7, green_edges, 1, 0.0, weight);

    return weight;
}
