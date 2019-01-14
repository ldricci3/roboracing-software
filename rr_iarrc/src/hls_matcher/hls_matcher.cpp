#include <ros/ros.h>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

ros::Publisher pub;

int hsv_select = 0; //0 = hue, 1 = sat, 2 = value

int threshold_min;
int threshold_max;

cv::Mat output;


void img_callback(const sensor_msgs::ImageConstPtr& msg) {
	//converty msg to mat
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	cv::Mat frame = cv_ptr->image;

	//convert from rgb to hls
	cv::Mat frame_hls;
	cvtColor(frame, frame_hls, cv::COLOR_BGR2HLS);
	std::vector<cv::Mat> frame_hls_planes;
	cv::split(frame_hls, frame_hls_planes);

	//blurring to assist in removing line from road mask...eh
	//cv::GaussianBlur(frame_hsv_planes[2], frame_hsv_planes[2], cv::Size(9,9), 2.0);

  //hls threshold
  cv::inRange(frame_hls_planes[1], cv::Scalar(threshold_min), cv::Scalar(threshold_max), output);
	auto kernel_small = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
	cv::morphologyEx(output, output, cv::MORPH_OPEN,kernel_small);

  /*
  //threshold
	cv::threshold(frame_hsv_planes[2], output, threshold, 255, cv::THRESH_BINARY);
	auto kernel_small = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
	cv::morphologyEx(output, output, cv::MORPH_OPEN,kernel_small);
	//#TODO: will need some very small morphology openning to remove stray pixels, and proper body removal
  */

	//remove the body kinda
	cv::rectangle(output,
			cv::Point(390,650),
			cv::Point(400 + 410,output.rows),
			cv::Scalar(0,0,0),CV_FILLED);

	//remove sky kinda
	cv::rectangle(output,
			cv::Point(0,0),
			cv::Point(output.cols,output.rows / 3 + 100),
			cv::Scalar(0,0,0),CV_FILLED);



	//publish mask image
  sensor_msgs::Image outmsg;
  cv_ptr->image = output;
  cv_ptr->encoding = "mono8";
  cv_ptr->toImageMsg(outmsg);

  pub.publish(outmsg);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "hls_matcher");

	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");
	nhp.param("threshold_min", threshold_min, 10);
  nhp.param("threshold_max", threshold_max, 100);
	nhp.param("hsv_select", hsv_select, 0);


  pub = nh.advertise<sensor_msgs::Image>("/hls_matcher", 1); //publish lines as binary image
	auto img_sub = nh.subscribe("/camera/image_color_rect_flipped", 1, img_callback);

	ros::spin();
	return 0;

}
