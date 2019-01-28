#include <ros/ros.h>
#include <ros/publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

ros::Publisher pub;

bool remove_sun;
string image_topic;

int threshold_lightness_min;
int threshold_lightness_max;
int threshold_hue_min;
int threshold_hue_max;


void img_callback(const sensor_msgs::ImageConstPtr& msg) {
	//converty msg to mat
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	cv::Mat frame = cv_ptr->image;

	//convert from rgb to hls
	cv::Mat frame_hls;
	cvtColor(frame, frame_hls, cv::COLOR_BGR2HLS);
	std::vector<cv::Mat> frame_hls_planes;
	cv::split(frame_hls, frame_hls_planes);

	cv::Mat output;

	//blurring to assist in removing line from road mask...eh
	//cv::GaussianBlur(frame_hls_planes[1], frame_hls_planes[1], cv::Size(9,9), 2.0);

  //Line Detector: hls lightness threshold
	cv::Mat thresh_lightness;
  cv::inRange(frame_hls_planes[1], cv::Scalar(threshold_lightness_min), cv::Scalar(threshold_lightness_max), thresh_lightness);

	if (remove_sun) {
		//Anti-Sun Detector: get rid of red/yellow/orange "SUN" hues
		cv::Mat thresh_hue;
		cv::inRange(frame_hls_planes[0], cv::Scalar(threshold_hue_min), cv::Scalar(threshold_hue_max), thresh_hue);

		//mix the line detector with the anti-sun detector: line AND NOT sun tint
		cv::bitwise_not(thresh_hue, thresh_hue);
		cv::bitwise_and(thresh_lightness, thresh_hue, output);
	} else {
		output = thresh_lightness;
	}

	//#TODO: Maybe do contours and then check if any of them are too round and ignore those;

	//Morphology to connect lines and remove random noise #TODO: play with these numbers maybe
	auto kernel_large = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(21,21));
	cv::morphologyEx(output, output, cv::MORPH_CLOSE, kernel_large);
	auto kernel_small = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
	cv::morphologyEx(output, output, cv::MORPH_OPEN, kernel_small);

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
	nhp.param("threshold_lightness_min", threshold_lightness_min, 100);
  nhp.param("threshold_lightness_max", threshold_lightness_max, 250);
	nhp.param("threshold_hue_min", threshold_hue_min, 0);
	nhp.param("threshold_hue_max", threshold_hue_max, 80);
	nhp.param("remove_sun", remove_sun, true);
	nhp.param("image_topic", image_topic, "/camera/image_color_rect");


  pub = nh.advertise<sensor_msgs::Image>("/hls_matcher", 1); //publish lines as binary image
	auto img_sub = nh.subscribe(image_topic, 1, img_callback);

	ros::spin();
	return 0;

}
