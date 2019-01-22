#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rr_platform/calibrate_image.h>
#include <rr_platform/camera_pose.h>
#include <cmath>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace cv;
using namespace ros;

const double PX_PER_METER = 20.0;
const double CAMERA_DIST_MAX = 7.0; //distance to look ahead of the car
const double CAMERA_DIST_MIN = 1.0; //avoid the front bumper

double camera_fov_horizontal = -1;
double camera_fov_vertical = -1;
double cam_mount_angle; //angle of camera from horizontal
double cam_height; //camera height from ground in meters

Size mapSize; //pixels = cm
Size imageSize;
Mat transform_matrix;

map<string, Publisher> transform_pubs;

void loadCameraPoseFromTf() {
    ROS_INFO("image_transform is loading camera pose from tf...");
    tf::TransformListener listener;

    tf::Quaternion qTFCamBase, qTFBaseChassis;
    qTFCamBase.setRPY(0,0,0);
    qTFBaseChassis.setRPY(0,0,0);
    geometry_msgs::Quaternion qMsgCamBase, qMsgBaseChassis;
    tf::quaternionTFToMsg(qTFCamBase, qMsgCamBase);
    tf::quaternionTFToMsg(qTFBaseChassis, qMsgBaseChassis);

    geometry_msgs::PoseStamped psSrcCam, psDstBase, psSrcBase;
    psSrcCam.header.frame_id = "camera";
    psSrcCam.pose.orientation = qMsgCamBase;
    psSrcBase.header.frame_id = "base_footprint";
    psSrcBase.pose.orientation = qMsgBaseChassis;

    bool success = false;
    while(!success) {
        try {
            listener.waitForTransform("base_footprint", "camera", ros::Time(0), ros::Duration(5.0));
            listener.transformPose("base_footprint", psSrcCam, psDstBase);
            listener.waitForTransform("chassis", "base_footprint", ros::Time(0), ros::Duration(5.0));
            success = true;
        } catch(tf2::LookupException& e) {
            ROS_ERROR("tf LookupException: %s", e.what());
        }
    }

    tf::quaternionMsgToTF(psDstBase.pose.orientation, qTFCamBase);

    double roll, pitch, yaw;
    tf::Matrix3x3(qTFCamBase).getRPY(roll, pitch, yaw);

    cam_mount_angle = pitch;
    cam_height = psDstBase.pose.position.z;
}

void fovCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
//    ROS_INFO("called fovCallback");
    double fx = msg->P[0]; //horizontal focal length of rectified image, in px
    double fy = msg->P[5]; //vertical focal length
    imageSize = Size(msg->width, msg->height);
    camera_fov_horizontal = 2 * atan2(imageSize.width, 2*fx);
    camera_fov_vertical = 2 * atan2(imageSize.height, 2*fy);
}

void loadCameraFOV(NodeHandle& nh) {
    auto infoSub = nh.subscribe("/camera/camera_info", 1, fovCallback);

    Time t_start = Time::now();
    while(camera_fov_horizontal <= 0 || camera_fov_vertical <= 0) {
        if((Time::now() - t_start).toSec() > 10) {
            ROS_INFO("[Image_Transform] Warning: setting camera FOV from camera_info timed out");

            // FLIR Blackfly backup FOV info
            camera_fov_horizontal = 1.6955;
            camera_fov_vertical = 1.3758;
            imageSize.height = 964;
            imageSize.width = 1280;
        }

        spinOnce();
        Duration(0.05).sleep();
    }
    ROS_INFO("Using horizontal FOV %f and vertical FOV %f", camera_fov_horizontal, camera_fov_vertical);
}


/*
 * Start with a horizontal line on the groud at dmin meters horizonally in front of the 
 * camera. It fills half the camera's FOV, from the center to the right edge. Then back
 * up the car so that the line is dmax meters away horizontally from the camera. The
 * apparent length of this hypothetical line (in pixels) is the output of this function.
 * See https://www.desmos.com/calculator/jsofcq1bi5
 * See https://drive.google.com/file/d/0Bw7-7Y3CUDw1Z0ZqdmdRZ3dUTE0/view?usp=sharing
 */
double pxFromDist_X(double dmin, double dmax) {
    // min_hyp, max_hyp, and theta1 are just temporary variables
    double min_hyp = sqrt(dmin*dmin + cam_height*cam_height);
    double max_hyp = sqrt(dmax*dmax + cam_height*cam_height);
    double theta1 = atan((min_hyp/max_hyp)*tan(camera_fov_horizontal/2));
    return imageSize.width * (theta1 / camera_fov_horizontal);
}

//calculate the y coord of the input image from the specified distance
// see https://www.desmos.com/calculator/pwjwlnnx77
// see https://drive.google.com/file/d/0Bw7-7Y3CUDw1Z0ZqdmdRZ3dUTE0/view?usp=sharing
double pxFromDist_Y(double dist) {
    double tmp = atan(cam_height/dist) - cam_mount_angle + camera_fov_vertical/2;
    return imageSize.height * tmp / (camera_fov_vertical);
}

void setTransformFromGeometry() {
    //set width and height of the rectangle in front of the robot with 1cm = 1px
    // the actual output image will show more than this rectangle
    float rectangle_w = static_cast<float>(sqrt(pow(CAMERA_DIST_MIN, 2) + pow(cam_height, 2))
                                           * tan(camera_fov_horizontal/2) * PX_PER_METER * 2);
    float rectangle_h = static_cast<float>((CAMERA_DIST_MAX - CAMERA_DIST_MIN)
                                            * PX_PER_METER);

    //find coordinates for corners above rectangle in input image
    float x_top_spread = static_cast<float>(pxFromDist_X(CAMERA_DIST_MIN,
                                                         CAMERA_DIST_MAX));
    float y_bottom = static_cast<float>(pxFromDist_Y(CAMERA_DIST_MIN));
    float y_top    = static_cast<float>(pxFromDist_Y(CAMERA_DIST_MAX));

    //set the ouput image size to include the whole transformed image,
    // not just the target rectangle
    mapSize.width = static_cast<int>(rectangle_w * (imageSize.width/x_top_spread) / 2);
    mapSize.height = static_cast<int>(rectangle_h);

    Point2f src[4] = {
        Point2f(imageSize.width/2.f - x_top_spread, y_top), //top left
        Point2f(imageSize.width/2.f + x_top_spread, y_top), //top right
        Point2f(0, y_bottom),                             //bottom left
        Point2f(imageSize.width, y_bottom)                //bottom right
    };

    Point2f dst[4] = {
        Point2f(mapSize.width/2.f - rectangle_w/2.f, 0),              //top left
        Point2f(mapSize.width/2.f + rectangle_w/2.f, 0),              //top right
        Point2f(mapSize.width/2.f - rectangle_w/2.f, mapSize.height), //bottom left
        Point2f(mapSize.width/2.f + rectangle_w/2.f, mapSize.height)  //bottom right
    };

    transform_matrix = getPerspectiveTransform(src, dst);
}

void TransformImage(const sensor_msgs::ImageConstPtr& msg, string& topic) {
    //if no one is listening or the transform is undefined, give up
    if(transform_pubs[topic].getNumSubscribers() == 0 || transform_matrix.empty()) {
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    const Mat &inimage = cv_ptr->image;

    Mat outimage;
    warpPerspective(inimage, outimage, transform_matrix, mapSize);

    sensor_msgs::Image outmsg;
    cv_ptr->image = outimage;
    cv_ptr->toImageMsg(outmsg);
    transform_pubs[topic].publish(outmsg);
}


int main(int argc, char **argv) {
    init(argc, argv, "image_transform");
    NodeHandle nh;
    NodeHandle pnh("~");

    loadCameraFOV(nh); //spins ROS event loop for a bit
    loadCameraPoseFromTf();
    setTransformFromGeometry();
    ROS_INFO("Calculated perspective transform. Used height %f and angle %f", cam_height, cam_mount_angle);

    string topicsConcat;
    pnh.getParam("transform_topics", topicsConcat);
    vector<string> topics;
    boost::split(topics, topicsConcat, boost::is_any_of(" ,"));
    vector<Subscriber> transform_subs;
    ROS_INFO_STREAM("Found " << topics.size() << " topics in param.");
    for(const string& topic : topics) {
        if(topic.size() == 0) continue;
        transform_subs.push_back(nh.subscribe<sensor_msgs::Image>(topic, 1,
                                 boost::bind(TransformImage, _1, topic)));
        ROS_INFO_STREAM("Image_transform subscribed to " << topic);
        string newTopic(topic + "_transformed");
        ROS_INFO_STREAM("Creating new topic " << newTopic);
        transform_pubs[topic] = nh.advertise<sensor_msgs::Image>(newTopic, 1);
    }

    spin();

    return 0;
}
