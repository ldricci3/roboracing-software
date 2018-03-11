#ifndef RR_COMMON_FOLLOWER_H
#define RR_COMMON_FOLLOWER_H

#include <ros/ros.h>
#include <cmath>
#include <cstring>
#include <string>
#include <random>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "flann/flann.hpp"

float MIN_Y;
float MAX_Y;
float MIN_X;
float MAX_X;
float MAX_Z;
float GOAL_Z;
float Z_SCALE;
float X_SCALE;

float MAX_SPEED;

ros::Publisher speed_pub, steer_pub;


#endif //RR_COMMON_FOLLOWER_H
