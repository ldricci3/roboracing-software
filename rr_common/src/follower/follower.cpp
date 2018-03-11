#include "follower.h"

using namespace std;

void mapCallback(const sensor_msgs::PointCloud2ConstPtr& map) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*map, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    if (cloud->empty()) {
        ROS_WARN("environment map pointcloud is empty");
        rr_platform::speedPtr speedMSG(new rr_platform::speed);
        rr_platform::steeringPtr steerMSG(new rr_platform::steering);
        speedMSG->speed = MAX_SPEED / 4; //proceed with caution; E-Kill if necessary
        steerMSG->angle = 0;
        steerMSG->header.stamp = ros::Time::now();
        speedMSG->header.stamp = ros::Time::now();
        speed_pub.publish(speedMSG);
        steer_pub.publish(steerMSG);
        return;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
}

void parseFloatArrayStr(string &arrayAsString, vector<float> &floats) {
    char s[arrayAsString.size()];
    strcpy(s, arrayAsString.c_str());
    char* token = strtok(s, " ,");
    while(token != NULL) {
        floats.push_back(stof(string(token)));
        token = strtok(NULL, " ,");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "follower");
    string obstacleCloudTopic;

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    nhp.param("MIN_X", MIN_X, -0.2f);
    nhp.param("MAX_X", MAX_X, 0.2f);
    nhp.param("MIN_Y", MIN_Y, 0.1f);
    nhp.param("MAX_Y", MAX_Y, 0.5f);
    nhp.param("MAX_Z", MAX_Z, 1.0f);
    nhp.param("GOAL_Z", GOAL_Z, 0.6f);
    nhp.param("Z_SCALE", Z_SCALE, 1.0f);
    nhp.param("X_SCALE", X_SCALE, 5.0f);
    nhp.param("MAX_SPEED", MAX_SPEED, 0.5f);
    nhp.param("INPUT_CLOUD_TOPIC", obstacleCloudTopic, string("/map"));

    auto map_sub = nh.subscribe(obstacleCloudTopic, 1, mapCallback);
    speed_pub = nh.advertise<rr_platform::speed>("plan/speed", 1);
    steer_pub = nh.advertise<rr_platform::steering>("plan/steering", 1);

    ROS_INFO("follower initialized");

    ros::spin();
    return 0;
}
