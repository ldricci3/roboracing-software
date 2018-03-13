#include "follower.h"

using namespace std;

void mapCallback(const sensor_msgs::PointCloud2ConstPtr& map) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*map, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    rr_platform::speedPtr speedMSG(new rr_platform::speed);
    rr_platform::steeringPtr steerMSG(new rr_platform::steering);
    if (cloud->empty()) {
        ROS_WARN("environment map pointcloud is empty");
        speedMSG->speed = 0;
        steerMSG->angle = 0;
        steerMSG->header.stamp = ros::Time::now();
        speedMSG->header.stamp = ros::Time::now();
        speed_pub.publish(speedMSG);
        steer_pub.publish(steerMSG);
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (MIN_X, MAX_X);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (GROUND_THRESHOLD, MAX_Z);
    pass.filter (*cloud_filtered); //creates filtered cloud excluding objects too far left or right, as well as those on the ground

    float avgY = 0.0f;

    for (pcl::PointXYZ point : cloud_filtered->points) {
        avgY += point.y;
    }
    avgY = avgY / cloud_filtered->points.size(); //computes the average distance away points are from the sensor directly forward

    steerMSG->header.stamp = ros::Time::now();
    speedMSG->header.stamp = ros::Time::now();
    if (avgY > 0 && avgY <= GOAL_Y)
        speedMSG->speed = -(FOLLOWER_SPEED); //when object is too close, move backward
    else if (avgY < MAX_Y && avgY >= GOAL_Y)
        speedMSG->speed = FOLLOWER_SPEED; //when object is too far, move forward
    else
        speedMSG->speed = 0; //when the object exceeds MAX_Y or another anomali occurs

    steerMSG->angle = 0;
    speed_pub.publish(speedMSG);
    steer_pub.publish(steerMSG);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "follower");
    string cloudTopic;

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    nhp.param("MIN_X", MIN_X, -0.2f);
    nhp.param("MAX_X", MAX_X, 0.2f);
    nhp.param("GOAL_Y", GOAL_Y, 0.5f);
    nhp.param("MAX_Y", MAX_Y, 3.0f);
    nhp.param("MAX_Z", MAX_Z, 5.0f);
    nhp.param("GROUND_THRESHOLD", GROUND_THRESHOLD, 0.1f);
    nhp.param("FOLLOWER_SPEED", FOLLOWER_SPEED, 1.0f);
    nhp.param("INPUT_CLOUD_TOPIC", cloudTopic, string("/map"));

    auto map_sub = nh.subscribe(cloudTopic, 1, mapCallback);
    speed_pub = nh.advertise<rr_platform::speed>("plan/speed", 1);
    steer_pub = nh.advertise<rr_platform::steering>("plan/steering", 1);

    ROS_INFO("follower initialized");

    ros::spin();
    return 0;
}
