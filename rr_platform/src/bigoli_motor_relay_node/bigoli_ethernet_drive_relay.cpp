#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <rr_platform/chassis_state.h>
#include <EthernetSocket.h>
#include <iostream>

using namespace std;

struct PIDConst {
  float p;
  float i;
  float d;
};

struct PIDConst accelDrivePID, decelDrivePID, steeringPID;

double speed = 0.0;
double steeringAngle = 0.0;

double maxAngleMsg;
const double maxOutput = 1.0;


void speedCallback(const rr_platform::speed::ConstPtr &msg) {
    speed = msg->speed;
}

void steerCallback(const rr_platform::steering::ConstPtr &msg) {
    steeringAngle = msg->angle / maxAngleMsg * maxOutput; // Taken from old relay
}

string buildPIDMessage(const PIDConst& pid) {
  //combines PID into useful message
  string command = to_string(pid.p) + " " + to_string(pid.i) + " " + to_string(pid.d);

  return command;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "bigoli_ethernet_drive_relay");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    //Setup speed info
    string speedTopic = nhp.param(string("speedTopic"), string("/speed"));
    auto speedSub = nh.subscribe(speedTopic, 1, speedCallback);

    //accel PID
    accelDrivePID.p = nhp.param(string("accel_pid_p"), 0.0);
    accelDrivePID.i = nhp.param(string("accel_pid_i"), 0.0);
    accelDrivePID.d = nhp.param(string("accel_pid_d"), 0.0);
    //decel PID
    decelDrivePID.p = nhp.param(string("decel_pid_p"), 0.0);
    decelDrivePID.i = nhp.param(string("decel_pid_i"), 0.0);
    decelDrivePID.d = nhp.param(string("decel_pid_d"), 0.0);

    //Steering PID
    steeringPID.p = nhp.param(string("steering_pid_p"), 0.0);
    steeringPID.i = nhp.param(string("steering_pid_i"), 0.0);
    steeringPID.d = nhp.param(string("steering_pid_d"), 0.0);

    //Setup steering info
    string steerTopic = nhp.param(string("steeringTopic"), string("/steering"));
    auto steerSub = nh.subscribe(steerTopic, 1, steerCallback);

    maxAngleMsg = nhp.param(string("max_angle_msg_in"), 1.0);

    //IP address and port
    string ip_addr = nhp.param(string("ip_address"), string("192.168.2.2"));
    int port = nhp.param(string("tcp_port"), 7);

    ROS_INFO_STREAM("Trying to connect to TCP Host at " + ip_addr + " port: " + to_string(port));
    EthernetSocket sock(ip_addr, port);

    //CONNECTION NOW OPEN, READY TO JAM
    ROS_INFO_STREAM("Connected to TCP Host");

    //send initialization message (PIDs) in order: accel, deccel, steering
    //combines PID into useful message; # means init message; EX: # p i d p i d p i d
    string pidMessage = "#" + buildPIDMessage(accelDrivePID) +
          " " + buildPIDMessage(decelDrivePID) +
          " " + buildPIDMessage(steeringPID);

    sock.sendMessage(pidMessage);
    ROS_INFO_STREAM("Sent PID: " + pidMessage);
    string response = sock.readMessage(); //Should say PID Received //#TODO: have a check for correct responses?

    ros::Rate rate(50); //#TODO set this value to a good rate time (50hz seems good)/ do we need this?
    while(ros::ok()) {
        ros::spinOnce();

        if (response.at(0) == '@') {
          /*sometimes readMessage returns blank, unknown cause #TODO
           *This if keep us from sending unused data that backs up and causes delays
           *Using the @ symbols also gives us handy status check info too
           */
          ROS_INFO_STREAM("RESPONSE: " + response);

          //write data to MBED
          //Send Motor Command and Steering Command. EX: $speed steeringAngle
          string command = "$" + to_string(speed) + " " + to_string(steeringAngle); //#TODO: the firmware may need to be clearing the buffer properly
          sock.sendMessage(command);
          ROS_INFO_STREAM("SENT:" + command);
          response = ""; //clear for next check
        }

        //read data from MBED
        response = sock.readMessage(); //#TODO: make this useful data

        rate.sleep();
    }

    return 0;
}
