#include <ros/ros.h>
#include <rr_platform/speed.h>
#include <rr_platform/steering.h>
#include <rr_platform/chassis_state.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <iostream>

/*@NOTE THIS CODE USES BOOST 1.58 as it is the version currently installed with ROS.
  If that changes, this code will need to be updated as such!
  So don't fear if it breaks, just fix the things by checking the links to examples given.
*/

using namespace std;
using boost::asio::ip::tcp;

struct PIDConst {
  float p;
  float i;
  float d;
};

struct PIDConst accelDrivePID, decelDrivePID, steeringPID;

double speed = 0.0;
double steeringAngle = 0.0;

double maxAngleMsg;
const double maxOutput = 1.0;  //#TODO: magic # to launch param

std::shared_ptr<tcp::socket> currentSocket;

void speedCallback(const rr_platform::speed::ConstPtr &msg) {
    speed = msg->speed;
}

void steerCallback(const rr_platform::steering::ConstPtr &msg) {
    steeringAngle = msg->angle / maxAngleMsg * maxOutput; //#TODO: should this calculation still done? Taken from old relay
}

string readMessage() {
  //read data from TCP connection
  boost::array<char, 128> buf;
  boost::system::error_code error;

  size_t len = currentSocket->read_some(boost::asio::buffer(buf), error);
  string reading(buf.begin(), buf.end()); //convert buffer into useable string

  if (error == boost::asio::error::eof) { //#TODO THIS ERROR GET OUT OF HERE MAY NEED TO CHANGE as recieving nothing = dead
    // Connection closed cleanly by peer
    ROS_INFO_STREAM("TCP Connection closed by peer: Disconnecting"); //#TODO: not sure what to do or just leave it orr.
  } else if (error) {
    ROS_ERROR_STREAM("TCP ERROR: Disconnecting");
    throw boost::system::system_error(error); // Some other error
  }

  return reading;
}

void sendMessage(string message) {
  boost::array<char, 128> buf;
  boost::system::error_code error;

  //write data to TCP connection
  boost::asio::write(*currentSocket, boost::asio::buffer(message), error); //#TODO: error check????
}

string buildPIDMessage(struct PIDConst pid) {
  //combines PID into useful message
  stringstream ss;
  ss << " " << pid.p << " " << pid.i << " " << pid.d;
  string command;
  ss >> command;

  return command;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "bigoli_ethernet_drive_relay");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    //Setup speed info
    string speedTopic = nhp.param(string("speedTopic"), string("/speed"));
    auto speedSub = nh.subscribe(speedTopic, 1, speedCallback);

    //accel PID #TODO: CHANGE NAME
    accelDrivePID.p = nhp.param(string("accel_pid_p"), 0.0);
    accelDrivePID.i = nhp.param(string("accel_pid_i"), 0.0);
    accelDrivePID.d = nhp.param(string("accel_pid_d"), 0.0);
    //decel PID #TODO: launch file params ALSO make PID struct to handle better?????
    decelDrivePID.p = nhp.param(string("decel_pid_p"), 0.0);
    decelDrivePID.i = nhp.param(string("decel_pid_i"), 0.0);
    decelDrivePID.d = nhp.param(string("decel_pid_d"), 0.0);

    //Setup steering info
    string steerTopic = nhp.param(string("steeringTopic"), string("/steering"));
    auto steerSub = nh.subscribe(steerTopic, 1, steerCallback);

    maxAngleMsg = nhp.param(string("max_angle_msg_in"), 1.0);

    steeringPID.p = nhp.param(string("steering_pid_p"), 0.0);
    steeringPID.i = nhp.param(string("steering_pid_i"), 0.0);
    steeringPID.d = nhp.param(string("steering_pid_d"), 0.0);

    //IP address and port
    string serverName = nhp.param(string("ip_address"), string("192.168.2.2")); //#TODO: magic # and below
    string serviceName = nhp.param(string("tcp_port"), string("7"));



    // wait for microcontroller to start
    ros::Duration(2.0).sleep(); //#TODO: do we need this anymore?

    //TCP client setup
    /*@Note https://www.boost.org/doc/libs/1_42_0/doc/html/boost_asio/tutorial/tutdaytime1.html
      this code changed based on the version of boost being used by ROS.
      Currently from 1.58 to 1.68 that looks like using io_context instead of io_service
      Look at the updated tutorial by following link and change as need be.
    */
    ROS_INFO_STREAM("Trying to connect to TCP Host");

    boost::asio::io_service io_service;
    tcp::resolver resolver(io_service);
    tcp::resolver::query query(serverName, serviceName);
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    currentSocket = make_shared<tcp::socket>(io_service); //used to allow us to pass socket to functions
    boost::asio::connect(*currentSocket, endpoint_iterator);


    //CONNECTION NOW OPEN, READY TO JAM
    ROS_INFO_STREAM("Connected to TCP Host");

    //send initialization message (PIDs) //accel, deccel, steering #TODO: deccel pids
    //combines PID into useful message; # means init message 
    string initMessage = "#" + buildPIDMessage(accelDrivePID) +
          " " + buildPIDMessage(decelDrivePID) +
          " " + buildPIDMessage(steeringPID);

    sendMessage(initMessage);
    ROS_INFO_STREAM("Sent initialization PID: " + initMessage);


    ros::Rate rate(10); //#TODO set this value to a good rate time/ do we need this?
    while(ros::ok()) {
        ros::spinOnce();

        boost::array<char, 128> buf;
        boost::system::error_code error;

        //write data to MBED
        //Send Motor Command and Steering Command (lead by a $)
        stringstream s;
        s << "$" << speed << " " << steeringAngle;
        string command;
        s >> command;
        sendMessage(command);

        //read data from MBED
        string response = readMessage(); //#TODO: make this useful data


        rate.sleep();
    }

    return 0;
}
