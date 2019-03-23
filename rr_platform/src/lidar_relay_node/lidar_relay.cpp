#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <iostream>
#include <EthernetSocket.h>

/*
 * This is a communication relay for the Lidar Lite Board.
*/

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_relay");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");


    //IP address and port
    string ip_addr = nhp.param(string("ip_address"), string("192.168.2.2"));
    int port = nhp.param(string("tcp_port"), 7);

    ROS_INFO_STREAM("Trying to connect to TCP Host at " + ip_addr + " port: " + to_string(port));
    EthernetSocket sock(ip_addr, port);

    //CONNECTION NOW OPEN, READY TO JAM
    ROS_INFO_STREAM("Connected to TCP Host!");

    string received;

    while(ros::ok()) {
        ros::spinOnce();
        //read data from Arduino
        sock.sendMessage("$data from node;");
        received = sock.readMessage();
        //ROS_INFO_STREAM("Received: " + received); //debug
        //get data as array with start and end markers removed
        std::vector <std::string> data = EthernetSocket::split(received.substr(received.find('$') + 1, received.find(';') - 1 ), ' ');
        if (!data.empty()) {
            //do something useful like:
            //write data to Arduino Board
            //Send response. EX: $data1 data2;
            //string command = "$" + to_string(distance) + " " + to_string(distance2) + ";";
            //sendMessage(command);
            //ROS_INFO_STREAM("SENT:" + command);
            ROS_INFO_STREAM(data[0] + " " + data[1]);
        }
    }

    return 0;
}
