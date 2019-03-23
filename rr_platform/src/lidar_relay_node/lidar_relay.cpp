#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <iostream>

/*@NOTE THIS CODE USES BOOST 1.58 as it is the version currently installed with ROS.
  If that changes, this code will need to be updated as such!
  So don't fear if it breaks, just fix the things by checking the links below to examples given.
*/

using namespace std;
using boost::asio::ip::tcp;


std::unique_ptr<tcp::socket> currentSocket;


string readMessage() {
  //read data from TCP connection
  boost::array<char, 128> buf;
  boost::system::error_code error;

  size_t len = currentSocket->read_some(boost::asio::buffer(buf), error);
  string reading(buf.begin(), buf.end()); //convert buffer into useable string

  if (error == boost::asio::error::eof) {
    // Connection closed cleanly by peer
    ROS_ERROR_STREAM("TCP Connection closed by peer. Disconnecting");
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

/**
 * @note http://stackoverflow.com/a/27511119
 */
std::vector <std::string> split(const std::string &s, char delim) {
    std::stringstream ss(s);
    std::string item;
    std::vector <std::string> elems;
    while (std::getline(ss, item, delim)) {
        elems.push_back(std::move(item));
    }
    return elems;
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_relay");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");


    //IP address and port
    string serverName = nhp.param(string("ip_address"), string("192.168.2.2"));
    string serviceName = nhp.param(string("tcp_port"), string("7"));


    ROS_INFO_STREAM("Trying to connect to TCP Host at " + serverName + " port: " + serviceName);

    boost::asio::io_service io_service; //@NOTE: in later versions of boost, this may have a name change
    tcp::resolver resolver(io_service);
    tcp::resolver::query query(serverName, serviceName);
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    currentSocket = make_unique<tcp::socket>(io_service); //used to allow us to pass socket to functions
    boost::asio::connect(*currentSocket, endpoint_iterator);


    //CONNECTION NOW OPEN, READY TO JAM
    ROS_INFO_STREAM("Connected to TCP Host!");

    string received;

    while(ros::ok()) {
        ros::spinOnce();
        //read data from Arduino
        sendMessage("$data from node;");
        received = readMessage();
        //ROS_INFO_STREAM("Received: " + received); //debug
        //get data as array
        std::vector <std::string> data = split(received.substr(received.find('$'), received.find(';') ), ' ');
        if (!data.empty()) {
            //do something useful like:
            //write data to Arduino Board
            //Send response. EX: $data1 data2;
            //string command = "$" + to_string(distance) + " " + to_string(distance2) + ";";
            //sendMessage(command);
            //ROS_INFO_STREAM("SENT:" + command);
            ROS_INFO_STREAM(data[0] + data[1]);
        }
    }

    return 0;
}
