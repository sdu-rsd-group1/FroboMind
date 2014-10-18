#include "ros/ros.h"
#include "std_msgs/String.h"
using namespace std_msgs;
using namespace std;
using namespace ros;

int main(int argc, char**argv)
{
  init(argc,argv,"Conveyor_test_node");

NodeHandle n;
Publisher string_pub = n.advertise<String>("serial_com",1000);

std::stringstream On;
std::stringstream Off;
std::stringstream Start;
std::stringstream Stop;
std::stringstream Forward;
std::stringstream Reverse;

On 	<< "1On";
Off 	<< "1Off";
Start 	<< "1Start";
Stop 	<< "1Stop";
Forward << "1Forward";
Reverse << "1Reverse";

ros::Rate loop_rate(1);
  while (ros::ok())
  {

    std_msgs::String msg;

    msg.data = On.str();
    string_pub.publish(msg);
    loop_rate.sleep();

    msg.data = Start.str();
    string_pub.publish(msg);
    loop_rate.sleep();

    msg.data = Forward.str();
    string_pub.publish(msg);
    loop_rate.sleep();

    msg.data = Reverse.str();
    string_pub.publish(msg);
    loop_rate.sleep();

    msg.data = Stop.str();
    string_pub.publish(msg);
    loop_rate.sleep();

    msg.data = Off.str();
    string_pub.publish(msg);
    loop_rate.sleep();

    ros::spinOnce();


  }
  return 0;
}
