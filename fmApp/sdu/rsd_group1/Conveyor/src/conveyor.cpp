#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rsd_group1/Log.h"

using namespace std_msgs;
using namespace std;
using namespace ros;

int main(int argc, char**argv)
{
  init(argc,argv,"Conveyor_node");

NodeHandle n;
Publisher string_pub = n.advertise<String>("serial_com",1000);
Publisher log_pub = n.advertise<rsd_group1::Log>("logging",1000);

std::stringstream On1;
std::stringstream Off1;
std::stringstream Start1;
std::stringstream Stop1;
std::stringstream Forward1;
std::stringstream Reverse1;

std::stringstream On2;
std::stringstream Off2;
std::stringstream Start2;
std::stringstream Stop2;
std::stringstream Forward2;
std::stringstream Reverse2;

On1 		<< "1On";
Off1 		<< "1Off";
Start1 		<< "1Start";
Stop1 		<< "1Stop";
Forward1 	<< "1Forward";
Reverse1 	<< "1Reverse";

On2 		<< "2On";
Off2 		<< "2Off";
Start2 		<< "2Start";
Stop2 		<< "2Stop";
Forward2 	<< "2Forward";
Reverse2 	<< "2Reverse";

ros::Rate loop_rate(1);

rsd_group1::Log log_msg;
log_msg.NodeID = 4;
log_msg.CodeID = 0;
log_msg.Level = 0;
log_msg.Text = "Conveyor node initialized";

log_pub.publish(log_msg);


    std_msgs::String msg;

    msg.data = On1.str();
    string_pub.publish(msg);
    loop_rate.sleep();
    msg.data = On2.str();
    string_pub.publish(msg);
    loop_rate.sleep();

    msg.data = Forward1.str();
    string_pub.publish(msg);
    loop_rate.sleep();
    msg.data = Reverse2.str();
    string_pub.publish(msg);
    loop_rate.sleep();


  while (ros::ok())
  {

    msg.data = Start1.str();
    string_pub.publish(msg);
    loop_rate.sleep();
    msg.data = Start2.str();
    string_pub.publish(msg);
    loop_rate.sleep();


    ros::spinOnce();


  }
  return 0;
}
