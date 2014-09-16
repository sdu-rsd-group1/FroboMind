#include "ros/ros.h"
#include "std_msgs/UInt32.h"

#include <sstream>
using namespace std_msgs;

int main(int argc, char **argv)
{
  ros::init(argc,argv, "Robotics");
  ros::NodeHandle n;
  ros::Publisher log_pub = n.advertise<UInt32>("robotics_log",1000);
  ros::Rate loop_rate(1); 
  int count = 0;
  while(ros::ok())
  {
    UInt32 err;
    err.data = 0x100+count;
    log_pub.publish(err);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}
