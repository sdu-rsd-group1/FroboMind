#include "ros/ros.h"
#include "std_msgs/UInt32.h"

#include <sstream>
using namespace std_msgs;
using namespace std;
using namespace ros;

int main(int argc, char **argv)
{
  	init(argc,argv, "Robotics");
  	NodeHandle n;
  	Publisher log_pub = n.advertise<UInt32>("logging",1000);

	UInt32 err;
	err.data = 0x100;	
	Rate rate(1);
	rate.sleep();
	log_pub.publish(err);
  
	while(ros::ok())
  	{
	
  	}


  	return 0;
}
