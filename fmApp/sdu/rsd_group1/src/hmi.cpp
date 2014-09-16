#include "ros/ros.h"
#include "std_msgs/UInt32.h"
using namespace std_msgs;
using namespace std;

//--------------------------------------------------------------//

struct{
  unsigned int value;
  char* name;
} err_code[] = {
    { 0x00, "Started up" },
    { 0x01, "Halted" }
};

//-------------------------------------------------------------//

char* err2msg(unsigned int code)
{
	unsigned int new_code = 0;
	char msg[80];
	if(code < 0x100)
	{
		strcpy(msg,"HMI: ");
		new_code = code;
	}
	else if(code < 0x200)
	{
		strcpy(msg,"Robotics: ");
		new_code = code-0x100;
	}
	else if(code < 0x300)
	{
		strcpy(msg,"Vision: ");
		new_code = code-0x200;
	}

    for (int i = 0; err_code[i].name; ++i)
        if (err_code[i].value == new_code)
            return strcat(msg,err_code[i].name);
    return strcat(msg,"unknown");
}

void logCallback(const UInt32::ConstPtr& log)
{
  ROS_INFO(err2msg(log->data));
}

int main(int argc, char**argv)
{
  ros::init(argc,argv,"HMI");
  ros::NodeHandle n;
  ros::Subscriber robotics_log_sub = n.subscribe("robotics_log",1000,logCallback);
  ros::spin();
  return 0;
}
