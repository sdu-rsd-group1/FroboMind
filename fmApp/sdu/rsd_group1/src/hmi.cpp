#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "fstream"
#include <iostream>
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

time_t t;
std::ofstream logfile;

string err2msg(unsigned int code)
{
	unsigned int new_code = 0;
	string node("");
	if(code < 0x100)
	{
		node.append("HMI: ");
		new_code = code;
	}
	else if(code < 0x200)
	{
		node.append("Robotics: ");
		new_code = code-0x100;
	}
	else if(code < 0x300)
	{
		node.append("Vision: ");
		new_code = code-0x200;
	}
	else
	{
		node.append("Unknown node: ");
		new_code = code-0x300;
	}

    for (int i = 0; err_code[i].name; ++i)
        if (err_code[i].value == new_code)
	{
		node.append(err_code[i].name);
		return node;
	}

node.append("unknown");
    return node;
}

void logCallback(const UInt32::ConstPtr& log)
{
	std::string error(err2msg(log->data));

	t = time(0);   // get time now
     struct tm * now = localtime( & t );

     char timeBuf [80];
     strftime (timeBuf,80,"%H:%M:%S",now);

  	logfile << timeBuf << " - [0x" << hex << log->data << "]: " << error << endl;
}

int main(int argc, char**argv)
{
  ros::init(argc,argv,"HMI");
  ros::NodeHandle n;
  ros::Subscriber robotics_log_sub = n.subscribe("robotics_log",1000,logCallback);
  ros::Subscriber vision_log_sub = n.subscribe("vision_log",1000,logCallback);
  ros::Subscriber other_log_sub = n.subscribe("other_log",1000,logCallback);

     t = time(0);   // get time now
     struct tm * now = localtime( & t );

     char buffer [80];
     strftime (buffer,80,"./roswork/HMI_Logs/%H:%M:%S_%d-%m-%Y.txt",now);

     logfile.open (buffer);
     if(logfile.is_open())
     {
        ros::spin();
     }
	else
	{
		cout << "Error opening\n";
	}
  return 0;
}
