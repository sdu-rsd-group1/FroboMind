/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include "fstream"
#include <sstream>
#include "../include/qt_test/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace std;
namespace qt_test {

/*****************************************************************************
** Implementation
*****************************************************************************/

struct{
  unsigned int value;
  char* name;
} err_code[] = {
        { 0x00, "Started up" },
        { 0x01, "Halted" }
};

string err2msg(unsigned int code)
{
    unsigned int new_code = 0;
    string node("");
    if(code < 0x100)
    {
        node.append("Other: ");
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
    else if(code < 0x400)
    {
        node.append("Conveyor: ");
        new_code = code-0x300;
    }
    else
    {
        node.append("Unknown node: ");
        new_code = code-0x400;
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


void QNode::logCallback(const std_msgs::UInt32::ConstPtr& logmsg)
{
    std::string error(err2msg(logmsg->data));
    std::stringstream ss;

    t = time(0);   // get time now
     struct tm * now = localtime( & t );

     char timeBuf [80];
     strftime (timeBuf,80,"%H:%M:%S",now);
    ss <<  timeBuf << ": [0x" << hex << logmsg->data << "] " << error << endl;
    logfile << ss;
    log(ss.str());
}

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
    {}

QNode::~QNode() {
    if(ros::isStarted()) {
      send_command("1Off");
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"HMI");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
    log_sub = n.subscribe("logging",1000,&QNode::logCallback, this);
    command_pub = n.advertise<std_msgs::String>("serial_com",1000);
	start();

    send_command("1On");
	return true;
}

void QNode::run() {

    t = time(0);   // get time now
    struct tm * now = localtime( & t );

    char buffer [80];
    strftime (buffer,80,"%H:%M:%S_%d-%m-%Y.txt",now);

    logfile.open (buffer);
    if(logfile.is_open())
    {
       ros::spin();
    }
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::send_command(string commandmsg)
{
    msg.data = commandmsg;
    command_pub.publish(msg);
}


void QNode::log(const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
    logging_model_msg << msg;
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace qt_test
