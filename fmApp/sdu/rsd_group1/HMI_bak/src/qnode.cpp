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
#include "../include/HMI/qnode.hpp"
#include "wsg_50_common/Move.h"
#include "wsg_50_common/Status.h"


namespace HMI {

/*****************************************************************************
** Implementation
*****************************************************************************/



struct{
  unsigned int value;
  char* name;
} err_code[] = {
    { 0x00, "Started up" },
    { 0x01, "Halted" },
    { 0x02, "1, Started" },
    { 0x03, "2, Started" },
    { 0x04, "1, Stopped" },
    { 0x05, "2, Stopped" },
    { 0x06, "1, Going forward" },
    { 0x07, "2, Going forward" },
    { 0x08, "1, Going backwards" },
    { 0x09, "2, Going backwards" }
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
    logfile << ss.rdbuf();
    log(ss.str());
}

void QNode::dummyCallback(std_msgs::UInt32 logmsg)
{
    std::string error(err2msg(logmsg.data));
    std::stringstream ss;

    t = time(0);   // get time now
     struct tm * now = localtime( & t );

     char timeBuf [80];
     strftime (timeBuf,80,"%H:%M:%S",now);
    ss <<  timeBuf << ": [0x" << hex << logmsg.data << "] " << error << endl;
    logfile << ss.rdbuf();
    log(ss.str());
}

void QNode::robPosCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    current_config[0] = msg->data[0];
    current_config[1] = msg->data[1];
    current_config[2] = msg->data[2];
    current_config[3] = msg->data[3];
    current_config[4] = msg->data[4];
    current_config[5] = msg->data[5];

    current_pose[0] = msg->data[6];
    current_pose[1] = msg->data[7];
    current_pose[2] = msg->data[8];
    current_pose[3] = msg->data[9];
    current_pose[4] = msg->data[10];
    current_pose[5] = msg->data[11];
}

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
    {cout << "qnode constructor" << endl;}

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

    cout << "qnode start" << endl;

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
    log_sub = n.subscribe("logging",1000,&QNode::logCallback, this);
    rob_pos_sub = n.subscribe("robotics_pose",1000,&QNode::robPosCallback, this);
    command_pub = n.advertise<std_msgs::String>("serial_com",1000);

	start();

    send_command("1On");
    cout << "init end" << endl;

	return true;
}

void QNode::run() {

    cout << "run start" << endl;
    t = time(0);   // get time now
    struct tm * now = localtime( & t );

    char buffer [80];
    strftime (buffer,80,"%H:%M:%S_%d-%m-%Y.txt",now);

    logfile.open (buffer);
    ros::Rate Loop_rate(100);
    cout << "running" << endl;
    if(logfile.is_open())
    {
       while(ros::ok())
       {
           ros::spinOnce();
           Loop_rate.sleep();
       }
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


}  // namespace HMI
