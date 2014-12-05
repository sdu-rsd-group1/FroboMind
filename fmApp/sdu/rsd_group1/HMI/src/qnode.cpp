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
#include <sstream>
#include "../include/HMI/qnode.hpp"
#include "std_msgs/UInt32.h"
#include "wsg_50_common/Status.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace std;
namespace HMI {

/*****************************************************************************
** Implementation
*****************************************************************************/

void QNode::publish_state(states state){
    std_msgs::UInt32 msg;
    msg.data = (uint32_t)state;
    state_publisher.publish(msg);
}

QNode::QNode(int argc, char** argv ) :init_argc(argc),init_argv(argv){
        for(int i = 0; i < 6; i++)
        {
            current_config[i] = 0.0;
            current_pose[i] = 0.0;
        }
    }

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

void QNode::statusCallback(const wsg_50_common::Status status){
    wsg_width = status.width;
    wsg_force = status.force;
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"HMI");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    rob_pos_sub = n.subscribe("robotics_pose",1000,&QNode::robPosCallback, this);
    state_publisher = n.advertise<std_msgs::UInt32>("robot_states", 1000);
    gripper_sub = n.subscribe("wsg_50/status",1000,&QNode::statusCallback,this);
    start();
    return true;
}

void QNode::run() {
    ros::Rate loop_rate(1);
    int count = 0;
    while ( ros::ok() ) {

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
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

    next_brick_pos[0] = msg->data[12];
    next_brick_pos[1] = msg->data[13];
    next_brick_pos[2] = msg->data[14];

    Q_EMIT runStateMachine();
}

}  // namespace HMI
