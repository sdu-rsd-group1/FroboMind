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

#include "../include/HMI/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace std;

namespace HMI {

/*****************************************************************************
** Log codes
*****************************************************************************/

struct code_format{
  unsigned int value;
  char* name;
} log_code[] = {
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

string code2msg(unsigned int code)
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

    for (int i = 0; log_code[i].name; ++i)
        if (log_code[i].value == new_code)
    {
        node.append(log_code[i].name);
        return node;
    }

    node.append("unknown");
        return node;
}

/*****************************************************************************
** Implementation
*****************************************************************************/


void QNode::logCallback(const std_msgs::UInt32::ConstPtr& logmsg)
{
    std::string error(code2msg(logmsg->data));
    std::stringstream ss;

    t = time(0);   // get time now
     struct tm * now = localtime( & t );

     char timeBuf [80];
     strftime (timeBuf,80,"%H:%M:%S",now);
    ss <<  timeBuf << ": [0x" << hex << logmsg->data << "] " << error << endl;
    logfile << ss.rdbuf();
    log(ss.str());
}

void QNode::localLogCallback(std_msgs::UInt32 logmsg)
{
    std::string error(code2msg(logmsg.data));
    std::stringstream ss;

    t = time(0);   // get time now
     struct tm * now = localtime( & t );

     char timeBuf [80];
     strftime (timeBuf,80,"%H:%M:%S",now);
    ss <<  timeBuf << ": [0x" << hex << logmsg.data << "] " << error << endl;
    logfile << ss.rdbuf();
    log(ss.str());
}

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
    log_sub = n.subscribe("logging",1000,&QNode::logCallback, this);
    state_publisher = n.advertise<std_msgs::UInt32>("robot_states", 1000);
    gripper_sub = n.subscribe("wsg_50/status",1000,&QNode::statusCallback,this);
    start();
    return true;
}

void QNode::run() {
    t = time(0);   // get time now
    struct tm * now = localtime( & t );

    char buffer [80];
    strftime (buffer,80,"%H:%M:%S_%d-%m-%Y.txt",now);

    logfile.open (buffer);
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


void QNode::log(const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
    logging_model_msg << msg;
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
