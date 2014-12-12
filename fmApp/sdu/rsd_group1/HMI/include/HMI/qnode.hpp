/**
 * @file /include/HMI/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef HMI_QNODE_HPP_
#define HMI_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Float32MultiArray.h>
#include "../../../shared.hpp"
#include "wsg_50_common/Status.h"
#include "wsg_50_common/Move.h"
#include <ros/network.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <sstream>



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace HMI {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
    bool init();
    void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
void localLogCallback(std_msgs::UInt32 logmsg);

    float current_config[6];
    float current_pose[6];
    float next_brick_pos[3];

    float wsg_width;
    float wsg_force;

    void publish_state(states state);

    ros::Publisher state_publisher;

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void runStateMachine();

private:
	int init_argc;
	char** init_argv;
    ros::Subscriber log_sub;
    time_t t;
    std::ofstream logfile;

    ros::Subscriber rob_pos_sub;
    ros::Subscriber gripper_sub;
    QStringListModel logging_model;

    void robPosCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void statusCallback(const wsg_50_common::Status status);
    void logCallback(const std_msgs::UInt32::ConstPtr& logmsg);

};

}  // namespace HMI

#endif /* HMI_QNODE_HPP_ */
