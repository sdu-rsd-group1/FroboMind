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
#include "std_msgs/UInt32.h"
#include "std_msgs/String.h"
#include "fstream"
#include <std_msgs/Float32MultiArray.h>





/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace std;

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
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
    void send_command(string msg);

	/*********************
	** Logging
	**********************/

	QStringListModel* loggingModel() { return &logging_model; }
    void log( const std::string &msg);

    void dummyCallback(std_msgs::UInt32 logmsg);

    float current_config[6];
    float current_pose[6];


Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();



private:
	int init_argc;
	char** init_argv;
    ros::Subscriber log_sub;
    ros::Subscriber rob_pos_sub;
    ros::Subscriber grip_sub;
    ros::Publisher command_pub;
    ros::Publisher robot_config_pub;
    ros::Timer timer;
    QStringListModel logging_model;
    time_t t;
    std::ofstream logfile;
    void callback(const ros::TimerEvent& event);
    void logCallback(const std_msgs::UInt32::ConstPtr& logmsg);
    void robPosCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

    std_msgs::String msg;

};

}  // namespace HMI

#endif /* HMI_QNODE_HPP_ */
