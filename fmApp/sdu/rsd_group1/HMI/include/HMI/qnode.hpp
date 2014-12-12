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
#include "fstream"
#include <sstream>
#include "rsd_group1/Log.h"



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

	QStringListModel* HmiLogModel() { return &hmi_logging_model; }
	QStringListModel* RobLogModel() { return &rob_logging_model; }
	QStringListModel* VisLogModel() { return &vis_logging_model; }
	QStringListModel* MesLogModel() { return &mes_logging_model; }
    QStringListModel* ConLogModel() { return &con_logging_model; }
    QStringListModel* CompleteLogModel() { return &complete_logging_model; }

	void log(int nodeid, int level, const std::string &msg);
    void logCallback(const rsd_group1::Log new_log);

    float current_config[6];
    float current_pose[6];
    float next_brick_pos[3];

    float wsg_width;
    float wsg_force;

    void publish_state(states state);

    ros::Publisher state_publisher;


	bool HMI_debug;
	bool Rob_debug;
	bool Vis_debug;
	bool MES_debug;
	bool Con_debug;

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();
    void runStateMachine();


private:
	int init_argc;
	char** init_argv;
    ros::Subscriber log_sub;
    time_t t;
	
    std::ofstream Log_Hmi_Normal;
	std::ofstream Log_Hmi_Debug;
    std::ofstream Log_Rob_Normal;
	std::ofstream Log_Rob_Debug;
    std::ofstream Log_Vis_Normal;
	std::ofstream Log_Vis_Debug;
	std::ofstream Log_Mes_Normal;
	std::ofstream Log_Mes_Debug;
    std::ofstream Log_Con_Normal;
	std::ofstream Log_Con_Debug;
    std::ofstream Log_Complete;


    ros::Subscriber rob_pos_sub;
    ros::Subscriber gripper_sub;
    QStringListModel hmi_logging_model;
    QStringListModel rob_logging_model;
    QStringListModel vis_logging_model;
    QStringListModel mes_logging_model;
    QStringListModel con_logging_model;
    QStringListModel complete_logging_model;


    void robPosCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void statusCallback(const wsg_50_common::Status status);

};

}  // namespace HMI

#endif /* HMI_QNODE_HPP_ */
