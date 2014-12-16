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
#include <std_msgs/UInt32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include "../../../shared.hpp"
#include "wsg_50_common/Status.h"
#include "wsg_50_common/Move.h"
#include <ros/network.h>
#include <std_msgs/String.h>
#include "fstream"
#include <sstream>
#include "rsd_group1/Log.h"
#include "rsd_group1/general.h"
#include "rsd_group1/OEEmsg.h"

#define MES_WAIT 0
#define MES_LOAD_BRICKS 1
#define MES_SORT_BRICKS 2
#define MES_ABORT 3

#define MES_FREE 0
#define MES_SORTING 1
#define MES_OUT_OF_BRICKS 2
#define MES_ORDER_SORTED 3
#define MES_LOADING 4
#define MES_ERROR 5

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

    void publish_vision_config(bool setting);

    void mes_publish_status(int status);

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

    bool visOutOfBricks;
    bool visOrderComplete;
    bool robQueueEmpty;

    float planned_operating_time;
    float operating_time;
    float availability;
    float net_operating_time;
    float performance;
    float fully_productive_operating_time;
    float quality;
    float OEE;
    float down_time;
    float speed_loss;
    float quality_loss;

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();
    void runStateMachine();
    void mesCommand(int command);
    void OEE_updated();
    void security_abort();


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
    ros::Subscriber mes_command;
    ros::Subscriber OEE_sub;
    ros::Subscriber safety_sub;
    QStringListModel hmi_logging_model;
    QStringListModel rob_logging_model;
    QStringListModel vis_logging_model;
    QStringListModel mes_logging_model;
    QStringListModel con_logging_model;
    QStringListModel complete_logging_model;

    ros::Publisher pub_vis_set;
    ros::Publisher pub_mes_status;

    void robPosCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void statusCallback(const wsg_50_common::Status status);
    void mesCallback(const rsd_group1::general msg);
    void OEECallback(const rsd_group1::OEEmsg msg);

    void safetyCallback(const std_msgs::String msg);

};

}  // namespace HMI

#endif /* HMI_QNODE_HPP_ */
