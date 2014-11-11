/**
 * @file /include/qt_test/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qt_test_QNODE_HPP_
#define qt_test_QNODE_HPP_

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
#include "wsg_50_common/Move.h"
#include "wsg_50_common/Status.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace std;
namespace qt_test {

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
    void grasp();
    void release();
    void test_robot(void);

    float force;
    float position;

	/*********************
	** Logging
	**********************/

	QStringListModel* loggingModel() { return &logging_model; }
    void log( const std::string &msg);


    void dummyCallback(std_msgs::UInt32 logmsg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();





private:
	int init_argc;
	char** init_argv;
    ros::Subscriber log_sub;
    ros::Subscriber grip_sub;
    ros::Publisher command_pub;
    ros::Timer timer;
    QStringListModel logging_model;
    time_t t;
    std::ofstream logfile;
    void callback(const ros::TimerEvent& event);
    void logCallback(const std_msgs::UInt32::ConstPtr& logmsg);
    void gripCallback(const wsg_50_common::Status::ConstPtr& msg);
    std_msgs::String msg;
    ros::ServiceClient client_grasp;
    wsg_50_common::Move srv_grasp;
    ros::ServiceClient client_release;
    wsg_50_common::Move srv_release;

    ros::ServiceClient robot_client;
};

}  // namespace qt_test

#endif /* qt_test_QNODE_HPP_ */
