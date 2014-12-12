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
** Implementation
*****************************************************************************/


void QNode::logCallback(const rsd_group1::Log new_log)
{
    	std::stringstream ss;

    	t = time(0);   // get time now
     	struct tm * now = localtime( & t );
     	char timeBuf [80];
     	strftime (timeBuf,80,"%H:%M:%S",now);

        ss <<  timeBuf << ": [0x" << hex << new_log.CodeID << "] " << new_log.Text << endl;

    switch(new_log.NodeID)
	{
		case 0:
		{
            if(new_log.Level == 0)
			{
                new_log_Hmi_Normal	<< ss.rdbuf();
			}
            else if(new_log.Level == 1)
			{
                new_log_Hmi_Debug << ss.rdbuf();
			}
			break;
		}
		case 1:
		{
            if(new_log.Level == 0)
			{
                new_log_Rob_Normal	<< ss.rdbuf();
			}
            else if(new_log.Level == 1)
			{
                new_log_Rob_Debug << ss.rdbuf();
			}
			break;
		}
		case 2:
		{
            if(new_log.Level == 0)
			{
                new_log_Vis_Normal	<< ss.rdbuf();
			}
            else if(new_log.Level == 1)
			{
                new_log_Vis_Debug << ss.rdbuf();
			}
			break;
		}
		case 3:
		{
            if(new_log.Level == 0)
			{
                new_log_Mes_Normal	<< ss.rdbuf();
			}
            else if(new_log.Level == 1)
			{
                new_log_Mes_Debug << ss.rdbuf();
			}
			break;
		}
		case 4:
		{
            if(new_log.Level == 0)
			{
                new_log_Con_Normal	<< ss.rdbuf();
			}
            else if(new_log.Level == 1)
			{
                new_log_Con_Debug << ss.rdbuf();
			}
			break;
		}
	}
    log(new_log.NodeID,new_log.Level,ss.str());
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

    	Log_Hmi_Normal.open("HMI_Normal_" + buffer);
	Log_Hmi_Debug.open("HMI_Debug_" + buffer);
    	Log_Rob_Normal.open("Rob_Normal_" + buffer);
	Log_Rob_Debug.open("Rob_Debug_" + buffer);
    	Log_Vis_Normal.open("Vis_Normal_" + buffer);
	Log_Vis_Debug.open("Vis_Debug_" + buffer);
    	Log_Mes_Normal;.open("MES_Normal_" + buffer);
	Log_Mes_Debug.open("MES_Debug_" + buffer);
    	Log_Con_Normal;.open("Con_Normal_" + buffer);
	Log_Con_Debug.open("Con_Debug_" + buffer);


	HMI_debug = false;
	Rob_debug = false;
	Vis_debug = false;
	MES_debug = false;
	Con_debug = false;

    //logfile.open (buffer);
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


void QNode::log(int nodeid, int level, const std::string &msg) {

	switch(nodeid)
	{
		case 0:
		{
			if(level == 0)
			{
				hmi_logging_model.insertRows(hmi_logging_model.rowCount(),1);
				std::stringstream hmi_logging_model_msg;
			    	hmi_logging_model_msg << msg;
				QVariant new_row(QString(hmi_logging_model_msg.str().c_str()));
				hmi_logging_model.setData(hmi_logging_model.index(hmi_logging_model.rowCount()-1),new_row);

                complete_logging_model.insertRows(complete_logging_model.rowCount(),1);
                std::stringstream complete_logging_model_msg;
                complete_logging_model_msg << "HMI node: " << msg;
                QVariant new_row1(QString(complete_logging_model_msg.str().c_str()));
                complete_logging_model.setData(complete_logging_model.index(complete_logging_model.rowCount()-1),new_row1);

                Q_EMIT loggingUpdated();
			}
			else if(level == 1)
			{
				if(HMI_debug)
				{
					hmi_logging_model.insertRows(hmi_logging_model.rowCount(),1);
					std::stringstream hmi_logging_model_msg;
				    	hmi_logging_model_msg << msg;
					QVariant new_row(QString(hmi_logging_model_msg.str().c_str()));
					hmi_logging_model.setData(hmi_logging_model.index(hmi_logging_model.rowCount()-1),new_row);
					Q_EMIT hmiLogUpdated();
				}
			}
			break;
		}
		case 1:
		{
			if(level == 0)
			{
				rob_logging_model.insertRows(rob_logging_model.rowCount(),1);
				std::stringstream rob_logging_model_msg;
			    	rob_logging_model_msg << msg;
				QVariant new_row(QString(rob_logging_model_msg.str().c_str()));
				rob_logging_model.setData(rob_logging_model.index(rob_logging_model.rowCount()-1),new_row);

                complete_logging_model.insertRows(complete_logging_model.rowCount(),1);
                std::stringstream complete_logging_model_msg;
                complete_logging_model_msg << "Robotics node: " << msg;
                QVariant new_row1(QString(complete_logging_model_msg.str().c_str()));
                complete_logging_model.setData(complete_logging_model.index(complete_logging_model.rowCount()-1),new_row1);

                Q_EMIT loggingUpdated();
			}
			else if(level == 1)
			{
				if(Rob_debug)
				{
					rob_logging_model.insertRows(rob_logging_model.rowCount(),1);
					std::stringstream rob_logging_model_msg;
				    	rob_logging_model_msg << msg;
					QVariant new_row(QString(rob_logging_model_msg.str().c_str()));
					rob_logging_model.setData(rob_logging_model.index(rob_logging_model.rowCount()-1),new_row);
					Q_EMIT robLogUpdated();
				}
			}
			break;
		}
		case 2:
		{
			if(level == 0)
			{
				vis_logging_model.insertRows(vis_logging_model.rowCount(),1);
				std::stringstream vis_logging_model_msg;
			    	vis_logging_model_msg << msg;
				QVariant new_row(QString(vis_logging_model_msg.str().c_str()));
				vis_logging_model.setData(vis_logging_model.index(vis_logging_model.rowCount()-1),new_row);

                complete_logging_model.insertRows(complete_logging_model.rowCount(),1);
                std::stringstream complete_logging_model_msg;
                complete_logging_model_msg << "Vision node: " << msg;
                QVariant new_row1(QString(complete_logging_model_msg.str().c_str()));
                complete_logging_model.setData(complete_logging_model.index(complete_logging_model.rowCount()-1),new_row1);

                Q_EMIT loggingUpdated();
			}
			else if(level == 1)
			{
				if(Vis_debug)
				{
					vis_logging_model.insertRows(vis_logging_model.rowCount(),1);
					std::stringstream vis_logging_model_msg;
				    	vis_logging_model_msg << msg;
					QVariant new_row(QString(vis_logging_model_msg.str().c_str()));
					vis_logging_model.setData(vis_logging_model.index(vis_logging_model.rowCount()-1),new_row);
					Q_EMIT visLogUpdated();
				}
			}
			break;
		}
		case 3:
		{
			if(level == 0)
			{
				mes_logging_model.insertRows(mes_logging_model.rowCount(),1);
				std::stringstream mes_logging_model_msg;
			    	mes_logging_model_msg << msg;
				QVariant new_row(QString(mes_logging_model_msg.str().c_str()));
				mes_logging_model.setData(mes_logging_model.index(mes_logging_model.rowCount()-1),new_row);

                complete_logging_model.insertRows(complete_logging_model.rowCount(),1);
                std::stringstream complete_logging_model_msg;
                complete_logging_model_msg << "MES node: " << msg;
                QVariant new_row1(QString(complete_logging_model_msg.str().c_str()));
                complete_logging_model.setData(complete_logging_model.index(complete_logging_model.rowCount()-1),new_row1);

                Q_EMIT loggingUpdated();
			}
			else if(level == 1)
			{
				if(MES_debug)
				{
					mes_logging_model.insertRows(mes_logging_model.rowCount(),1);
					std::stringstream mes_logging_model_msg;
				    	mes_logging_model_msg << msg;
					QVariant new_row(QString(mes_logging_model_msg.str().c_str()));
					mes_logging_model.setData(mes_logging_model.index(mes_logging_model.rowCount()-1),new_row);
					Q_EMIT mesLogUpdated();
				}
			}
			break;
		}
		case 4:
		{
			if(level == 0)
			{
				con_logging_model.insertRows(con_logging_model.rowCount(),1);
				std::stringstream con_logging_model_msg;
			    	con_logging_model_msg << msg;
				QVariant new_row(QString(con_logging_model_msg.str().c_str()));
				con_logging_model.setData(con_logging_model.index(con_logging_model.rowCount()-1),new_row);

                complete_logging_model.insertRows(complete_logging_model.rowCount(),1);
                std::stringstream complete_logging_model_msg;
                complete_logging_model_msg << "Conveyor node: " << msg;
                QVariant new_row1(QString(complete_logging_model_msg.str().c_str()));
                complete_logging_model.setData(complete_logging_model.index(complete_logging_model.rowCount()-1),new_row1);

                Q_EMIT loggingUpdated();
			}
			else if(level == 1)
			{
				if(Con_debug)
				{
					con_logging_model.insertRows(con_logging_model.rowCount(),1);
					std::stringstream con_logging_model_msg;
				    	con_logging_model_msg << msg;
					QVariant new_row(QString(con_logging_model_msg.str().c_str()));
					con_logging_model.setData(con_logging_model.index(con_logging_model.rowCount()-1),new_row);
					Q_EMIT conLogUpdated();
				}
			}
			break;
		}
	}

	 // used to readjust the scrollbar
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
