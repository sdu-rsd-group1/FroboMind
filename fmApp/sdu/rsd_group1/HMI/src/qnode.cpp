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

void QNode::publish_vision_config(bool setting)
{
    std_msgs::String message;
    message.data = "true";
    if(setting)
        message.data = "true";
    else
        message.data = "false";

    pub_vis_set.publish(message);
}

void QNode::mesCallback(const rsd_group1::general msg)
{
    Q_EMIT mesCommand(msg.general);
}

void QNode::logCallback(const rsd_group1::Log new_log)
{

    if(new_log.NodeID == 2 && new_log.CodeID == 8) //Vision out of bricks
    {
        visOutOfBricks = true;
    }
    else if(new_log.NodeID == 2 && new_log.CodeID == 2) //Vision order done
    {
        visOrderComplete = true;
    }

    if(new_log.NodeID == 1 && new_log.CodeID == 1) //Vision out of bricks
    {
        robQueueEmpty = true;
    }

    	std::stringstream ss;

    	t = time(0);   // get time now
     	struct tm * now = localtime( & t );
     	char timeBuf [80];
     	strftime (timeBuf,80,"%H:%M:%S",now);

        ss <<  timeBuf << ": [0x" << hex << new_log.CodeID << "] " << new_log.Text << endl;

        if(new_log.Level == 0)
        {
            Log_Complete	<< ss.rdbuf();
        }

    switch(new_log.NodeID)
	{
		case 0:
		{
            if(new_log.Level == 0)
			{
                Log_Hmi_Normal	<< ss.rdbuf();
			}
            else if(new_log.Level == 1)
			{
                Log_Hmi_Debug << ss.rdbuf();
			}
			break;
		}
		case 1:
		{
            if(new_log.Level == 0)
			{
                Log_Rob_Normal	<< ss.rdbuf();
			}
            else if(new_log.Level == 1)
			{
                Log_Rob_Debug << ss.rdbuf();
			}
			break;
		}
		case 2:
		{
            if(new_log.Level == 0)
			{
                Log_Vis_Normal	<< ss.rdbuf();
			}
            else if(new_log.Level == 1)
			{
                Log_Vis_Debug << ss.rdbuf();
			}
			break;
		}
		case 3:
		{
            if(new_log.Level == 0)
			{
                Log_Mes_Normal	<< ss.rdbuf();
			}
            else if(new_log.Level == 1)
			{
                Log_Mes_Debug << ss.rdbuf();
			}
			break;
		}
		case 4:
		{
            if(new_log.Level == 0)
			{
                Log_Con_Normal	<< ss.rdbuf();
			}
            else if(new_log.Level == 1)
			{
                Log_Con_Debug << ss.rdbuf();
			}
			break;
		}
	}
    log(new_log.NodeID,new_log.Level,ss.str());
}

void QNode::mes_publish_status(int status){
    rsd_group1::general stat;
    stat.general = status;
    pub_mes_status.publish(stat);
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

void QNode::OEECallback(const rsd_group1::OEEmsg msg){
    planned_operating_time = msg.planned_operating_time;
    operating_time = msg.operating_time;
    availability = msg.availability;
    net_operating_time = msg.net_operating_time;
    performance = msg.performance;
    fully_productive_operating_time = msg.fully_productive_operating_time;
    quality = msg.quality;
    OEE = msg.OEE;
    down_time = msg.down_time;
    speed_loss = msg.speed_loss;
    quality_loss = msg.quality_loss;
    Q_EMIT OEE_updated();
}

void QNode::safetyCallback(const std_msgs::String msg)
{
    if(msg.data[0] == 'F')
    {
        cout << "bitches wants to F" << endl;
        Q_EMIT security_abort();
    }
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"HMI");
    if ( ! ros::master::check() ) {
        return false;
    }
    visOrderComplete = false;
    visOutOfBricks = false;
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    rob_pos_sub = n.subscribe("robotics_pose",1000,&QNode::robPosCallback, this);
    safety_sub = n.subscribe("/Safety_node_chatter",1000,&QNode::safetyCallback, this);
    log_sub = n.subscribe("logging",1000,&QNode::logCallback, this);
    mes_command = n.subscribe("/mes/outgoing",1000,&QNode::mesCallback, this);
    OEE_sub = n.subscribe("/OEE_stats",1000,&QNode::OEECallback, this);
    pub_vis_set = n.advertise<std_msgs::String>("vision_config",1000);
    pub_mes_status = n.advertise<rsd_group1::general>("/mes/incoming",1000);
    state_publisher = n.advertise<std_msgs::UInt32>("robot_states", 1000);
    gripper_sub = n.subscribe("wsg_50/status",1000,&QNode::statusCallback,this);
    start();
    return true;
}

void QNode::run() {
    t = time(0);   // get time now
    struct tm * now = localtime( & t );

    char hmi_normal_buffer [80];
    char rob_normal_buffer [80];
    char vis_normal_buffer [80];
    char mes_normal_buffer [80];
    char con_normal_buffer [80];
    char complete_buffer [80];


    char hmi_debug_buffer [80];
    char rob_debug_buffer [80];
    char vis_debug_buffer [80];
    char mes_debug_buffer [80];
    char con_debug_buffer [80];


    strftime (hmi_normal_buffer,80,"HMI_Normal_%H:%M:%S_%d-%m-%Y.txt",now);
    strftime (rob_normal_buffer,80,"Rob_Normal_%H:%M:%S_%d-%m-%Y.txt",now);
    strftime (vis_normal_buffer,80,"Vis_Normal_%H:%M:%S_%d-%m-%Y.txt",now);
    strftime (mes_normal_buffer,80,"MES_Normal_%H:%M:%S_%d-%m-%Y.txt",now);
    strftime (con_normal_buffer,80,"Con_Normal_%H:%M:%S_%d-%m-%Y.txt",now);
    strftime (complete_buffer,80,"Complete_%H:%M:%S_%d-%m-%Y.txt",now);

    strftime (hmi_debug_buffer,80,"HMI_Debug_%H:%M:%S_%d-%m-%Y.txt",now);
    strftime (rob_debug_buffer,80,"Rob_Debug_%H:%M:%S_%d-%m-%Y.txt",now);
    strftime (vis_debug_buffer,80,"Vis_Debug_%H:%M:%S_%d-%m-%Y.txt",now);
    strftime (mes_debug_buffer,80,"MES_Debug_%H:%M:%S_%d-%m-%Y.txt",now);
    strftime (con_debug_buffer,80,"Con_Debug_%H:%M:%S_%d-%m-%Y.txt",now);

        Log_Hmi_Normal.open(hmi_normal_buffer);
        Log_Hmi_Debug.open(hmi_debug_buffer);
        Log_Rob_Normal.open(rob_normal_buffer);
        Log_Rob_Debug.open(rob_debug_buffer);
        Log_Vis_Normal.open(vis_normal_buffer);
        Log_Vis_Debug.open(vis_debug_buffer);
        Log_Mes_Normal.open(mes_normal_buffer);
        Log_Mes_Debug.open(mes_debug_buffer);
        Log_Con_Normal.open(con_normal_buffer);
        Log_Con_Debug.open(con_debug_buffer);
        Log_Complete.open(complete_buffer);


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
				}
			}
			break;
		}
	}
Q_EMIT loggingUpdated();
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
