#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "rx60controller/command.h"
#include "std_msgs/Float32MultiArray.h"
#include "robot.hpp"
#include "wsg_50_common/Move.h"
#include "wsg_50_common/Status.h"
#include "../../shared.hpp"
#include "rsd_group1/Num.h"
#include "rsd_group1/Log.h"
#include <time.h>

#include <sstream>

using namespace std_msgs;
using namespace std;
using namespace ros;

rx60controller::commandRequest cmdReq;
rx60controller::commandResponse cmdRes;
ros::ServiceClient robot_client;

ros::ServiceClient gripper_grasp_client;
ros::ServiceClient gripper_release_client;
wsg_50_common::Move srv_grasp;
wsg_50_common::Move srv_release;

rsd_group1::Num current_order;

float wsg_width;
float wsg_force;

bool upperBrickWait = false;

robot::robot *Staubli;

double getPositionFromOrder(double speed, double seconds)
{
double new_time = ros::Time::now().toSec();

    cout << "get position function: " << speed << " " << seconds << " " << new_time << " " << (new_time-seconds) << endl;

    return speed*(new_time-seconds) + PICKUP_BOX_YNEG-PICKUP_BRICK_OFFSET;
}

void stateStop(){
    //Staubli->goToZero();
}

void stateReset(){
    cout << "Going to middle pos";
    Staubli->goToMiddlePos();
}

void stateStart(){

}

void stateReady(){

}

void stateExecute(){

}

void stateSuspended(){

}

void stateUpperBrick(){


}

void stateReleaseBrick(){
    Staubli->release();
}

void stateLowerBrick(){
    Staubli->goToBottomPos();
}

void stateGraspBrick(){
    Staubli->next_brick[0] = 0;
    Staubli->next_brick[1] = 0;
    Staubli->next_brick[2] = 0;
    Staubli->grasp();
}

void stateBrickToMiddle(){
    Staubli->brickToMiddle();
}

void stateMiddleToBox(){
    Staubli->middleToBox();
}

void stateBoxToMiddle(){
    Staubli->boxToMiddle();
}

void stateCompleted(){

}

void orderCallback(const rsd_group1::Num status)
{
    cout << "Order" << endl;
    Staubli->orderlist.push_back(status);
    //cout << "New Brick " << Staubli->orderlist.front().color << endl;
    for(int i = 0; i < Staubli->orderlist.size(); i++)
    {
        cout << i <<". callback: "<<Staubli->orderlist.at(i).brick << endl;
    }
}

void statusCallback(const wsg_50_common::Status status){
    wsg_width = status.width;
    wsg_force = status.force;
    Staubli->wsg_force = status.force;
    Staubli->wsg_width = status.width;
}

void stateCallback(const std_msgs::UInt32::ConstPtr& state){
    switch((states)state->data)
    {
        case STOP:
        {
            stateStop();
            break;
        }
        case RESET:
        {
            stateReset();
            break;
        }
        case START:
        {
            stateStart();
            break;
        }
        case READY:
        {
            stateReady();
            break;
        }
        case EXECUTE:
        {
            stateExecute();
            break;
        }
        case SUSPENDED:
        {
            stateSuspended();
            break;
        }
        case GO_TO_UPPER_BRICK:
        {
            stateUpperBrick();
            break;
        }
        case OPEN_GRIP:
        {
            stateReleaseBrick();
            break;
        }
        case GO_TO_LOWER_BRICK:
        {
            stateLowerBrick();
            break;
        }
        case GRASP_BRICK:
        {
            stateGraspBrick();
            break;
        }
        case BRICK_TO_MIDDLE:
        {
            stateBrickToMiddle();
            break;
        }
        case MIDDLE_TO_BOX:
        {
            stateMiddleToBox();
            break;
        }
        case RELEASE_BRICK:
        {
            stateReleaseBrick();
            break;
        }
        case BOX_TO_MIDDLE:
        {
            stateBoxToMiddle();
            break;
        }
        case COMPLETED:
        {
            stateCompleted();
            break;
        }
        case ABORT:
        {
			break;
		}
		case SECURITY:
		{
			break;
		}
    }
    if((states)state->data == GO_TO_UPPER_BRICK)
    {
        upperBrickWait = true;
    }
    else
    {
        upperBrickWait = false;
    }
}

int main(int argc, char **argv)
{
  	init(argc,argv, "Robotics");
  	NodeHandle n;
    Publisher log_pub = n.advertise<rsd_group1::Log>("logging",1000);
    Publisher rob_pose_pub = n.advertise<Float32MultiArray>("robotics_pose",1000);
    ros::Subscriber state_sub = n.subscribe("robot_states",1000,stateCallback);
    ros::Subscriber gripper_sub = n.subscribe("wsg_50/status",1000,statusCallback);

    ros::Subscriber brick_sub = n.subscribe("/brick2pick",1000,orderCallback);

    gripper_grasp_client = n.serviceClient<wsg_50_common::Move>("/wsg_50/grasp");

    srv_grasp.request.width = GRASP_WIDTH;
    srv_grasp.request.speed = 400.0;
    gripper_grasp_client.call(srv_grasp);

    gripper_release_client = n.serviceClient<wsg_50_common::Move>("/wsg_50/release");

	robot_client = n.serviceClient<rx60controller::command>("/rx60_controller/rx60_command");

    Staubli = new robot::robot(robot_client,gripper_grasp_client,gripper_release_client);

    Staubli->initialize();

    rsd_group1::Log log;
	log.NodeID = 1;
	log.CodeID = 0;
	log.Level = 0;
	log.Text = "Robot initialized";	
	log_pub.publish(log);
	
   cout << "initialized" << endl;
   bool first = true;
   ros::Rate loop_rate(10);
   while(ros::ok())
   {
	log.NodeID = 1;
	log.CodeID = 1;
	log.Level = 1;
	log.Text = "Spamming Debug mode";	
	log_pub.publish(log);
       if(upperBrickWait)
       {
           if(!Staubli->orderlist.empty())
           {
               for(int i = 0; i < Staubli->orderlist.size(); i++)
               {
                   cout << i <<". upper: "<<Staubli->orderlist.at(i).brick << endl;
               }
               current_order = Staubli->orderlist.front();
               double y = getPositionFromOrder(current_order.speed,current_order.time);
               double x = PICKUP_BOX_XNEG-current_order.x;
               double angle = -current_order.angle;

               angle += ANGLE_OFFSET;


               cout << "Order: " << x << ", " << y << ", " << angle << endl;

               //if(first)
               //{
                //cout << "GOING TO goToUpperPos() TRUE" << endl;
                //Staubli->goToUpperPos2(x,PICKUP_BOX_YNEG,((3.1415/180.0)*angle));
                //first = false;
               //}

               if(y > PICKUP_BOX_YNEG)
               {
                   cout << "GOING TO goToUpperPos()" << endl;
				   Staubli->next_brick[0] = x;
				   Staubli->next_brick[1] = y;
				   Staubli->next_brick[2] = (3.1415/180.0)*angle;
				   Staubli->goToUpperPos();
                   Staubli->orderlist.erase(Staubli->orderlist.begin());
                   first = true;
               }


           }
       }
       rob_pose_pub.publish(Staubli->get_pub_pose());
	    ros::spinOnce();
        loop_rate.sleep();
	}


  	return 0;
}
