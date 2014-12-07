#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "rx60controller/command.h"
#include "std_msgs/Float32MultiArray.h"
#include "robot.hpp"
#include "wsg_50_common/Move.h"
#include "wsg_50_common/Status.h"
#include "../../shared.hpp"
#include "rsd_group1/Num.h"
#include <time.h>

#include <sstream>

#define LENGTH_OFFSET 2

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

robot::robot *Staubli;

double getPositionFromOrder(double speed, double seconds)
{
    time_t now;
    struct tm sendTime;

    time(&now);
    sendTime.tm_sec = seconds;

    seconds = difftime(now,mktime(&sendTime));

    return speed*seconds + LENGTH_OFFSET;
}

void stateStop(){
    Staubli->goToZero();
}

void stateStart(){
    cout << "Going to middle pos";
    Staubli->goToMiddlePos();
}

void stateReady(){

}

void stateExecute(){

}

void stateSuspended(){

}

void stateUpperBrick(){
    if(!Staubli->orderlist.empty())
    {
        current_order = Staubli->orderlist.front();
        double y = getPositionFromOrder(current_order.speed,current_order.time);
        double x = current_order.x-PICKUP_BOX_XNEG;
        double angle = current_order.angle;

        cout << "Order: " << x << ", " << y << ", " << angle << endl;
//        if(y > PICKUP_BRICK_OFFSET-PICKUP_BOX_YNEG)
//        {
//            Staubli->orderlist.erase(0);
//            Staubli->next_brick[0] = current_order.x-0.3;
//            Staubli->next_brick[1] = getPositionFromOrder(current_order.speed,current_order.time);
//            Staubli->next_brick[2] = current_order.angle;
//            Staubli->goToUpperPos();
//        }
    }

}

void stateReleaseBrick(){
    Staubli->release();
}

void stateLowerBrick(){
    Staubli->goToBottomPos();
}

void stateGraspBrick(){
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
    Staubli->orderlist.push_back(status);
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
    }
}

int main(int argc, char **argv)
{
  	init(argc,argv, "Robotics");
  	NodeHandle n;
    Publisher log_pub = n.advertise<UInt32>("logging",1000);
    Publisher rob_pose_pub = n.advertise<Float32MultiArray>("robotics_pose",1000);
    ros::Subscriber state_sub = n.subscribe("robot_states",1000,stateCallback);
    ros::Subscriber gripper_sub = n.subscribe("wsg_50/status",1000,statusCallback);

    ros::Subscriber brick_sub = n.subscribe("rsd_group1/brick2pick",1000,orderCallback);

    gripper_grasp_client = n.serviceClient<wsg_50_common::Move>("/wsg_50/grasp");

    srv_grasp.request.width = 25.0;
    srv_grasp.request.speed = 400.0;
    gripper_grasp_client.call(srv_grasp);

    gripper_release_client = n.serviceClient<wsg_50_common::Move>("/wsg_50/release");

	robot_client = n.serviceClient<rx60controller::command>("/rx60_controller/rx60_command");

    Staubli = new robot::robot(robot_client,gripper_grasp_client,gripper_release_client);

    Staubli->initialize();

	UInt32 err;
	err.data = 0x100;	
	log_pub.publish(err);
	
   cout << "initialized" << endl;
   
   ros::Rate loop_rate(10);
   while(ros::ok())
   {
       rob_pose_pub.publish(Staubli->get_pub_pose());
	    ros::spinOnce();
        loop_rate.sleep();
	}


  	return 0;
}
