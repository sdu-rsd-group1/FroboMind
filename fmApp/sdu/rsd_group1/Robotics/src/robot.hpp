#ifndef robot_HPP
#define robot_HPP

/*****************************************************************************
** Includes
*****************************************************************************/
#include "ros/ros.h"
#include <rw/rw.hpp>
#include <rw/common.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/FKRange.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/invkin/InvKinSolver.hpp>

#include <rw/loaders/path/PathLoader.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rw/proximity/CollisionStrategy.hpp>

#include "rx60controller/command.h"
#include "wsg_50_common/Move.h"
#include "wsg_50_common/Status.h"

#include "std_msgs/Float32MultiArray.h"

#include "rsd_group1/Num.h"

#include "../../shared.hpp"

/*****************************************************************************
** Defines
*****************************************************************************/



/*****************************************************************************
** Namespaces
*****************************************************************************/

using namespace rwlibs::proximitystrategies;
using namespace rwlibs::pathplanners;
using namespace rw::trajectory;
using namespace rw::pathplanning;
using namespace rwlibs::pathoptimization;
using namespace rw::proximity;
using namespace rw::common;
using namespace rw::models;
using namespace rw::loaders;
using namespace rw::kinematics;
using namespace rw::math;

using namespace std;

namespace robot {

struct Q_deg2rad{
    std::vector< double > original_joints;
    std::vector< double > converted_joints;

    Q_deg2rad(){}

    Q_deg2rad(double J1,double J2,double J3,double J4,double J5,double J6){
        original_joints.clear();
        original_joints.push_back(J1);
        original_joints.push_back(J2);
        original_joints.push_back(J3);
        original_joints.push_back(J4);
        original_joints.push_back(J5);
        original_joints.push_back(J6);
        converted_joints.clear();
        converted_joints.push_back(Deg2Rad*J1);
        converted_joints.push_back(Deg2Rad*J2);
        converted_joints.push_back(Deg2Rad*J3);
        converted_joints.push_back(Deg2Rad*J4);
        converted_joints.push_back(Deg2Rad*J5);
        converted_joints.push_back(Deg2Rad*J6);
        //joints = new Q(6, Deg2Rad*J1, Deg2Rad*J2, Deg2Rad*J3, Deg2Rad*J4, Deg2Rad*J5, Deg2Rad*J6);
    }

    void set_Q_deg(double J1,double J2,double J3,double J4,double J5,double J6){
        original_joints.clear();
        original_joints.push_back(J1);
        original_joints.push_back(J2);
        original_joints.push_back(J3);
        original_joints.push_back(J4);
        original_joints.push_back(J5);
        original_joints.push_back(J6);
        converted_joints.clear();
        converted_joints.push_back(Deg2Rad*J1);
        converted_joints.push_back(Deg2Rad*J2);
        converted_joints.push_back(Deg2Rad*J3);
        converted_joints.push_back(Deg2Rad*J4);
        converted_joints.push_back(Deg2Rad*J5);
        converted_joints.push_back(Deg2Rad*J6);
    //joints = new Q(6, Deg2Rad*J1, Deg2Rad*J2, Deg2Rad*J3, Deg2Rad*J4, Deg2Rad*J5, Deg2Rad*J6);
    }

    Q get_Q_rad(){
        const std::vector< double > new_joint = converted_joints;
        return Q(new_joint);
    }
};

struct Q_rad2deg{
    std::vector< double > original_joints;
    std::vector< double > converted_joints;

    Q_rad2deg(){}

    Q_rad2deg(double J1,double J2,double J3,double J4,double J5,double J6){
    original_joints.clear();
    original_joints.push_back(J1);
    original_joints.push_back(J2);
    original_joints.push_back(J3);
    original_joints.push_back(J4);
    original_joints.push_back(J5);
    original_joints.push_back(J6);
    converted_joints.clear();
    converted_joints.push_back(Rad2Deg*J1);
    converted_joints.push_back(Rad2Deg*J2);
    converted_joints.push_back(Rad2Deg*J3);
    converted_joints.push_back(Rad2Deg*J4);
    converted_joints.push_back(Rad2Deg*J5);
    converted_joints.push_back(Rad2Deg*J6);
    }

    void set_Q_rad(double J1,double J2,double J3,double J4,double J5,double J6){
        original_joints.clear();
        original_joints.push_back(J1);
        original_joints.push_back(J2);
        original_joints.push_back(J3);
        original_joints.push_back(J4);
        original_joints.push_back(J5);
        original_joints.push_back(J6);
        converted_joints.clear();
        converted_joints.push_back(Rad2Deg*J1);
        converted_joints.push_back(Rad2Deg*J2);
        converted_joints.push_back(Rad2Deg*J3);
        converted_joints.push_back(Rad2Deg*J4);
        converted_joints.push_back(Rad2Deg*J5);
        converted_joints.push_back(Rad2Deg*J6);
    }

    Q get_Q_deg(){
        const std::vector< double > new_joint = converted_joints;
        return Q(new_joint);
    }
};

struct pose{
    double x_pos;
    double y_pos;
    double z_pos;
    double orientation;
};

/*****************************************************************************
** Class
*****************************************************************************/

class robot{

    public:
        robot(ros::ServiceClient new_robot_client, ros::ServiceClient new_client_grasp, ros::ServiceClient new_client_release);
        virtual ~robot();
        int graspBrick(double x, double y, double rot);
        void initialize();
        std_msgs::Float32MultiArray get_pub_pose();

        void goToMiddlePos();
        void goToZero();
        void middleToBox();
        void boxToMiddle();
        int goToUpperPos();
        int goToBottomPos();
        int brickToMiddle();

        float wsg_width;
        float wsg_force;

        std::vector<rsd_group1::Num> orderlist;

        Q_deg2rad get_current_joints();

        void grasp();
        void release();

        double next_brick[3];


    private:
        Q_deg2rad boxQ;
        Q_deg2rad middleQ;
        Q_deg2rad zeroQ;



        ros::Subscriber gripper_sub;

        Path<Q> MiddleToBoxPath;
        WorkCell::Ptr currentWorkCell;
        Device::Ptr RobotDevice;
        State current_state;
        Frame::Ptr gripper_frame;
        Frame::Ptr world_frame;

        ros::ServiceClient robot_client;
        ros::ServiceClient gripper_client_grasp;
        wsg_50_common::Move srv_grasp;
        ros::ServiceClient gripper_client_release;
        wsg_50_common::Move srv_release;
        wsg_50_common::Status srv_status;
        rx60controller::commandRequest cmdReq;
        rx60controller::commandResponse cmdRes;

        //private functions

        QToQPlanner::Ptr getPlanner(CollisionDetector* collisionDetector);
        Path<Q> OptimizedRRT(Q Q1, Q Q2);

        pose getPose();
        void setPose(pose grip);

        void set_robot_config(Q_rad2deg config);



        ros::Rate *loop_rate;

    };

}  // namespace HMI



#endif // robot_HPP
