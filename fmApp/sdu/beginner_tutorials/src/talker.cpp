#include "ros/ros.h"
#include "std_msgs/String.h"


#include <sstream>

#include <rw/rw.hpp>

#include <rw/common.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/invkin/InvKinSolver.hpp>
#include </home/robot/RWPROJECTROOT/RobWork/src/sandbox/invkin/ClosedFormURSolver.hpp>
//For optimization
#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include "std_msgs/Float32MultiArray.h"
//USE_ROBWORK_NAMESPACE

using namespace rw::common;
using namespace rw::models;
using namespace rw::loaders;
using namespace rw::kinematics;

using namespace rwlibs::proximitystrategies;
using namespace rwlibs::pathplanners;
using namespace rw::trajectory;
using namespace rw::pathplanning;

using namespace rwlibs::pathoptimization;
//using namespace rws;
using namespace rw::math;
using namespace rw::proximity;
using namespace std;

#define PI 3.1415

rw::invkin::InvKinSolver::Ptr _iksolver;
 State current_state;
 Device::Ptr RobotDevice;

double deg_to_rad(double deg)
{
	return deg*(PI/180.0);
}

double rad_to_deg(double rad)
{
	return rad*(180.0/PI);
}

QToQPlanner::Ptr getPlanner(CollisionDetector* collisionDetector)
{
	WorkCell::Ptr currentWorkCell = WorkCellLoader::Factory::load("/home/robot/StaubliRX60_Scene/SceneStaubliRX60.wc.xml");
	Device::Ptr RobotDevice = currentWorkCell->findDevice("StaubliRX60");
	const State& state = currentWorkCell->getDefaultState();
	RobotDevice = currentWorkCell->findDevice("StaubliRX60");
	const PlannerConstraint constraint = PlannerConstraint::make(collisionDetector, RobotDevice, state);

	return RRTPlanner::makeQToQPlanner(constraint, RobotDevice);
}
Path<Q> OptimizedRRT(Q Q1, Q Q2)
{
	WorkCell::Ptr currentWorkCell = WorkCellLoader::Factory::load("/home/robot/StaubliRX60_Scene/SceneStaubliRX60.wc.xml");
	Device::Ptr RobotDevice = currentWorkCell->findDevice("StaubliRX60");
	//Initilize path
	Path<Q> path;
	
	//Update state
	State currentState = currentWorkCell->getDefaultState();
	//Update the Robot Device
	RobotDevice = currentWorkCell->findDevice("StaubliRX60");
	
	//Collision Detection strategy
	CollisionStrategy::Ptr cdstrategy = ProximityStrategyFactory::makeCollisionStrategy("PQP");
	//if(!cdstrategy)
		
	CollisionDetector collisionDetector(currentWorkCell, cdstrategy);
	QToQPlanner::Ptr planner = getPlanner(&collisionDetector);
	
	PlannerConstraint constraint = PlannerConstraint::make(&collisionDetector, RobotDevice, currentState);
	PathLengthOptimizer optimizer(constraint, MetricFactory::makeEuclidean<Q>());
	planner->query(Q1,Q2,path);
	path = optimizer.shortCut(path);
	
	return path;
}

void inverseKinematics(Transform3D<> target)
{
	//_iksolver = ownedPtr(new ClosedFormURSolver(device,state));
	//std::vector<Q> solutions = _iksolver->solve(target,state);
	rw::invkin::JacobianIKSolver solver(RobotDevice, current_state);
	
	//rw::invkin::IKMetaSolver Msolver(&solver, device);
	////cout << target << endl; //solver.solve(target, state)[0] << endl;
	std::vector<Q> solutions = solver.solve(target, current_state);

	for(int i = 0; i < solutions.size(); i++)
	{
		cout<<"Solution = "<<solutions[i]<<endl;
	}
		cout<<"Whuuut"<<endl;
}

int main(int argc, char **argv)
{
	WorkCell::Ptr currentWorkCell = WorkCellLoader::Factory::load("/home/robot/StaubliRX60_Scene/SceneStaubliRX60.wc.xml");	
	RobotDevice = currentWorkCell->findDevice("StaubliRX60");
	current_state = currentWorkCell->getDefaultState();
	Q temp(6,0,0,0,0,0,0);
	RobotDevice->setQ(temp,current_state);
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("robot_configuration", 1000);
	ros::Rate loop_rate(10);
	//int count = 0;
	Path<Q> path;
	Vector3D<> new_pos(-0.3,-0.64,1.2);
	RPY<> new_RPY(0,0,3);
	Rotation3D<> new_rot = new_RPY.toRotation3D();
	Transform3D<> new_transform(new_pos,new_rot);
	RPY<> new_RPY1(new_rot);
	//Vector3D<> orig_pos1(-0.6,-0.35,0.8);
	//Rotation3D<> orig_rot1(-1,0,0,0,-1,0,0,0,1);
	
	//Vector3D<> orig_pos(0,-0.35,-0.8);
	//Rotation3D<> orig_rot(0,1,0,-1,0,0,0,0,1);
	
	Vector3D<> world_pos(-0.6,-0.35,-0.8);
	RPY<> world_RPY(-3.142,0,0);
	Rotation3D<> world_rot = world_RPY.toRotation3D();
	Transform3D<> world_transform(world_pos,world_rot);
	
	//Transform3D<> orig_transform1(orig_pos1,orig_rot1);
	//Transform3D<> orig_transform(orig_pos,orig_rot);
	
	//RPY<> orig_RPY(orig_rot);
	//RPY<> orig_RPY1(orig_rot);
	
	//Transform3D<> actual_transform1 = new_transform*orig_transform1;
	//RPY<> actual_RPY1(actual_transform1.R());
	//Transform3D<> actual_transform2 = new_transform*orig_transform;
	//RPY<> actual_RPY2(actual_transform2.R());
	
	Transform3D<> actual_world_transform = world_transform*new_transform;
	RPY<> actual_world_RPY(actual_world_transform.R());
	//while (ros::ok())
	//{
		//int c = getchar();
		//if( c == 'a')
		//{
			cout << "new transform" << endl;
			cout << new_transform.P() << endl;
			cout << new_transform.R() << endl;
			cout << new_RPY1 << endl << endl;
			
			//cout << "orig transform" << endl;
			//cout << orig_transform.P() << endl;
			//cout << orig_transform.R() << endl;
			//cout << orig_RPY << endl << endl;
			
			//cout << "orig transform1" << endl;
			//cout << orig_transform1.P() << endl;
			//cout << orig_transform1.R() << endl;
			//cout << orig_RPY1 << endl << endl;
			
			//cout << "actual transform 1" << endl;
			//cout << actual_transform1.P() << endl;
			//cout << actual_transform1.R() << endl;
			//cout << actual_RPY1 << endl << endl;
			
			//cout << "actual transform 2" << endl;
			//cout << actual_transform2.P() << endl;
			//cout << actual_transform2.R() << endl;
			//cout << actual_RPY2 << endl << endl;
			
			cout << "world" << endl;
			cout << world_transform.P() << endl;
			cout << world_transform.R() << endl;
			cout << world_RPY << endl << endl;
			
			cout << "actual world" << endl;
			cout << actual_world_transform.P() << endl;
			cout << actual_world_transform.R() << endl;
			cout << actual_world_RPY << endl << endl;
			
			inverseKinematics(actual_world_transform);
			
			//Q currentQ(6,0,0,0,0,0,0);
			//Q PickUpQ2(6, deg_to_rad(137.2), deg_to_rad(-24), deg_to_rad(-85), deg_to_rad(-2.5), deg_to_rad(-71), deg_to_rad(0));
			//State currentState = currentWorkCell->getDefaultState();
			//RobotDevice->setQ(PickUpQ2, currentState);
			//path = OptimizedRRT(currentQ,PickUpQ2);
			//ROS_INFO("Sending PickUp Configurations");
			//for (int i = 0; i < (int)path.size(); i++)
			//{

				//////print path info
				////std::cout << path.data()[i] << "\n";
				////std_msgs::String msg;

				////std::stringstream ss;
				//////ss << "hello world " << count;
				////ss << (float)rad_to_deg(path.data()[i][0]) << "\t"<< rad_to_deg(path.data()[i][1]) << "\t"<< rad_to_deg(path.data()[i][2]) << "\t"<< rad_to_deg(path.data()[i][3]) << "\t"<< rad_to_deg(path.data()[i][4]) << "\t"<< rad_to_deg(path.data()[i][5]);
				////msg.data = ss.str();

				////chatter_pub.publish(msg);
				
				//std_msgs::Float32MultiArray message;
				//for(int it = 0; it < 6; it++)
				//{
					//message.data.push_back((float)rad_to_deg(path.data()[i][it]));
				//}
				//chatter_pub.publish(message);
			//}

		//}
		//ros::spinOnce();

		//loop_rate.sleep();
  //}


  return 0;
}
