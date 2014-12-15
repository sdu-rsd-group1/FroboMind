#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <rw/rw.hpp>

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
//For optimization
#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
//USE_ROBWORK_NAMESPACE
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

QToQPlanner::Ptr getPlanner(CollisionDetector* collisionDetector)
{
	WorkCell::Ptr currentWorkCell = WorkCellLoader::Factory::load("/home/anoch/Dropbox/RX60/StaubliRX60_Scene/SceneStaubliRX60.wc.xml");
	Device::Ptr RobotDevice = currentWorkCell->findDevice("StaubliRX60");
	const State& state = currentWorkCell->getDefaultState();
	RobotDevice = currentWorkCell->findDevice("StaubliRX60");
	const PlannerConstraint constraint = PlannerConstraint::make(collisionDetector, RobotDevice, state);

	return RRTPlanner::makeQToQPlanner(constraint, RobotDevice);
}
Path<Q> OptimizedRRT(Q Q1, Q Q2)
{
	WorkCell::Ptr currentWorkCell = WorkCellLoader::Factory::load("/home/anoch/Dropbox/RX60/StaubliRX60_Scene/SceneStaubliRX60.wc.xml");
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

int main(int argc, char **argv)
{
	WorkCell::Ptr currentWorkCell = WorkCellLoader::Factory::load("/home/anoch/Dropbox/RX60/StaubliRX60_Scene/SceneStaubliRX60.wc.xml");	
	Device::Ptr RobotDevice = currentWorkCell->findDevice("StaubliRX60");
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Rate loop_rate(10);
	//int count = 0;
	Path<Q> path;
	while (ros::ok())
	{
		int c = getchar();
		if( c == 'a')
		{
			Q currentQ(6,0,0,0,0,0,0);
			Q PickUpQ2(6, -3.142, 1.833, 0, 0.109, 0.610, -0.286);
			State currentState = currentWorkCell->getDefaultState();
			RobotDevice->setQ(PickUpQ2, currentState);
			path = OptimizedRRT(currentQ,PickUpQ2);
			ROS_INFO("Sending PickUp Configurations");
			for (int i = 0; i < (int)path.size()-1; i++)
			{

				//print path info
				//std::cout << path.data()[i] << "\n";
				std_msgs::String msg;

				std::stringstream ss;
				//ss << "hello world " << count;
				ss << path.data()[i][0] << "\t"<< path.data()[i][1] << "\t"<< path.data()[i][2] << "\t"<< path.data()[i][3] << "\t"<< path.data()[i][4] << "\t"<< path.data()[i][5];
				msg.data = ss.str();

				

				chatter_pub.publish(msg);
			}

			ros::spinOnce();

			loop_rate.sleep();
		}
  }


  return 0;
}
