#include "roboteq_sdc2130/sdc2130.hpp"
#include "dynamic_reconfigure/server.h"
#include "roboteq_sdc2130/roboteq_sdc2130_dynparamsConfig.h"


int main (int argc, char** argv)
{
	ros::init(argc,argv,"roboteq_controller");
	sdc2130 controller;

	// dynamic reconfigure
	dynamic_reconfigure::Server<roboteq_sdc2130::roboteq_sdc2130_dynparamsConfig> server;
	dynamic_reconfigure::Server<roboteq_sdc2130::roboteq_sdc2130_dynparamsConfig>::CallbackType cb;


	cb = boost::bind(&sdc2130::onDynreconfig,&controller, _1, _2);
	server.setCallback(cb);

	controller.spin();

	return 0;
}
