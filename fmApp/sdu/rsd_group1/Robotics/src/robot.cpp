#include "robot.hpp"

namespace robot {

    robot::robot(ros::ServiceClient new_robot_client, ros::ServiceClient new_client_grasp, ros::ServiceClient new_client_release){
        boxQ.set_Q_deg(88,-53,-55,-4,-73,2);
        middleQ.set_Q_deg(130.7,-30.84,-91.58,-5.18,-59.36,43.35);
        zeroQ.set_Q_deg(0,0,0,0,0,0);

        currentWorkCell = WorkCellLoader::Factory::load("/home/robot/StaubliRX60_Scene/SceneStaubliRX60.wc.xml");
        RobotDevice = currentWorkCell->findDevice("StaubliRX60");
        current_state = currentWorkCell->getDefaultState();



        MiddleToBoxPath = OptimizedRRT(middleQ.get_Q_rad(),boxQ.get_Q_rad());

        robot_client = new_robot_client;
        gripper_client_grasp = new_client_grasp;
        gripper_client_release = new_client_release;

        loop_rate = new ros::Rate(5);

        next_brick[0] = 0;
        next_brick[1] = 0;
        next_brick[2] = 0;

    }

    robot::~robot(){}

    QToQPlanner::Ptr robot::getPlanner(CollisionDetector* collisionDetector)
    {
        const PlannerConstraint constraint = PlannerConstraint::make(collisionDetector, RobotDevice, current_state);
        return RRTPlanner::makeQToQPlanner(constraint, RobotDevice);
    }

    Path<Q> robot::OptimizedRRT(Q Q1, Q Q2)
    {
        //Initilize path
        Path<Q> path;

        //Collision Detection strategy
        CollisionStrategy::Ptr cdstrategy = ProximityStrategyFactory::makeCollisionStrategy("PQP");
        //if(!cdstrategy)

        CollisionDetector collisionDetector(currentWorkCell, cdstrategy);
        QToQPlanner::Ptr planner = getPlanner(&collisionDetector);

        PlannerConstraint constraint = PlannerConstraint::make(&collisionDetector, RobotDevice, current_state);
        PathLengthOptimizer optimizer(constraint, MetricFactory::makeEuclidean<Q>());
        planner->query(Q1,Q2,path);
        path = optimizer.shortCut(path);

        return path;
    }



    pose robot::getPose(){
        Q_deg2rad currentQ = get_current_joints();

        FKRange forward_kin(currentWorkCell->getWorldFrame(), RobotDevice->getEnd(), current_state);
        RobotDevice->setQ(currentQ.get_Q_rad(),current_state);
        Transform3D<> current_pos = forward_kin.get(current_state);
        RPY<> current_RPY(current_pos.R());
        pose grip_pose;
        grip_pose.x_pos = current_pos.P()[0];
        grip_pose.y_pos = current_pos.P()[1];
        grip_pose.z_pos = current_pos.P()[2];
        grip_pose.orientation = current_RPY[0];

        cout << "Q temp: " << currentQ.get_Q_rad() << endl;
        cout << "current pos:" << endl << current_pos << endl << current_RPY << endl << endl;

        return grip_pose;
    }

    void robot::setPose(pose grip){
        Vector3D<> new_pos(grip.x_pos,grip.y_pos,grip.z_pos);
        RPY<> new_RPY(grip.orientation,-0.078,-3.113);
        Transform3D<> new_transform(new_pos,new_RPY.toRotation3D());

        cout << " new pose " << new_pos << endl << new_RPY << endl << new_transform << endl;

        Vector3D<> world_pos(-0.6,-0.35,-0.8);
        RPY<> world_RPY(-3.142,0,0);
        Transform3D<> world_transform(world_pos,world_RPY.toRotation3D());

        Transform3D<> actual_world_transform = world_transform*new_transform;

        rw::invkin::JacobianIKSolver solver(RobotDevice, current_state);
        std::vector<Q> solutions = solver.solve(actual_world_transform, current_state);

         cout<< "new pos:" << endl << actual_world_transform.P() << endl << RPY<>(actual_world_transform.R()) << endl << endl;
        if(solutions.size() != 0)
        {
            Q_rad2deg config(solutions[0][0],solutions[0][1],solutions[0][2],solutions[0][3],solutions[0][4],solutions[0][5]);
            set_robot_config(config);
            cout << "solution " << solutions[0] << endl;
        }
        else
            cout << "no solution " << endl;
    }

    void robot::set_robot_config(Q_rad2deg config)
    {
        Q temp = config.get_Q_deg();

        cmdReq.command_number = rx60controller::commandRequest::SET_JOINT_CONFIGURATION;
        cmdReq.joint1 = temp[0];
        cmdReq.joint2 = temp[1];
        cmdReq.joint3 = temp[2];
        cmdReq.joint4 = temp[3];
        cmdReq.joint5 = temp[4];
        cmdReq.joint6 = temp[5];

        cout << cmdReq.joint1 << ", " << cmdReq.joint2 << ", " << cmdReq.joint3 << ", " << cmdReq.joint4 << ", " << cmdReq.joint5 << ", " << cmdReq.joint6 << endl;
        robot_client.call(cmdReq,cmdRes);
    }


    #define OUT_OF_BOUNDS 0
    #define BRICK_DELIVERED 1
    #define NO_SOLUTIONS 2

    void robot::middleToBox(){

        for (int i = 0; i < (int)MiddleToBoxPath.size(); i++)
        {
            Q_rad2deg config(MiddleToBoxPath.data()[i][0],MiddleToBoxPath.data()[i][1],MiddleToBoxPath.data()[i][2],MiddleToBoxPath.data()[i][3],MiddleToBoxPath.data()[i][4],MiddleToBoxPath.data()[i][5]);
            set_robot_config(config);
        }
    }

    void robot::boxToMiddle(){
        for (int i = (int)MiddleToBoxPath.size()-1; i >= 0; i--)
        {
            Q_rad2deg config(MiddleToBoxPath.data()[i][0],MiddleToBoxPath.data()[i][1],MiddleToBoxPath.data()[i][2],MiddleToBoxPath.data()[i][3],MiddleToBoxPath.data()[i][4],MiddleToBoxPath.data()[i][5]);
            set_robot_config(config);
        }
    }

    int robot::brickToMiddle()
    {
        double x = next_brick[0];
        double y = next_brick[1];
        double rot = next_brick[2];
        Q_deg2rad currentQ = get_current_joints();

        double z_up = (0.02/0.29)*(y-0.2) + 0.345;
        if((x >= -0.3 && x <= -0.15) && (y >= 0.2 && y <= 0.49))
        {
            Vector3D<> new_pos(x,y,z_up);
            RPY<> new_RPY(rot,-0.078,-3.113);
            Transform3D<> new_transform(new_pos,new_RPY.toRotation3D());

            cout << " new pose " << new_pos << endl << new_RPY << endl << new_transform << endl;

            rw::invkin::JacobianIKSolver solver(RobotDevice, current_state);
            std::vector<Q> solutions = solver.solve(new_transform, current_state);

            if(solutions.size() != 0)
            {

                Path<Q> CurrentToBrickPath = OptimizedRRT(currentQ.get_Q_rad(),solutions[0]);
                Q_rad2deg config;
                for (int i = 0; i < (int)CurrentToBrickPath.size(); i++)
                {
                    config.set_Q_rad(CurrentToBrickPath.data()[i][0],CurrentToBrickPath.data()[i][1],CurrentToBrickPath.data()[i][2],CurrentToBrickPath.data()[i][3],CurrentToBrickPath.data()[i][4],CurrentToBrickPath.data()[i][5]);
                    set_robot_config(config);
                }

                Path<Q> toMiddlePath = OptimizedRRT(CurrentToBrickPath.data()[(int)CurrentToBrickPath.size()-1],middleQ.get_Q_rad());

                for (int i = 0; i < (int)toMiddlePath.size(); i++)
                {
                    Q_rad2deg config(double(toMiddlePath.data()[i][0]),double(toMiddlePath.data()[i][1]),double(toMiddlePath.data()[i][2]),double(toMiddlePath.data()[i][3]),double(toMiddlePath.data()[i][4]),double(toMiddlePath.data()[i][5]));
                    set_robot_config(config);
                }

                RobotDevice->setQ(middleQ.get_Q_rad(),current_state);

            }
            cout << "No solutions" << endl;
            return NO_SOLUTIONS;
        }
        else
        {
            cout << "Out of bounds" << endl;
            return OUT_OF_BOUNDS;
        }
    }

    int robot::goToUpperPos()
    {
        double x = next_brick[0];
        double y = next_brick[1];
        double rot = next_brick[2];
        Q_deg2rad currentQ = get_current_joints();

        double z_up = (0.02/0.29)*(y-0.2) + 0.345;
        if((x >= -0.3 && x <= -0.15) && (y >= 0.2 && y <= 0.49))
        {
            Vector3D<> new_pos(x,y,z_up);
            RPY<> new_RPY(rot,-0.078,-3.113);
            Transform3D<> new_transform(new_pos,new_RPY.toRotation3D());

            cout << " new pose " << new_pos << endl << new_RPY << endl << new_transform << endl;

            rw::invkin::JacobianIKSolver solver(RobotDevice, current_state);
            std::vector<Q> solutions = solver.solve(new_transform, current_state);

            if(solutions.size() != 0)
            {

                Path<Q> CurrentToBrickPath = OptimizedRRT(currentQ.get_Q_rad(),solutions[0]);
                Q_rad2deg config;
                for (int i = 0; i < (int)CurrentToBrickPath.size(); i++)
                {
                    config.set_Q_rad(CurrentToBrickPath.data()[i][0],CurrentToBrickPath.data()[i][1],CurrentToBrickPath.data()[i][2],CurrentToBrickPath.data()[i][3],CurrentToBrickPath.data()[i][4],CurrentToBrickPath.data()[i][5]);
                    set_robot_config(config);
                }

            }
            cout << "No solutions" << endl;
            return NO_SOLUTIONS;
        }
        else
        {
            cout << "Out of bounds" << endl;
            return OUT_OF_BOUNDS;
        }
    }

    int robot::goToBottomPos()
    {
        double x = next_brick[0];
        double y = next_brick[1];
        double rot = next_brick[2];
        Q_deg2rad currentQ = get_current_joints();

        double z_down = (0.02/0.29)*(y-0.2) + 0.31 -(0.005/0.15)*(x+0.3);
        if((x >= -0.3 && x <= -0.15) && (y >= 0.2 && y <= 0.49))
        {
            Vector3D<> new_pos(x,y,z_down);
            RPY<> new_RPY(rot,-0.078,-3.113);
            Transform3D<> new_transform(new_pos,new_RPY.toRotation3D());

            cout << " new pose " << new_pos << endl << new_RPY << endl << new_transform << endl;

            rw::invkin::JacobianIKSolver solver(RobotDevice, current_state);
            std::vector<Q> solutions = solver.solve(new_transform, current_state);

            if(solutions.size() != 0)
            {

                Path<Q> CurrentToBrickPath = OptimizedRRT(currentQ.get_Q_rad(),solutions[0]);
                Q_rad2deg config;
                for (int i = 0; i < (int)CurrentToBrickPath.size(); i++)
                {
                    config.set_Q_rad(CurrentToBrickPath.data()[i][0],CurrentToBrickPath.data()[i][1],CurrentToBrickPath.data()[i][2],CurrentToBrickPath.data()[i][3],CurrentToBrickPath.data()[i][4],CurrentToBrickPath.data()[i][5]);
                    set_robot_config(config);
                }

            }
            cout << "No solutions" << endl;
            return NO_SOLUTIONS;
        }
        else
        {
            cout << "Out of bounds" << endl;
            return OUT_OF_BOUNDS;
        }
    }

//    int robot::graspBrick(double x, double y, double rot)
//    {
//        Q_deg2rad currentQ = get_current_joints();

//        Path<Q> toMiddlePath = OptimizedRRT(currentQ.get_Q_rad(),middleQ.get_Q_rad());

//        for (int i = 0; i < (int)toMiddlePath.size(); i++)
//        {
//            Q_rad2deg config(double(toMiddlePath.data()[i][0]),double(toMiddlePath.data()[i][1]),double(toMiddlePath.data()[i][2]),double(toMiddlePath.data()[i][3]),double(toMiddlePath.data()[i][4]),double(toMiddlePath.data()[i][5]));
//            set_robot_config(config);
//        }

//        RobotDevice->setQ(middleQ.get_Q_rad(),current_state);

//        double z_up = (0.02/0.29)*(y-0.2) + 0.345;
//        double z_down = (0.02/0.29)*(y-0.2) + 0.31 -(0.005/0.15)*(x+0.3);
//        if((x >= -0.3 && x <= -0.15) && (y >= 0.2 && y <= 0.49))
//        {
//            Vector3D<> new_pos(x,y,z_up);
//            RPY<> new_RPY(rot,-0.078,-3.113);
//            Transform3D<> new_transform(new_pos,new_RPY.toRotation3D());

//            cout << " new pose " << new_pos << endl << new_RPY << endl << new_transform << endl;

//            rw::invkin::JacobianIKSolver solver(RobotDevice, current_state);
//            std::vector<Q> solutions = solver.solve(new_transform, current_state);

//            if(solutions.size() != 0)
//            {

//                Path<Q> MiddleToBrickPath = OptimizedRRT(currentQ.get_Q_rad(),solutions[0]);
//                Q_rad2deg config;
//                for (int i = 0; i < (int)MiddleToBrickPath.size(); i++)
//                {
//                    config.set_Q_rad(MiddleToBrickPath.data()[i][0],MiddleToBrickPath.data()[i][1],MiddleToBrickPath.data()[i][2],MiddleToBrickPath.data()[i][3],MiddleToBrickPath.data()[i][4],MiddleToBrickPath.data()[i][5]);
//                    set_robot_config(config);
//                }

//                Vector3D<> new_pos2(x,y,z_down);
//                RPY<> new_RPY2(rot,-0.078,-3.113);
//                Transform3D<> new_transform2(new_pos2,new_RPY2.toRotation3D());

//                cout << " new pose " << new_pos2 << endl << new_RPY2 << endl << new_transform2 << endl;

//                std::vector<Q> solutions2 = solver.solve(new_transform2, current_state);

//                if(solutions2.size() != 0)
//                {
//                    Q_rad2deg config1(solutions2[0][0],solutions2[0][1],solutions2[0][2],solutions2[0][3],solutions2[0][4],solutions2[0][5]);
//                    set_robot_config(config1);
//                }

//                grasp();

//                for (int i = (int)MiddleToBrickPath.size()-1; i >= 0; i--)
//                {
//                    config.set_Q_rad(MiddleToBrickPath.data()[i][0],MiddleToBrickPath.data()[i][1],MiddleToBrickPath.data()[i][2],MiddleToBrickPath.data()[i][3],MiddleToBrickPath.data()[i][4],MiddleToBrickPath.data()[i][5]);
//                    set_robot_config(config);
//                }
//            }
//            cout << "No solutions" << endl;
//            return NO_SOLUTIONS;
//        }
//        else
//        {
//            cout << "Out of bounds" << endl;
//            return OUT_OF_BOUNDS;
//        }
//    }

    void robot::grasp()
    {

        loop_rate->sleep();
        srv_grasp.request.width = 25.0;
        srv_grasp.request.speed = 400.0;
        gripper_client_grasp.call(srv_grasp);



    }

    void robot::release()
    {
        srv_release.request.width = 75.0;
        srv_release.request.speed = 400.0;
        gripper_client_release.call(srv_release);

        loop_rate->sleep();

    }

    Q_deg2rad robot::get_current_joints(){
        cmdReq.command_number = rx60controller::commandRequest::GET_JOINT_CONFIGURATION;
        robot_client.call(cmdReq,cmdRes);
        Q_deg2rad currentQ(cmdRes.joint1,cmdRes.joint2,cmdRes.joint3,cmdRes.joint4,cmdRes.joint5,cmdRes.joint6);

        return currentQ;
    }

    void robot::initialize(){

        Q currentQ = get_current_joints().get_Q_rad();

        Path<Q> toMiddlePath = OptimizedRRT(currentQ,middleQ.get_Q_rad());

        for (int i = 0; i < (int)toMiddlePath.size(); i++)
        {
            Q_rad2deg config(toMiddlePath.data()[i][0],toMiddlePath.data()[i][1],toMiddlePath.data()[i][2],toMiddlePath.data()[i][3],toMiddlePath.data()[i][4],toMiddlePath.data()[i][5]);
            set_robot_config(config);
        }

        RobotDevice->setQ(middleQ.get_Q_rad(),current_state);

    }

    void robot::goToMiddlePos(){
        Q currentQ = get_current_joints().get_Q_rad();
        Path<Q> toMiddlePath = OptimizedRRT(currentQ,middleQ.get_Q_rad());

        for (int i = 0; i < (int)toMiddlePath.size(); i++)
        {
            Q_rad2deg config(toMiddlePath.data()[i][0],toMiddlePath.data()[i][1],toMiddlePath.data()[i][2],toMiddlePath.data()[i][3],toMiddlePath.data()[i][4],toMiddlePath.data()[i][5]);
            set_robot_config(config);
        }

        RobotDevice->setQ(middleQ.get_Q_rad(),current_state);
    }



    void robot::goToZero(){
        Q currentQ = get_current_joints().get_Q_rad();
        Path<Q> toZeroPath = OptimizedRRT(currentQ,zeroQ.get_Q_rad());

        for (int i = 0; i < (int)toZeroPath.size(); i++)
        {
            Q_rad2deg config(toZeroPath.data()[i][0],toZeroPath.data()[i][1],toZeroPath.data()[i][2],toZeroPath.data()[i][3],toZeroPath.data()[i][4],toZeroPath.data()[i][5]);
            set_robot_config(config);
        }

        RobotDevice->setQ(zeroQ.get_Q_rad(),current_state);
    }

    std_msgs::Float32MultiArray robot::get_pub_pose(){

            //double *config = new double[6];
            double *pose = new double[6];

            Q_deg2rad currentQ = get_current_joints();

            FKRange forward_kin(RobotDevice->getBase(), RobotDevice->getEnd(), current_state);
            RobotDevice->setQ(currentQ.get_Q_rad(),current_state);
            Transform3D<> current_pos = forward_kin.get(current_state);
            RPY<> current_RPY(current_pos.R());

            pose[0] = current_pos.P()[0];
            pose[1] = current_pos.P()[1];
            pose[2] = current_pos.P()[2];
            pose[3] = current_RPY[0];
            pose[4] = current_RPY[0];
            pose[5] = current_RPY[0];

            std_msgs::Float32MultiArray message;
            for(int it = 0; it < 6; it++)
            {
                message.data.push_back((float)currentQ.original_joints[it]);
            }
            for(int it = 0; it < 6; it++)
            {
                message.data.push_back(pose[it]);
            }
            for(int it = 0; it < 3; it++)
            {
                message.data.push_back((float)next_brick[it]);
            }
            return message;
    }

}
