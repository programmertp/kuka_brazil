#include <AutomappingNode.h>


AutomappingNode::AutomappingNode() : nh_for_params("~"){
    ROS_INFO_STREAM("Initialization...");
    nh_for_params.param<double>("safe_distance", safe_distance, 0.5);
    nh_for_params.param<double>("tolerance", tolerance, 0.1);
    nh_for_params.param<std::string>("map_name", map_name, "~/map");
    nh_for_params.param<std::string>("pose_params_file_name", pose_params_file_name, "~/init_pose.yaml");

    goal_requester = nh.serviceClient<kuka_brazil_msgs::GetGoal>("get_goal_at_map");
    pathfinder = nh.serviceClient<nav_msgs::GetPlan>("move_base_node/make_plan");
    ROS_INFO_STREAM("Initialization done");
}



AutomappingNode::~AutomappingNode(){
    goal_requester.shutdown();
    pathfinder.shutdown();
}



double AutomappingNode::distanceBetweenPoses(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2){
    double distance = std::pow(pose1.pose.position.x - pose2.pose.position.x, 2) +
                      std::pow(pose1.pose.position.y - pose2.pose.position.y, 2);
    return std::sqrt(distance);
}



geometry_msgs::PoseStamped AutomappingNode::safeGoalInPlan(const nav_msgs::Path &plan){
    int last = plan.poses.size() - 1;
    for(int i = last-1; i > 0; i--)
        if(distanceBetweenPoses(plan.poses[i], plan.poses[last]) >= safe_distance)
            return plan.poses[i];
    return plan.poses[0];
}



void AutomappingNode::doYourWork(){

    ROS_INFO_STREAM("Run initial rotations...");
    for(int i = 0; i < 3; i++)
        robot.rotateInPlace( 2*M_PI/3);
    for(int i = 0; i < 3; i++)
        robot.rotateInPlace(-2*M_PI/3);
    ROS_INFO_STREAM("Initial rotations done");


    std::string status;
    kuka_brazil_msgs::GetGoal get_goal_srv;
    geometry_msgs::PoseStamped curr_robot_pose;
    nav_msgs::GetPlan get_plan_srv;
    get_plan_srv.request.goal.header.frame_id = "map";
    get_plan_srv.request.tolerance = tolerance;

    while(ros::ok()){

        ROS_INFO_STREAM("Find robot's pose...");
        try{
            curr_robot_pose = robot.currentPose("map");
        }
        catch (tf::TransformException ex){
            ROS_ERROR("Unable to find robot's pose! Error is %s", ex.what());
            continue;
        }
        ROS_INFO_STREAM("Robot's pose found");


        get_goal_srv.request.robot_pose = curr_robot_pose.pose.position;
        ROS_INFO_STREAM("Call " << goal_requester.getService() << " service for getting new goal...");
        if( !goal_requester.call(get_goal_srv) ){
            ROS_ERROR_STREAM("Unable to call " << goal_requester.getService() << " service!");
            ros::shutdown();
        }
        if(EQUAL(get_goal_srv.response.goal, curr_robot_pose.pose.position)){
            ROS_INFO_STREAM("No new goal received. Map of the environment done");
            break;
        }
        ROS_INFO_STREAM("Goal from " << goal_requester.getService() << " service obtained");


        get_plan_srv.request.start = curr_robot_pose;
        get_plan_srv.request.goal.pose.position = get_goal_srv.response.goal;
        ROS_INFO_STREAM("Call " << pathfinder.getService() << " service for path...");
        if( !pathfinder.call(get_plan_srv) ){
            ROS_ERROR_STREAM("Unable to call " << pathfinder.getService() << " service!");
            ros::shutdown();
        }
        if(get_plan_srv.response.plan.poses.size() == 0){
            ROS_INFO_STREAM("No path received");
            get_goal_srv.request.excluded_goals.push_back(get_goal_srv.response.goal);
            ROS_INFO_STREAM("Current goal marked as unachievable");
            continue;
        }
        ROS_INFO_STREAM("Path from " << pathfinder.getService() << " service obtained");


        ROS_INFO_STREAM("Find safe goal in path and give it to move_base");
        status = robot.moveTo(safeGoalInPlan(get_plan_srv.response.plan));
        ROS_INFO_STREAM("Move_base end his work with status " << status);
        get_goal_srv.request.excluded_goals.clear();
        if(status == "ABORTED"){
            get_goal_srv.request.excluded_goals.push_back(get_goal_srv.response.goal);
            ROS_INFO_STREAM("Current goal marked as unachievable");
        }
    }

    ROS_INFO_STREAM("Saving map...");
    map_name = "rosrun map_server map_saver -f " + map_name;
    std::system(map_name.c_str());
    ROS_INFO_STREAM("Map was saved");


    ROS_INFO_STREAM("Saving information about robot's pose...");
    std::ofstream fout(pose_params_file_name.c_str());
    if(fout.is_open()){
        fout << "initial_pose_x:" << curr_robot_pose.pose.position.x << std::endl;
        fout << "initial_pose_y:" << curr_robot_pose.pose.position.y << std::endl;
        fout << "initial_pose_a:" << tf::getYaw(curr_robot_pose.pose.orientation);
        ROS_INFO_STREAM("Information about robot's pose was saved");
    }
    else
        ROS_ERROR_STREAM("Unable to open/create \"" << pose_params_file_name << "\" file!");
    fout.close();

    ROS_INFO_STREAM("Shutdowning...");
}
