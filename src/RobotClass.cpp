#include <RobotClass.hpp>

RobotClass::RobotClass() : mb_client("move_base", true){
    vel_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    std::cout << LOG_NAME << ": Initialization success" << std::endl;
    std::cout << LOG_NAME << ": Waiting for action move_base server to start..." << std::endl;
    mb_client.waitForServer();
    std::cout << LOG_NAME << ": Action move_base server started" << std::endl;
}



RobotClass::~RobotClass(){
    vel_publisher.shutdown();
}



std::string RobotClass::moveTo(geometry_msgs::PoseStamped goal){
    std::cout << LOG_NAME << ": Sending goal for moveTo..." << std::endl;
    move_base_msgs::MoveBaseGoal mb_goal;
    mb_goal.target_pose = goal;
    std::string state = mb_client.sendGoalAndWait(mb_goal).toString();
    stop();
    std::cout << LOG_NAME << ": moveTo finished its work" << std::endl;
    return state;
}



void RobotClass::rotateInPlace(double angle){
    std::cout << LOG_NAME << ": Sending goal for rotateInPlace..." << std::endl;
    move_base_msgs::MoveBaseGoal mb_goal;
    mb_goal.target_pose.header.stamp = ros::Time::now();
    mb_goal.target_pose.header.frame_id = "base_link";
    mb_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
    mb_client.sendGoalAndWait(mb_goal);
    stop();
    std::cout << LOG_NAME << ": rotateInPlace finished its work" << std::endl;
}



void RobotClass::stop(){
    geometry_msgs::Twist null_velocity;
    //this 3 from for{} is not used anywhere else
    for(int i = 0; i < 3; i++)
            vel_publisher.publish(null_velocity);
}
