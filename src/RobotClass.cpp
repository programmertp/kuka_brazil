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



void RobotClass::moveTo(geometry_msgs::PoseStamped goal){
    std::cout << LOG_NAME << ": Sending goal for moveTo..." << std::endl;
    move_base_msgs::MoveBaseGoal mb_goal;
    mb_goal.target_pose = goal;
    mb_client.sendGoal(mb_goal, boost::bind(&RobotClass::requestDone, this, _1, _2),
                      boost::bind(&RobotClass::requestBecameActive, this),
                      boost::bind(&RobotClass::requestFeedback, this, _1));
    std::cout << LOG_NAME << ": Goal for moveTo was sent" << std::endl;

    mb_client.waitForResult();
}



void RobotClass::rotateInPlace(double angle){
    std::cout << LOG_NAME << ": Sending goal for rotateInPlace..." << std::endl;
    move_base_msgs::MoveBaseGoal mb_goal;
    mb_goal.target_pose.header.stamp = ros::Time::now();
    mb_goal.target_pose.header.frame_id = "base_link";
    mb_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

    mb_client.sendGoal(mb_goal, boost::bind(&RobotClass::requestDone, this, _1, _2),
                       boost::bind(&RobotClass::requestBecameActive, this),
                       boost::bind(&RobotClass::requestFeedback, this, _1));
    std::cout << LOG_NAME << ": Goal for rotateInPlace was sent" << std::endl;

    mb_client.waitForResult();
}



void RobotClass::requestBecameActive(){
    std::cout << LOG_NAME << ": MoveBase server start processing of the goal" << std::endl;
}



void RobotClass::requestFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){
    cur_action_pose = feedback->base_position;
}



void RobotClass::requestDone(const actionlib::SimpleClientGoalState& state,
                             const move_base_msgs::MoveBaseResultConstPtr& result){
    std::cout << LOG_NAME << ": MoveBase server finished processing of the goal in state " << state.toString() << std::endl;

    geometry_msgs::Twist null_velocity;
    for(int i = 0; i < 3; i++)
            vel_publisher.publish(null_velocity);
}
