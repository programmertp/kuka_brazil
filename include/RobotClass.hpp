#ifndef _ROBOTCLASS_
#define _ROBOTCLASS_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <iostream>
#include <cmath>

#define LOG_NAME "RobotClass object"

class RobotClass{

    ros::NodeHandle nh;
    ros::Publisher vel_publisher;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mb_client;

public:

    RobotClass();
    ~RobotClass();

    void stop();

    std::string moveTo(geometry_msgs::PoseStamped);

    void rotateInPlace(double angle);
};

#endif //_ROBOTCLASS_
