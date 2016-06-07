#ifndef _ROBOTCLASS_RED
#define _ROBOTCLASS_RED

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <string>
#include <iostream>
#include <cmath>

#define ROBOTCLASS_GETTING_GOAL 0
#define ROBOTCLASS_MOVING_TO_GOAL 1
#define ROBOTCLASS_STAYING_IN_PLACE 2

#define LOG_NAME "RobotClass object"

class RobotClass{

	ros::NodeHandle nh;
	ros::Publisher vel_publisher;
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> mbClient;
	geometry_msgs::PoseStamped cur_action_pose, cur_goal;
	double distance;

	void requestDone(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
	void requestBecameActive();
	void requestFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
	void requestFeedbackWithDist(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

public:
		
	int status;	

	RobotClass();
	~RobotClass();
	
	//Robot will stop
	void stop();

	//Robot will move forward for a needed distance. Negative distance will produce reverse movement
	void moveForward(float dist, float koeff_lin = 0.2, float koeff_ang = 0.2, float max_error_lin = 0.05, float max_error_ang = 0.05);
	
	//Robot will move to needed pose without stop the program 
	void moveToBG(geometry_msgs::PoseStamped);

	//Until robot arrives to the point, your program will stop 
	void moveTo(geometry_msgs::PoseStamped, double dist = 0.0);

	//Robot will rotate in place without stop the program
	void rotateInPlace(float angle, std::string in_frame);

	//Until robot rotates to the needed angle, your program will stop 
	void rotateInPlaceBG(float angle, std::string in_frame);
};

#endif 
