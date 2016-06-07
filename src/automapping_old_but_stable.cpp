#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <cstdlib>

#define DISTANCE_TO_GOAL 0.5

#define PI 3.14159265359

#define FIRST_ANGLE 140
#define SECOND_ANGLE 280
#define THIRD_ANGLE 420

#define NO_ACTIONS 0
#define ROTATING_CLOCKWISE_TO_1 1
#define ROTATING_CLOCKWISE_TO_2 2
#define ROTATING_CLOCKWISE_TO_3 3
#define ROTATING_COUNTERCLOCKWISE_TO_1 4
#define ROTATING_COUNTERCLOCKWISE_TO_2 5
#define ROTATING_COUNTERCLOCKWISE_TO_3 6
#define WAITING_FOR_MAP 7
#define MOVING_TO_GOAL 8

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void locate_target(nav_msgs::OccupancyGrid &grid, geometry_msgs::Pose &robot_pose, move_base_msgs::MoveBaseGoal &a);

class AutomappingNode{

	ros::NodeHandle nh;
	//ros::Subscriber map_reader;
	ros::Publisher stopper;
	MoveBaseClient mbClient;
	ros::ServiceClient map_client;
	tf::TransformListener listener_curr_pose;
	geometry_msgs::PoseStamped currGoal, currRobotPose;
	int status;

	void mapHandler(const nav_msgs::GetMap::Response &map);
	void requestDone(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
	void requestFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

public:
	
	AutomappingNode();
	~AutomappingNode();
	void doYourWork();
	
};



AutomappingNode::AutomappingNode() : mbClient("move_base", true){
	
	status = NO_ACTIONS;
	stopper = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	map_client = nh.serviceClient<nav_msgs::GetMap>("dynamic_map");
	ROS_INFO_STREAM("Initialization: success.");	
   	ROS_INFO_STREAM("Waiting for action move_base server to start.");
    	mbClient.waitForServer();
    	ROS_INFO_STREAM("Action move_base server started.");
}


AutomappingNode::~AutomappingNode(){
	stopper.shutdown();
	map_client.shutdown();
}


void AutomappingNode::mapHandler(const nav_msgs::GetMap::Response &map){	
	ROS_INFO_STREAM("Map received.");	
	if(status == WAITING_FOR_MAP){
		
		geometry_msgs::PoseStamped currRobotPose_in_base_link;
		currRobotPose_in_base_link.header.frame_id = "base_link";
		currRobotPose_in_base_link.header.stamp = ros::Time::now();
		currRobotPose_in_base_link.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	
		try{
      			listener_curr_pose.transformPose("map", currRobotPose_in_base_link, currRobotPose);
    		}
    		catch (tf::TransformException ex){
      			ROS_ERROR("%s",ex.what());
      			ros::Duration(1.0).sleep();
    		}

		nav_msgs::OccupancyGrid map_help = map.map;
		

		move_base_msgs::MoveBaseGoal mbGoal;
		locate_target(map_help, currRobotPose.pose, mbGoal);
		currGoal = mbGoal.target_pose;		
		ROS_INFO_STREAM(mbGoal.target_pose.pose.position << " " <<mbGoal.target_pose.pose.orientation);
		if(currGoal.pose.position.x == currRobotPose.pose.position.x){
			std::system("rosrun map_server map_saver -f auto_map");

			ros::NodeHandle nh_for_param("~");
			nh_for_param.setParam("red_init_x", currRobotPose.pose.position.x);
			nh_for_param.setParam("red_init_y", currRobotPose.pose.position.y);
			nh_for_param.setParam("red_init_angle", tf::getYaw(currRobotPose.pose.orientation));				
			std::system("rosparam dump ~/for_ros/workspace/kuka_brazil/map/init_params.yaml /automapping_node");			

			ros::shutdown();
		}
		
		status = MOVING_TO_GOAL;
		ROS_INFO_STREAM("Sending goal.");
  		mbClient.sendGoal(mbGoal, boost::bind(&AutomappingNode::requestDone, this, _1, _2), 
					MoveBaseClient::SimpleActiveCallback(), 
					boost::bind(&AutomappingNode::requestFeedback, this, _1));
		ROS_INFO_STREAM("Goal was sent.");
	}
}


void AutomappingNode::requestFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){

	double distance_to_goal;
	distance_to_goal = std::sqrt(std::pow(feedback->base_position.pose.position.x - currGoal.pose.position.x,2) + 
				     std::pow(feedback->base_position.pose.position.y - currGoal.pose.position.y,2));		
	ROS_INFO("Distance to goal = %.3f", distance_to_goal);
	if(distance_to_goal < DISTANCE_TO_GOAL){
		mbClient.cancelGoal();
	}
}



void AutomappingNode::requestDone(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result){

	ROS_INFO_STREAM("Request finished in state " << state.toString().c_str());
	geometry_msgs::Twist null_velocity;
	stopper.publish(null_velocity);
	
	move_base_msgs::MoveBaseGoal mbGoal;
	if( (status >= ROTATING_CLOCKWISE_TO_1) && (status<=ROTATING_COUNTERCLOCKWISE_TO_2) ){		
		mbGoal.target_pose.header.stamp = ros::Time::now();
		mbGoal.target_pose.header.frame_id = "odom";
	}

	ROS_INFO_STREAM(status);
	switch(status){
		case ROTATING_CLOCKWISE_TO_1: case ROTATING_CLOCKWISE_TO_3:
			mbGoal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(SECOND_ANGLE * PI / 180);
			status++;
			break;
		case ROTATING_CLOCKWISE_TO_2:
			mbGoal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(THIRD_ANGLE * PI / 180);
			status++;
			break;
		case ROTATING_COUNTERCLOCKWISE_TO_1:
			mbGoal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(FIRST_ANGLE * PI / 180);
			status++;
			break;
		case ROTATING_COUNTERCLOCKWISE_TO_2:
			mbGoal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0 * PI / 180);
			status++;
			break;
		case ROTATING_COUNTERCLOCKWISE_TO_3:
			status = WAITING_FOR_MAP;
			//map_reader = nh.subscribe("map", 1, &AutomappingNode::mapReaderCallback, this);
			break;
		default:
			status = WAITING_FOR_MAP;
			break;
	}

	if( (status >= ROTATING_CLOCKWISE_TO_2) && (status <= ROTATING_COUNTERCLOCKWISE_TO_3) )
		mbClient.sendGoal(mbGoal, boost::bind(&AutomappingNode::requestDone, this, _1, _2), 
					  MoveBaseClient::SimpleActiveCallback(), 
					  MoveBaseClient::SimpleFeedbackCallback());
}


void AutomappingNode::doYourWork(){
	
	status = ROTATING_CLOCKWISE_TO_1;	
	
	move_base_msgs::MoveBaseGoal mbGoal;
	mbGoal.target_pose.header.stamp = ros::Time::now();
	mbGoal.target_pose.header.frame_id = "odom";
	mbGoal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(FIRST_ANGLE * PI / 180);
	ROS_INFO_STREAM("Sending goal.");
	mbClient.sendGoal(mbGoal, boost::bind(&AutomappingNode::requestDone, this, _1, _2), 
					MoveBaseClient::SimpleActiveCallback(), 
					MoveBaseClient::SimpleFeedbackCallback());
	ROS_INFO_STREAM("Goal was sent.");	
	ros::Rate sleeper(10);
	while(ros::ok()){
		if(status == WAITING_FOR_MAP){
			nav_msgs::GetMap map_srv;
			if (map_client.call(map_srv)){
				ROS_INFO_STREAM("Map was taken");
				mapHandler(map_srv.response);
			} 
			else{
				ROS_ERROR("Failed to call service dynamic_map");
			}	
		}
		ros::spinOnce();
		sleeper.sleep();
	}
}



int main (int argc, char **argv){
	ros::init(argc, argv, "automapping_node");
	ROS_INFO_STREAM("Initialization..");
	AutomappingNode node;
	node.doYourWork();
	return 0;
}
