#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <RobotClass.hpp>
#include <tf/transform_listener.h>
#include <cstdlib>

#define PI 3.14159265359
#define LOGNAME "automapping_node"
#define NEEDED_DIST 0.5
#define ANGLE 140

#define WAITING_GOAL 0
#define ROBOT_MOVING 1
#define WAITING_FOR_ROBOT_MOVING 2


void locate_target(nav_msgs::OccupancyGrid &grid, geometry_msgs::Pose &robot_pose, move_base_msgs::MoveBaseGoal &a);

class AutomappingNode{

	ros::NodeHandle nh;
	RobotClass robot;	
	ros::ServiceClient map_client;
	move_base_msgs::MoveBaseGoal curr_goal;
	tf::TransformListener listener_curr_pose;
	int status;

	void mapHandler(const nav_msgs::GetMap::Response &map);

public:
	
	AutomappingNode();
	~AutomappingNode();
	void doYourWork();
	
};



AutomappingNode::AutomappingNode(){
	
	map_client = nh.serviceClient<nav_msgs::GetMap>("dynamic_map");
	std::cout << LOGNAME <<  ": Initialization: success." << std::endl;	
}


AutomappingNode::~AutomappingNode(){
	map_client.shutdown();
}


void AutomappingNode::mapHandler(const nav_msgs::GetMap::Response &map){	

	std::cout << LOGNAME << ": Start processing of the map." << std::endl; 

	geometry_msgs::PoseStamped currRobotPose_in_base_link, currRobotPose;
	currRobotPose_in_base_link.header.frame_id = "/base_link";
	currRobotPose_in_base_link.header.stamp = ros::Time::now();
	currRobotPose_in_base_link.pose.orientation = tf::createQuaternionMsgFromYaw(0);

	bool flag = true;
	while(flag){
		try{	
			flag = false;
			listener_curr_pose.transformPose("/map", currRobotPose_in_base_link, currRobotPose);
		}
		catch (tf::TransformException ex){
			flag = true;
			ROS_ERROR("%s: %s",LOGNAME, ex.what());
			ros::Duration(1.0).sleep();
		}
	}

	nav_msgs::OccupancyGrid map_help = map.map;

	locate_target(map_help, currRobotPose.pose, curr_goal);
	std::cout << LOGNAME << ": locate_target() returned: x = " << curr_goal.target_pose.pose.position.x << ", y = "
			<< curr_goal.target_pose.pose.position.y << std::endl; 
	
	if(curr_goal.target_pose.pose.position.x == currRobotPose.pose.position.x){
		std::cout << LOGNAME << ": Finish work..." << std::endl;
		std::system("rosrun map_server map_saver -f auto_map");
		
		std::cout << LOGNAME << ": Map was saved." << std::endl;		

		ros::NodeHandle nh_for_param("~");
		nh_for_param.setParam("red_init_x", currRobotPose.pose.position.x);
		nh_for_param.setParam("red_init_y", currRobotPose.pose.position.y);
		nh_for_param.setParam("red_init_angle", tf::getYaw(currRobotPose.pose.orientation));				
		std::system("rosparam dump ~/for_ros/workspace/kuka_brazil/map/init_params.yaml /automapping_node");

		std::cout << LOGNAME << ": Pose's params were saved." << std::endl;			

		ros::shutdown();
	}

}


void AutomappingNode::doYourWork(){
	
	std::cout << LOGNAME << ": Start initial rotations." << std::endl;
	robot.rotateInPlace(ANGLE*PI/180,     "odom");	
	robot.rotateInPlace(2 * ANGLE*PI/180, "odom");
	robot.rotateInPlace(3 * ANGLE*PI/180, "odom");
	robot.rotateInPlace(2 * ANGLE*PI/180, "odom");
	robot.rotateInPlace(1 * ANGLE*PI/180, "odom");
	robot.rotateInPlace(0.0,	      "odom");
	std::cout << LOGNAME << ": Initial rotations have been done." << std::endl;	

	nav_msgs::GetMap map_srv;
	status = WAITING_GOAL;
	ros::Rate sleeper(50);
	
	
	while(ros::ok()){
		if (map_client.call(map_srv)){
			mapHandler(map_srv.response);
			robot.moveTo(curr_goal.target_pose, NEEDED_DIST);
		} 
		else{
			ROS_ERROR_STREAM(LOGNAME << ": Failed to call service dynamic_map!!!");
		}
	}
}



int main (int argc, char **argv){
	ros::init(argc, argv, LOGNAME);
	std::cout << LOGNAME << ": Initialization..." << std::endl;
	AutomappingNode node;
	node.doYourWork();
	return 0;
}
