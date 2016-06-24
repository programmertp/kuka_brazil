#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <RobotClass.hpp>
#include <kuka_brazil_msgs/ExtraMove.h>
#include <std_msgs/Int8.h>

#define WAITING_FOR_POSE 0
#define POSE_RECEIVED 1
#define WAITING_FOR_GOAL 2
#define GOAL_RECEIVED 3
#define WAIT_FOR_EXTRA_MOVE 4
#define EXTRA_MOVE_RECEIVED 5
#define NO_MORE_EXTRA_MOVE_AVAILABLE_OR_NEED 6

#define LOGNAME "move_to_table_node"
#define ANGLE 120*M_PI/180
#define XTION_MIN_DISTANCE 0.55
#define XTION_MAX_DISTANCE 1.00
#define XTION_CORRECTION_STEP 0.15

class MoveToTable_Node{
	ros::NodeHandle nh, nh_for_param;
	ros::Subscriber goal_reader;
	ros::Subscriber pose_reader;
	ros::Publisher goose_publisher;
	ros::Publisher cord_publisher;
	ros::ServiceServer glass_service;
	//ros::Publisher init_amcl_publer;
	int status;
	double init_x, init_y, init_angle;
	float correction_distance;
	RobotClass robot;
	geometry_msgs::PoseStamped cur_goal;
	int extra_movement_num;	

	void goalReaderCallback(const geometry_msgs::PoseStamped &geomGoal);
	void poseReaderCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
	bool extraMoveServerCallback(kuka_brazil_msgs::ExtraMove::Request &req, kuka_brazil_msgs::ExtraMove::Response &res);
public:
	
	MoveToTable_Node();
	~MoveToTable_Node();
	void doYourWork();
	
};


MoveToTable_Node::MoveToTable_Node() : nh_for_param("~"){

	extra_movement_num = 0;
	status = WAITING_FOR_POSE;
	nh_for_param.param<double>("red_init_x", init_x, 0.0);
	nh_for_param.param<double>("red_init_y", init_y, 0.0);
	nh_for_param.param<double>("red_init_angle", init_angle, 0.0);
	
	pose_reader = nh.subscribe("initialpose", 1, &MoveToTable_Node::poseReaderCallback, this);
	goose_publisher = nh.advertise<std_msgs::Int8>("goose", 1);

	//init_amcl_publer = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);

	std::cout << LOGNAME << ": Initialization: success." << std::endl;	
}


MoveToTable_Node::~MoveToTable_Node(){
	goal_reader.shutdown();
	pose_reader.shutdown();
	//init_amcl_publer.shutdown();
}


void MoveToTable_Node::goalReaderCallback(const geometry_msgs::PoseStamped &goal){
	status = GOAL_RECEIVED;
	std::cout << LOGNAME <<": Goal near table received." << std::endl;
	cur_goal = goal; 	
}



void MoveToTable_Node::poseReaderCallback(const geometry_msgs::PoseWithCovarianceStamped &msg){
	status = POSE_RECEIVED;
	std::cout << LOGNAME <<": Robot's pose received." << std::endl; 	
}



bool MoveToTable_Node::extraMoveServerCallback(kuka_brazil_msgs::ExtraMove::Request &req, kuka_brazil_msgs::ExtraMove::Response &res){
	
	std::cout << LOGNAME <<": Start running extra movement" << std::endl;	
	static int server_status = 3;	
	
	if(server_status == 3){
		extra_movement_num++;
		if ((extra_movement_num <= std::ceil( (XTION_MAX_DISTANCE - XTION_MIN_DISTANCE) / XTION_CORRECTION_STEP)) && (req.state == 0)){	
		
			if(extra_movement_num == 1){
				double dist_to_table;
				nh.getParam("table_detector_node/distance_to_table", dist_to_table);
				ROS_INFO_STREAM(dist_to_table);
				correction_distance = std::abs(dist_to_table) - XTION_MIN_DISTANCE;	
				ROS_INFO_STREAM(std::abs(dist_to_table) - XTION_MIN_DISTANCE);
			}
			else
				correction_distance = -XTION_CORRECTION_STEP;
		
			status = EXTRA_MOVE_RECEIVED;
			res.answer = 1;	
			server_status = 2;
		}
		else{
			res.answer = 0;
			status = NO_MORE_EXTRA_MOVE_AVAILABLE_OR_NEED;	
		}	
	}
	else if(server_status == 2){
		if(status == WAIT_FOR_EXTRA_MOVE){
			std::cout << LOGNAME <<": Extra movement was done." << std::endl;			
			server_status = 3;
			res.answer = 3;
		} else		
			res.answer = 2;
	}
	
	
	return true; 	
}



void MoveToTable_Node::doYourWork(){
	
	ros::Rate sleeper_big(1000), sleeper_small(50);	

	/*for(int i = 0; i < 10; i++){
		std_msgs::Int8 msg;	
		msg.data = 1;
		goose_publisher.publish(msg);
		sleeper_small.sleep();
	}

	goose_publisher.shutdown();*/

	/*geometry_msgs::PoseWithCovarianceStamped init_msgs;
	init_msgs.header.frame_id = "map";
	init_msgs.pose.pose.position.x = init_x;
	init_msgs.pose.pose.position.y = init_y;
	init_msgs.pose.pose.orientation = tf::createQuaternionMsgFromYaw(init_angle);		
	
	sleeper_big.sleep();
	
	for(int i = 0; i < 8; i++){
		init_amcl_publer.publish(init_msgs);		
		sleeper_small.sleep();	
	}
	init_amcl_publer.shutdown();*/

	while((status != POSE_RECEIVED) && ros::ok()){
		ros::spinOnce();
		sleeper_small.sleep();	
	}

	pose_reader.shutdown();

	std::cout << LOGNAME << ": Correct initial pose with rotations." << std::endl;
	robot.rotateInPlace(ANGLE);
	robot.rotateInPlace(ANGLE);
	robot.rotateInPlace(ANGLE);
	robot.rotateInPlace(-ANGLE);
	robot.rotateInPlace(-ANGLE);
        robot.rotateInPlace(-ANGLE);

	std::cout << LOGNAME << ": All rotations have been done." << std::endl;

	goal_reader = nh.subscribe("detector_pose", 1, &MoveToTable_Node::goalReaderCallback, this);

	status = WAITING_FOR_GOAL;
	while((status != GOAL_RECEIVED) && ros::ok()){
		ros::spinOnce();
		sleeper_small.sleep();	
	}
	
	goal_reader.shutdown();

	robot.moveTo(cur_goal);

	ros::shutdown();
}



int main (int argc, char **argv){
	ros::init(argc, argv, "move_to_table_node");
	std::cout << LOGNAME << ": Initialization..." << std::endl;
	MoveToTable_Node node;
	node.doYourWork();
	return 0;
}
