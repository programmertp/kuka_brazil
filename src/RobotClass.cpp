#include <RobotClass.hpp>

RobotClass::RobotClass() : mbClient("move_base", true){

	vel_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	status = ROBOTCLASS_STAYING_IN_PLACE;
	std::cout << LOG_NAME << ": Initialization success." << std::endl;	
   	std::cout << LOG_NAME << ": Waiting for action move_base server to start." << std::endl;
    	mbClient.waitForServer();
    	std::cout << LOG_NAME << ": Action move_base server started." << std::endl;
}



RobotClass::~RobotClass(){
	vel_publisher.shutdown();
}



void RobotClass::moveToBG(geometry_msgs::PoseStamped goal){
	status = ROBOTCLASS_GETTING_GOAL;
	std::cout << LOG_NAME << ": Sending goal for moveToBG..." << std::endl;
	move_base_msgs::MoveBaseGoal mbGoal;
	mbGoal.target_pose = goal;
	mbClient.sendGoal(mbGoal, boost::bind(&RobotClass::requestDone, this, _1, _2), 
					boost::bind(&RobotClass::requestBecameActive, this), 
					boost::bind(&RobotClass::requestFeedback, this, _1));
	std::cout << LOG_NAME << ": Goal for moveToBG was sent." << std::endl;
}



void RobotClass::moveTo(geometry_msgs::PoseStamped goal, double dist){
	distance = dist;	
	cur_goal = goal;
	status = ROBOTCLASS_GETTING_GOAL;	
	std::cout << LOG_NAME << ": Sending goal for moveTo..." << std::endl;
	move_base_msgs::MoveBaseGoal mbGoal;
	mbGoal.target_pose = goal;
	if(distance != 0.0){
		mbClient.sendGoal(mbGoal, boost::bind(&RobotClass::requestDone, this, _1, _2), 
						boost::bind(&RobotClass::requestBecameActive, this), 
						boost::bind(&RobotClass::requestFeedbackWithDist, this, _1));
		std::cout << LOG_NAME << ": Goal for moveTo was sent." << std::endl;
	}
	else{
		mbClient.sendGoal(mbGoal, boost::bind(&RobotClass::requestDone, this, _1, _2), 
						boost::bind(&RobotClass::requestBecameActive, this), 
						boost::bind(&RobotClass::requestFeedback, this, _1));
		std::cout << LOG_NAME << ": Goal for moveTo was sent." << std::endl;	
	}
	
	ros::Rate sleeper(50);
	while ((status != ROBOTCLASS_STAYING_IN_PLACE) && ros::ok()){	
		ros::spinOnce();
		sleeper.sleep();	
	}
}



void RobotClass::moveForward(float dist, float koeff_lin, float koeff_ang, float max_error_lin, float max_error_ang){
	
	std::cout << LOG_NAME << ": Starting processing of moveForward()..." << std::endl;
	ros::Rate sleeper(50);
	static tf::TransformListener tf_listener;
	geometry_msgs::PoseStamped goal_pose, help_pose;
	help_pose.header.frame_id = "/base_link";
	//help_pose.header.stamp = ros::Time::now();
	help_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	help_pose.pose.position.x = dist;
	
	bool flag = true;
	while(ros::ok() && flag){
		try{	
			flag = false;
			tf_listener.transformPose("/map", help_pose, goal_pose);
		}
		catch (tf::TransformException ex){
			flag = true;
			ROS_ERROR("%s: During listening needed pose in map occured a problem: %s",LOG_NAME, ex.what());
			ros::Duration(1.0).sleep();
		}
	}
	
	geometry_msgs::Vector3 cur_error;
	tf::StampedTransform transform;
	geometry_msgs::Twist control_vel;	
	do{
		goal_pose.header.stamp = ros::Time::now();
		vel_publisher.publish(control_vel);		
		flag = true;
		while(ros::ok() && flag){
			try{
				flag = false;
				tf_listener.transformPose("base_link", goal_pose, help_pose);
			}
			catch (tf::TransformException ex){
				flag = true;
				ROS_ERROR("%s: During listening needed pose in base_link occured a problem: %s", LOG_NAME, ex.what());
				ros::Duration(1.0).sleep();
			}
		}

		cur_error.x = help_pose.pose.position.x;
		cur_error.y = help_pose.pose.position.y;		
		cur_error.z = tf::getYaw(help_pose.pose.orientation);
	
		tf::vector3TFToMsg(tf::Vector3(koeff_lin * cur_error.x, koeff_lin * cur_error.y, 0.0), control_vel.linear);
		tf::vector3TFToMsg(tf::Vector3(0.0, 0.0, koeff_ang * cur_error.z), control_vel.angular);
		ros::spinOnce();
		sleeper.sleep();

	}while( ros::ok() && ((std::abs(cur_error.x) >= max_error_lin) || 
		(std::abs(cur_error.y) >= max_error_lin) || 
		(std::abs(cur_error.z) >= max_error_ang)) );

	tf::vector3TFToMsg(tf::Vector3(0.0, 0.0, 0.0), control_vel.linear);
	tf::vector3TFToMsg(tf::Vector3(0.0, 0.0, 0.0), control_vel.angular);
	for(int i = 0; i < 3; i++)	
		vel_publisher.publish(control_vel);	
	std::cout << LOG_NAME << ": Ending processing of moveForward()..." << std::endl;
}



void RobotClass::rotateInPlaceBG(float angle, std::string in_frame){
	status = ROBOTCLASS_GETTING_GOAL;
	std::cout << LOG_NAME << ": Sending goal for rotateInPlaceBG..." << std::endl;
	static tf::TransformListener tf_listener;
	tf::StampedTransform transform;
	
	bool flag = true;
	while(ros::ok() && flag){
		try{
			flag = false;
			tf_listener.lookupTransform("base_link", in_frame, ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			flag = true;
			ROS_ERROR("%s: %s", LOG_NAME, ex.what());
			ros::Duration(1.0).sleep();
		}
	}

	move_base_msgs::MoveBaseGoal mbGoal;
	mbGoal.target_pose.header.stamp = ros::Time::now();
	mbGoal.target_pose.header.frame_id = in_frame;
	mbGoal.target_pose.pose.position.x = transform.getOrigin().x();
	mbGoal.target_pose.pose.position.x = transform.getOrigin().y();

	geometry_msgs::Quaternion start_quaternion;
	tf::quaternionTFToMsg(transform.getRotation(), start_quaternion);
	mbGoal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle + tf::getYaw(start_quaternion));

	mbClient.sendGoal(mbGoal, boost::bind(&RobotClass::requestDone, this, _1, _2), 
					boost::bind(&RobotClass::requestBecameActive, this), 
					boost::bind(&RobotClass::requestFeedback, this, _1));
	std::cout << LOG_NAME << ": Goal for rotateInPlaceBG was sent." << std::endl;
}



void RobotClass::rotateInPlace(float angle, std::string in_frame){
	status = ROBOTCLASS_GETTING_GOAL;
	std::cout << LOG_NAME << ": Sending goal for rotateInPlace..." << std::endl;
	static tf::TransformListener tf_listener;
	tf::StampedTransform transform;
	
	bool flag = true;
	while(ros::ok() && flag){
		try{
			flag = false;
			tf_listener.lookupTransform("base_link", in_frame, ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			flag = true;
			ROS_ERROR("%s: %s", LOG_NAME, ex.what());
			ros::Duration(1.0).sleep();
		}
	}

	move_base_msgs::MoveBaseGoal mbGoal;
	mbGoal.target_pose.header.stamp = ros::Time::now();
	mbGoal.target_pose.header.frame_id = in_frame;
	mbGoal.target_pose.pose.position.x = transform.getOrigin().x();
	mbGoal.target_pose.pose.position.x = transform.getOrigin().y();
	
	geometry_msgs::Quaternion start_quaternion;
	tf::quaternionTFToMsg(transform.getRotation(), start_quaternion);
	mbGoal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle + tf::getYaw(start_quaternion));

	std::cout << LOG_NAME << ": angle = " << tf::getYaw(start_quaternion) <<"; angles sum = " << angle + tf::getYaw(start_quaternion) << std::endl;

	mbClient.sendGoal(mbGoal, boost::bind(&RobotClass::requestDone, this, _1, _2), 
					boost::bind(&RobotClass::requestBecameActive, this), 
					boost::bind(&RobotClass::requestFeedback, this, _1));
	std::cout << LOG_NAME << ": Goal for rotateInPlace was sent." << std::endl;
	
	ros::Rate sleeper(50);
	while ((status != ROBOTCLASS_STAYING_IN_PLACE) && ros::ok()){	
		ros::spinOnce();
		sleeper.sleep();	
	}
}



void RobotClass::stop(){
	std::cout << LOG_NAME << ": Cancelling the current goal..." << std::endl;
	mbClient.cancelGoal();
}



void RobotClass::requestBecameActive(){
	status = ROBOTCLASS_MOVING_TO_GOAL;
	std::cout << LOG_NAME << ": MoveBase server start processing of the goal..." << std::endl;
}



void RobotClass::requestFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){
	cur_action_pose = feedback->base_position;
}



void RobotClass::requestFeedbackWithDist(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){
	
	cur_action_pose = feedback->base_position;	

	double distance_to_goal;
	distance_to_goal = std::sqrt( std::pow(cur_goal.pose.position.x - feedback->base_position.pose.position.x, 2) +
							std::pow(cur_goal.pose.position.y - feedback->base_position.pose.position.y, 2));
	if(distance_to_goal <= distance){
		stop();
	}
}



void RobotClass::requestDone(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result){
	std::cout << LOG_NAME << ": MoveBase server finished processing of the goal in state " << state.toString().c_str() << std::endl;
	geometry_msgs::Twist null_velocity;
	for(int i = 0; i < 3; i++)	
		vel_publisher.publish(null_velocity);
	status = ROBOTCLASS_STAYING_IN_PLACE;
}
