#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define DIFF_TIME 0.5


class Stopper{
	ros::Publisher publ;
	ros::Subscriber sub;
	ros::NodeHandle n; 	
	double last_time;
	bool was_message;	

	void StopCallback(const geometry_msgs::Twist &msg);
public:
	Stopper();
	void spin();
};


Stopper::Stopper(){
	was_message = false;
	publ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	sub = n.subscribe("/cmd_vel", 1000, &Stopper::StopCallback, this);	
}


void Stopper::spin(){
	ros::Rate sleeper(10);	
	while(ros::ok()){
		ros::spinOnce();
		if(was_message){
			double cur_time = ros::Time::now().toSec();
			if((cur_time - last_time) > DIFF_TIME){
				geometry_msgs::Twist empty_msg;		
				publ.publish(empty_msg);
				was_message = false;			
			}
		}
		sleeper.sleep();
	}	
}



void Stopper::StopCallback(const geometry_msgs::Twist &msg){
	last_time = ros::Time::now().toSec();
	if( !( (msg.linear.x == 0) && (msg.linear.y == 0) && (msg.angular.z == 0) ) ) 
		was_message = true;	
}



int main(int argc, char **argv){
	ros::init(argc, argv, "stopper_node");
	Stopper stop_obj;
	stop_obj.spin();
	return 0;
}
