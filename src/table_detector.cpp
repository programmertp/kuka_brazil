#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cmath>

#define LOGNAME "table_detector_node"

class TableDetector{
	ros::NodeHandle nh_for_param, usual_nh;
	ros::Subscriber subscr_for_map;
	ros::Publisher pose_publer;
	geometry_msgs::PoseStamped goal;
	
	// Node's params *{
	double table_width, table_length, max_size_error;
	double distance_to_table, angle_with_table;	//last -- in_rad	
	// }*

	void mapReaderCallback(const nav_msgs::OccupancyGrid &map);

public:
	TableDetector();
	~TableDetector();
	void findTable();
};


TableDetector::TableDetector(): nh_for_param("~"){
	nh_for_param.param<double>("distance_to_table", distance_to_table, 0.5);
	nh_for_param.param<double>("angle_with_table", angle_with_table, 0.0);
	nh_for_param.param<double>("table_width", table_width, 0.5);
	nh_for_param.param<double>("table_length", table_length, 0.5);
	nh_for_param.param<double>("max_size_error", max_size_error, 0.05);
	pose_publer = usual_nh.advertise<geometry_msgs::PoseStamped>("detector_pose", 1);
	subscr_for_map = usual_nh.subscribe("map", 1000, &TableDetector::mapReaderCallback, this);
	//ROS_INFO_STREAM(distance_to_table << " " <<  angle_with_table << " " << table_width << " ");
}



TableDetector::~TableDetector(){
	pose_publer.shutdown();
	subscr_for_map.shutdown();
}



void TableDetector::mapReaderCallback(const nav_msgs::OccupancyGrid &map){	
	std::cout << LOGNAME << ": Map received" << std::endl;
	
	//Must be rewrite more professional {*	
	std::vector<uint8_t> help_vect;
	
	for(int i = 0; i < map.info.width*map.info.height; i++)
		help_vect.push_back( (uint8_t) map.data[i]);
	
	cv::Mat mat_image = cv::Mat(help_vect).reshape(0, map.info.height);
	//*}

	cv::Mat edges;
	std::vector<cv::Vec4i> hierarchy;
	std::vector<std::vector<cv::Point> > contours;

	try{cv::Canny(mat_image, edges, 10, 1000, 3);}
	catch(...){
		std::cout << LOGNAME << ": Error occured durind calling Canny function!" << std::endl;
	}
	
	cv::findContours( edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );	
	
	for(int i = 0; i < contours.size(); i++){
		
		cv::RotatedRect minRect;
		minRect = cv::minAreaRect( cv::Mat(contours[i]) );
		//ROS_INFO_STREAM(minRect.center << " " << minRect.angle << " " << minRect.size*map.info.resolution);	 
		if( ( (std::abs(minRect.size.width*map.info.resolution - table_width) < max_size_error) && 
		(std::abs(minRect.size.height*map.info.resolution - table_length) < max_size_error) ) || 
		( (std::abs(minRect.size.height*map.info.resolution - table_width) < max_size_error) && 
		(std::abs(minRect.size.width*map.info.resolution - table_length) < max_size_error) ) ) {
			
			std::cout << LOGNAME << ": Table was found," << std::endl;
			std::cout << LOGNAME << ": his rectangle: " << minRect.center << " " << minRect.angle << " " << minRect.size << std::endl;

			goal.pose.position.x = minRect.center.x * map.info.resolution - distance_to_table * std::cos(minRect.angle * M_PI / 180/* + 0.5*M_PI*/) + 
						map.info.origin.position.x;
			goal.pose.position.y = minRect.center.y * map.info.resolution - distance_to_table * std::sin(minRect.angle * M_PI / 180 /*+ 0.5*M_PI*/) +
						map.info.origin.position.y;
			goal.pose.orientation = tf::createQuaternionMsgFromYaw(minRect.angle* M_PI / 180 + M_PI/2 + angle_with_table);
			goal.header.frame_id = "map";	

			ros::Rate sleep_obj(5000);
			while(ros::ok()){
				pose_publer.publish(goal);
				sleep_obj.sleep();
			}	
						
		} 		

		/*cv::Rect rectangle;		
		rectangle = cv::boundingRect(contours[i]);
		ROS_INFO_STREAM(rectangle.height << " " << " " << rectangle.width << " " << rectangle.x << " " << rectangle.y);*/	
	}

	/*cv::namedWindow("Our_Map");
	cv::imshow("Our_Map", mat_image);
	cv::waitKey(10);*/
}



void TableDetector::findTable(){
	ros::spin();	
}



int main(int argc, char **argv){
	ros::init(argc, argv, LOGNAME);
	TableDetector detector;
	detector.findTable();
	return 0;
}
