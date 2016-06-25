#include <kuka_brazil_msgs/GetGoal.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/LinearMath/Vector3.h>
#include <nav_msgs/GetMap.h>

// Explorer class that handles epxloration server
class Explorer
{
        ros::NodeHandle n;
        ros::ServiceServer service;
        ros::ServiceClient client;

        bool callback(kuka_brazil_msgs::GetGoal::Request &req,
              kuka_brazil_msgs::GetGoal::Response &res);
public:
        Explorer();
        ~Explorer();
        void start_work();
	int ji_from_xy(double xy, nav_msgs::MapMetaData meta);
	double x_from_j(int j, nav_msgs::MapMetaData meta);
	double y_from_i(int i, nav_msgs::MapMetaData meta);
	bool processRegion(const nav_msgs::OccupancyGrid &gr, int i, int j);
    nav_msgs::OccupancyGrid map;
    bool searching;
};
