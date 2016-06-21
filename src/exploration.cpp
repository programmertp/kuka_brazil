#include "ros/ros.h"
#include "kuka_brazil_msgs/GetGoal.h"
)
// Here will remain service callback function
bool get_goal(kuka_brazil_msgs::GetGoal::Request &req,
              kuka_brazil_msgs::GetGoal::Response &res)
{
    // count distance between map origin and robot pose in "/map" frame
    double x_dist = req.robot_pose.x 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_goal_at_map_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("get_goal_at_map", explorer);
    ros::spin();

    return 0;
}

