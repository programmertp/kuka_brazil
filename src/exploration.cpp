#include "ros/ros.h"
#include "kuka_brazil_msgs/GetGoal.h"

// Here will remain service callback function

int main()
{
    ros::init(argc, argv, "get_goal_at_map_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("get_goal_at_map", explorer);
    ros::spin();

    return 0;
}

