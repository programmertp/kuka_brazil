#include <RobotClass.hpp>
#include <nav_msgs/GetPlan.h>
#include <kuka_brazil_msgs/GetGoal.h>
#include <cstdlib>
#include <vector>
#include <fstream>

#define EQUAL(A, B) A.x == B.x && A.y == B.y

class AutomappingNode{

    ros::NodeHandle nh, nh_for_params;
    RobotClass robot;
    ros::ServiceClient goal_requester, pathfinder;

    double safe_distance, tolerance;
    std::string map_name, pose_params_file_name;

    geometry_msgs::PoseStamped safeGoalInPlan(const nav_msgs::Path &plan);
    double distanceBetweenPoses(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2);

public:

    AutomappingNode();
    ~AutomappingNode();
    void doYourWork();
};
