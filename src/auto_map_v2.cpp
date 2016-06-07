#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
//#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/LinearMath/Vector3.h>

// gets a value of a cell in (j, i) coordinates
int8_t get_val(int j, int i, nav_msgs::OccupancyGrid gr);

// get coordinates in (x, y) form from (j, i) form
double xy_from_ji(int ji, nav_msgs::MapMetaData meta);

// get coordinates in (j, i) form from (x, y) form
int ji_from_xy(double xy, nav_msgs::MapMetaData meta);

// function checks first if a value of the current cell is -1.
// If so, it check whether the value of previous cell is 0
// And if it's true, it looks at each cell around for at least
// one more value -1. If there is a value -1,
// then current cell is a target. 
bool check(int8_t cur_val, int exp_j, int exp_i, nav_msgs::OccupancyGrid gr);

void locate_target(nav_msgs::OccupancyGrid &grid, geometry_msgs::Pose &robot_pose, move_base_msgs::MoveBaseGoal &goal)
{    
/******************** INITIALIZATION STEP ********************/
    
    // count distance between map origin and robot pose in map frame
    double x_dist = robot_pose.position.x - grid.info.origin.position.x;
    double y_dist = robot_pose.position.y - grid.info.origin.position.y;
    
    // get the i and j cell-based coordinates of a robot
    int j_dist = ji_from_xy(x_dist, grid.info);
    int i_dist = ji_from_xy(y_dist, grid.info);
    
    // next, set two integers that will define
    // (j, i)-coordinates of cell that we will explore.
    // Set their initial values to current j and i
    // of a robot
    int exp_j = j_dist;
    int exp_i = i_dist;
    
    int k = 0;
    int m = 0; // j-axes increment/decrement
    int n = 0; // i-axes increment/decrement
    
    // goal value that function will return
    // header is set to map
    // and stamp is set to current time
    // move_base_msgs::MoveBaseActionGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.orientation.w = 1.0;
    
    /******************** EXPLORING STEP ********************/
    
    // the method of exploring consists of going through each
    // cell starting from the closest and repeatedly increasing
    // the radius. Outer loop should go until it reaches either
    // map's width or map's height.
    
    while((exp_j != grid.info.width) && (exp_i != grid.info.height))
    {
    
        // first inner loop check goes up
        for (k = 0, n++; k <= n; k++)
        {
            if (exp_i + n < grid.info.height)
            {            
                // j coord. remains the same
                // i increments
                exp_i = exp_i + k;
                
                // check current value
                int8_t cur_val = get_val(exp_j, exp_i, grid);
                
                // check if it is indeed a target
                // if so, create an action goal type value
                // set it's parameters as (x, y) coordinates
                // return as a result of this function
                if (check(cur_val, exp_j, exp_i, grid))
                {
                    goal.target_pose.pose.position.x = xy_from_ji(exp_j, grid.info) + grid.info.origin.position.x;
                    goal.target_pose.pose.position.y = xy_from_ji(exp_i, grid.info) + grid.info.origin.position.y;
                    return ;//goal;
                }
            }
        }
        
        // second - left
        for (k = 0, m++; k <= m; k++)
        {
            if (exp_j - m > 0)
            {       
                // i coord. remains the same
                exp_j = exp_j - k;
                
                // check current value
                int8_t cur_val = get_val(exp_j, exp_i, grid);
                tf::Vector3 hksd;
                // check if it is indeed a target
                // if so, create an action goal type value
                // set it's parameters as (x, y) coordinates
                // fill the rest of the fields and return as 
                // a result of this function
                if (check(cur_val, exp_j, exp_i, grid))
                {
                    goal.target_pose.pose.position.x = xy_from_ji(exp_j, grid.info) + grid.info.origin.position.x;
                    goal.target_pose.pose.position.y = xy_from_ji(exp_i, grid.info) + grid.info.origin.position.y;
                    return ;
                }
            }            
        }
        
        // third - down
        for (k = 0, n++; k <= n; k++)
        {
            if (exp_i - n > 0)
            {
                // j coord. remains the same
                exp_i = exp_i - k;
                
                // check current value
                int8_t cur_val = get_val(exp_j, exp_i, grid);
                
                // check if it is indeed a target
                // if so, create an action goal type value
                // set it's parameters as (x, y) coordinates
                // fill the rest of the fields and return as 
                // a result of this function
                if (check(cur_val, exp_j, exp_i, grid))
                {
                    goal.target_pose.pose.position.x = xy_from_ji(exp_j, grid.info) + grid.info.origin.position.x;
                    goal.target_pose.pose.position.y = xy_from_ji(exp_i, grid.info) + grid.info.origin.position.y;
                    return ;
                }
            }            
        }
        
        // fourth - right
        for (k = 0, m++; k <= m; k++)
        {
            if (exp_j + m < grid.info.width)
            {       
                // i coord. remains the same
                exp_j = exp_j + k;
                
                // check current value
                int8_t cur_val = get_val(exp_j, exp_i, grid);
                
                // check if it is indeed a target
                // if so, create an action goal type value
                // set it's parameters as (x, y) coordinates
                // fill the rest of the fields and return as 
                // a result of this function
                if (check(cur_val, exp_j, exp_i, grid))
                {
                    goal.target_pose.pose.position.x = xy_from_ji(exp_j, grid.info) + grid.info.origin.position.x;
                    goal.target_pose.pose.position.y = xy_from_ji(exp_i, grid.info) + grid.info.origin.position.y;
                    return ;
                }
            }            
        }
    }
    
    // if there is no places to explore, then return current position of a robot as a goal
    if (goal.target_pose.pose.position.x == 0 && goal.target_pose.pose.position.y == 0)
    {
        goal.target_pose.pose = robot_pose;
        return;
    }
}

int8_t get_val(int j, int i, nav_msgs::OccupancyGrid gr)
{
    return gr.data[i * gr.info.width + j]; 
}

double xy_from_ji(int ji, nav_msgs::MapMetaData meta)
{
    return meta.resolution * ji;
}

int ji_from_xy(double xy, nav_msgs::MapMetaData meta)
{
    return xy / meta.resolution;
}

bool check(int8_t cur_val, int exp_j, int exp_i, nav_msgs::OccupancyGrid gr)
{
    // check current value
    if (cur_val == -1)
    {
        
        // if at least one of the cells around is free
        if ((get_val(exp_j + 1, exp_i, gr)) == 0 ||
            (get_val(exp_j + 1, exp_i + 1, gr)) == 0 ||
            (get_val(exp_j, exp_i + 1, gr)) == 0 ||
            (get_val(exp_j - 1, exp_i + 1, gr)) == 0 ||
            (get_val(exp_j - 1, exp_i, gr)) == 0 ||
            (get_val(exp_j - 1, exp_i - 1, gr)) == 0 ||
            (get_val(exp_j, exp_i - 1, gr)) == 0 ||
            (get_val(exp_j + 1, exp_i - 1, gr)) == 0)
        {
            
            // look for cells around again and count
            // how many of them are unknown. 
            // If there is at least 2 more unknown
            // cells, return true
            int counter = 0;
            if (get_val(exp_j + 1, exp_i, gr) == -1)
            {
                counter++;
            }
            if (get_val(exp_j + 1, exp_i + 1, gr) == -1)
            {
                counter++;
            }
            if (get_val(exp_j, exp_i + 1, gr) == -1)
            {
                counter++;
            }
            if (get_val(exp_j - 1, exp_i + 1, gr) == -1)
            {
                counter++;
            }
            if (get_val(exp_j - 1, exp_i, gr) == -1)
            {
                counter++;
            }
            if (get_val(exp_j - 1, exp_i - 1, gr) == -1)
            {
                counter++;
            }
            if (get_val(exp_j, exp_i - 1, gr) == -1)
            {
                counter++;
            }
            if (get_val(exp_j + 1, exp_i - 1, gr) == -1)
            {
                counter++;
            }
            
            if (counter >= 2)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }    
}
