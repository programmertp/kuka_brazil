#include <ExplorationHelper.h>

Explorer::Explorer()
{
        searching = false;
        ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("dynamic_map");
        ros::ServiceServer service = n.advertiseService("get_goal_at_map", &Explorer::callback, this);
}

Explorer::~Explorer()
{
        client.shutdown();
        service.shutdown();
}

void Explorer::start_work()
{
                client.waitForExistence();
                ros::spin();
}

bool Explorer::callback(kuka_brazil_msgs::GetGoal::Request &req,
              kuka_brazil_msgs::GetGoal::Response &res)
{
    nav_msgs::GetMap srv;
    if (client.call(srv))
    {
        map = srv.response.map;
        searching = true;
    }
    else
    {
        ROS_ERROR("Failed to call server dynamic_map");
        ros::shutdown();
    }

    // distance between map origin and /base_link in /map frame
    double x_dist = req.robot_pose.x - map.info.origin.position.x;
    double y_dist = req.robot_pose.y - map.info.origin.position.y;

    // same distance in (j, i)-coordinates
    int cur_j = ji_from_xy(x_dist, map.info);
    int cur_i = ji_from_xy(y_dist, map.info);

    // move potential centroid in the "top-left" corner
    int exp_j = cur_j - 10;
    int exp_i = cur_i + 10;

    int k = 0;
    int m = 21; // ---> j-axes increment/decrement
    int n = 21; // ---> i-axes increment/decrement

    while((exp_j - 10 >= 0) && (exp_j + 10 < map.info.width)
       && (exp_i - 10 >= 0) && (exp_i + 10 < map.info.height))
    {
        // going down
	for(k = 1, n++; k <= n; k++)
	{
		// j coordinate remains the same
		exp_i = exp_i - 1;
		if(processRegion(map, exp_i, exp_j))
		{
			double x = x_from_j(exp_j, map.info);
			double y = y_from_i(exp_i, map.info);
			for (int z = 0, size = req.excluded_goals.size(); z < size; z++)
			{
				if ((x == req.excluded_goals[z].x) &&
					(y == req.excluded_goals[z].y)) break;
			}
			res.goal.x = x;
			res.goal.y = y;
			return true;
		}
	}

        // going right
	for(k = 1, m++; k <= m; k++)
	{
		exp_j = exp_j + 1;
		if(processRegion(map, exp_i, exp_j))
		{
			double x = x_from_j(exp_j, map.info);
			double y = y_from_i(exp_i, map.info);
			for (int z = 0, size = req.excluded_goals.size(); z < size; z++)
			{
				if ((x == req.excluded_goals[z].x) &&
					(y == req.excluded_goals[z].y)) break;
			}
			res.goal.x = x;
			res.goal.y = y;
			return true;
		}
	}

        // going up
	for(k = 1, n++; k <= n; k++)
	{
		exp_i = exp_i + 1;
		if(processRegion(map, exp_i, exp_j))
		{
			double x = x_from_j(exp_j, map.info);
			double y = y_from_i(exp_i, map.info);
			for (int z = 0, size = req.excluded_goals.size(); z < size; z++)
			{
				if ((x == req.excluded_goals[z].x) &&
					(y == req.excluded_goals[z].y)) break;
			}
			res.goal.x = x;
			res.goal.y = y;
			return true;
		}
	}

        // going left
	for(k = 1, m++; k <= m; k++)
	{
		exp_j = exp_j - 1;
		if(processRegion(map, exp_i, exp_j))
		{
			double x = x_from_j(exp_j, map.info);
			double y = y_from_i(exp_i, map.info);
			for (int z = 0, size = req.excluded_goals.size(); z < size; z++)
			{
				if ((x == req.excluded_goals[z].x) &&
					(y == req.excluded_goals[z].y)) break;
			}
			res.goal.x = x;
			res.goal.y = y;
			return true;
		}
	}
    }
}

int Explorer::ji_from_xy(double xy, nav_msgs::MapMetaData meta)
{
    return xy / meta.resolution;
}

double Explorer::x_from_j(int j, nav_msgs::MapMetaData meta)
{
        return meta.resolution * j + meta.origin.position.x;
}

double Explorer::y_from_i(int i, nav_msgs::MapMetaData meta)
{
        return meta.resolution * i + meta.origin.position.y;
}

bool Explorer::processRegion(const nav_msgs::OccupancyGrid &gr, int i, int j)
{
        // FUNCTION THAT PROCESSES 21x21 REGION AND RETURNS TRUE IF IT'S A FRONTIER 
        /**************************************************************************/

        // variables that contain an amount of occupied, free and unknown cells
        int free = 0;
        int occupied = 0;
        int unknown = 0;

        // new indices
        int w;
        int h;

        for(h = i-10; h <= i+10; h++)
        {
                for(w = j-10; w <= j+10; w++)
                {
                        switch(gr.data[w + (gr.info.width)*h])
                        {
                                case 100:
                                        occupied++;
                                        if (occupied > 2) return false;
                                        break;
                                case 0:
                                        free++;
                                        break;
                                case -1:
                                        unknown++;
                                        break;
                                default:
                                        ROS_WARN("THIS CELL IS NEITHER [0, 100], NOR IT IS -1");
                        }
                }
        }

        if ((unknown >= 200) && (free >= 150)) return true;
        ROS_WARN("FOR SOME REASON, I THINK THIS REGION IS NOW A FRONTIER");
        return false;
}
