#ifndef _PATH_PLANNING_H_
#define _PATH_PLANNING_H_

#include <fstream>
#include <tf/tf.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "../include/utils/algorithm_utils.h"
#include "../include/algorithm/params.h"
#include "../include/algorithm/reeds_shepp.h"
#include "../include/algorithm/revised_astar.h"
#include "../include/algorithm/hybrid_astar.h"
#include "../include/case_solver/case_map/csv_to_gridmap.h"

namespace TPCAP
{
	//作用：实现起点到终点的路径寻找
	class PathPlanning
	{
	public:
		PathPlanning();
		bool parseCsv();
		void pubStartAndGoal();
		void pubHybridAstarPath();
		// void pubRevisedAstarPath();
		// void pubReedsSheppPath();
	private:
		ros::NodeHandle nh;
		ros::Publisher pub_start;
		ros::Publisher pub_goal;
		ros::Publisher pub_start_area;
		ros::Publisher pub_goal_area;
		// ros::Publisher pub_astar_path;
		// ros::Publisher pub_rs_path;
		ros::Publisher pub_hybrid_path;
		ros::Publisher pub_path_area;
		Vector start;
		Vector goal; 
		GridMap grid_map; 
		HybridAstar hybrid_astar;
		bool finish_flag;
	};
}

#endif