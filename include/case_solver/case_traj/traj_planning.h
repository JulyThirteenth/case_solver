#ifndef _TRAJ_PLANNING_H_
#define _TRAJ_PALNNING_H_

#include <tf/tf.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "../include/utils/algorithm_utils.h"

namespace TPCAP
{
    class TrajPlanning
    {
    public:
        TrajPlanning();
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub_path;
        ros::Publisher pub_traj;
        ros::Publisher pub_traj_area;
        void smoothPath(std::vector<Vector> &path, double weight_data, 
            double weigth_smooth, double tolerance);
        void subPath(const nav_msgs::PathConstPtr path);
    };
}

#endif