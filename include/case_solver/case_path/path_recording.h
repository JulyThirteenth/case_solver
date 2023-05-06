#ifndef _PATH_RECORDING_H_
#define _PATH_RECORDING_H

#include <tf/tf.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <fstream>
#include <string>
#include "../include/algorithm/params.h"
#include "../include/utils/algorithm_utils.h"
namespace TPCAP
{
    class PathRecording
    {
    public:
        PathRecording();
        void subShift(const geometry_msgs::PointStampedConstPtr &shift);
        void subPath(const nav_msgs::PathConstPtr path);

    private:
        ros::NodeHandle nh;
        ros::Subscriber sub_shift;
        ros::Subscriber sub_path;
        std::ofstream out_file;
        std::string file_path;
        int case_id;
        bool shift_flag;
        bool finish_flag;
        Point corrd_shift;
        double recording_t;
    };
}

#endif