#include "../include/case_solver/case_path/path_recording.h"

namespace TPCAP
{
    PathRecording::PathRecording()
    {
        sub_shift = nh.subscribe("/corrdShift", 1, &PathRecording::subShift, this);
        sub_path = nh.subscribe("/hybridAstarPath", 1, &PathRecording::subPath, this);
        nh.param("/case_id", case_id, -1);
        nh.param("/solution_path", file_path, std::string("/home/wsa/tpcap_ws/src/case_solver/solutions/path"));
        file_path += ("/Traj" + std::to_string(case_id) + ".csv");
        shift_flag = false;
        finish_flag = false;
        recording_t = DiscreteT;
    }
    void PathRecording::subShift(const geometry_msgs::PointStampedConstPtr &shift)
    {
        if (shift_flag)
            return;
        corrd_shift.x = shift->point.x;
        corrd_shift.y = shift->point.y;
        shift_flag = true;
        ROS_INFO("CaseSolver subscribes corrdShift successfully!");
        ROS_INFO("Corrdinate shift: x=%f,y=%f", corrd_shift.x, corrd_shift.y);
    }
    void PathRecording::subPath(const nav_msgs::PathConstPtr path)
    {
        ROS_INFO("Path:%s", file_path.c_str());
        if (shift_flag && !finish_flag && path->poses.size() > 1)
        {
            out_file.open(file_path.c_str(), std::ios::trunc);
            if (!out_file.is_open())
            {
                ROS_ERROR("Can`t open the file!");
            }
            else
            {
                ROS_INFO("Open file successfully!");
                for (int i = 0; i < path->poses.size(); i++)
                {
                    double x = path->poses[i].pose.position.x;
                    double y = path->poses[i].pose.position.y;
                    double theta = tf::getYaw(path->poses[i].pose.orientation);
                    out_file.setf(std::ios::fixed | std::ios::showpoint);
                    out_file.precision(6);
                    out_file << i * recording_t << "," << (x + corrd_shift.x) << "," << (y + corrd_shift.y) << "," << theta << std::endl;
                }
                out_file.close();
                finish_flag = true;
                ROS_INFO("Record path successfully!");
            }
        }
        else if (!shift_flag)
        {
            ROS_ERROR("Cannot recording the path without corrdShift!");
        }
        else if (finish_flag)
        {
            ROS_ERROR("Finish recording already!");
        }
        else if (path->poses.size() <= 1)
        {
            ROS_ERROR("Cannot recording the path since length less than 1!");
        }
    }
}