#include "../include/case_solver/case_traj/traj_planning.h"
namespace TPCAP
{
    TrajPlanning::TrajPlanning()
    {
        sub_path = nh.subscribe("/hybridAstarPath", 1, &TrajPlanning::subPath, this);
        pub_traj = nh.advertise<nav_msgs::Path>("hybridAstarTraj", 1, true);
        pub_traj_area = nh.advertise<visualization_msgs::MarkerArray>("/trajArea",1,true);
    }
    void TrajPlanning::smoothPath(std::vector<Vector> &path, double weight_data = 0.1,
                                  double weigth_smooth = 0.45, double tolerance = 0.05)
    {
        if (path.size() < 2)
            return;
        const std::vector<Vector> &path_in = path;
        std::vector<Vector> path_out = path;
        double change = tolerance;
        double temp_x, temp_y;
        int size = path_in.size();
        while (change >= tolerance)
        {
            change = 0.;
            for (int i = 1; i < size - 1; i++)
            {
                temp_x = path_out[i].x;
                temp_y = path_out[i].y;
                path_out[i].x += weight_data * (path_in[i].x - path_out[i].x);
                path_out[i].y += weight_data * (path_in[i].y - path_out[i].y);
                path_out[i].x += weigth_smooth * (path_out[i - 1].x + path_out[i + 1].x - 2.0 * path_out[i].x);
                path_out[i].y += weigth_smooth * (path_out[i - 1].y + path_out[i + 1].y - 2.0 * path_out[i].y);
                change += fabs(temp_x - path_out[i].x);
                change += fabs(temp_y - path_out[i].y);
            }
        }
        path = path_out;
    }
    void TrajPlanning::subPath(const nav_msgs::PathConstPtr path)
    {
        std::vector<Vector> path_in;
        Vector pose;
        for (int i = 0; i < path->poses.size(); i++)
        {
            pose = {path->poses[i].pose.position.x,
                    path->poses[i].pose.position.y,
                    tf::getYaw(path->poses[i].pose.orientation)};
            path_in.push_back(pose);
        }
        smoothPath(path_in, WeightData, WeightSmooth, Tolerance);
        nav_msgs::Path path_out;
        path_out.header.frame_id = "map";
        geometry_msgs::PoseStamped temp;
        temp.header.frame_id = "map";
        for (int i = 0; i < path_in.size(); i++)
        {
            temp.pose.position.x = path_in[i].x;
            temp.pose.position.y = path_in[i].y;
            temp.pose.orientation = tf::createQuaternionMsgFromYaw(path_in[i].t);
            temp.header.stamp = ros::Time::now();
            path_out.poses.push_back(temp);
        }
        path_out.header.stamp = ros::Time::now();
        pub_traj.publish(path_out);

        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "pathArea";
        marker.lifetime = ros::Duration(0.);
        marker.frame_locked = true;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        for (int i = 0; i < path_in.size(); i += Skip)
        {
            marker.id = i;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.1;
            marker.pose.orientation.w = 1.0;
            geometry_msgs::Point temp_point;
            temp_point.x = path_in[i].x + (FrontHang + WheelBase) * sin(M_PI_2 - path_in[i].t);
            temp_point.y = path_in[i].y + (FrontHang + WheelBase) * cos(M_PI_2 - path_in[i].t);
            marker.points.push_back(temp_point);
            temp_point.x = path_in[i].x + (-RearHang) * sin(M_PI_2 - path_in[i].t);
            temp_point.y = path_in[i].y + (-RearHang) * cos(M_PI_2 - path_in[i].t);
            marker.points.push_back(temp_point);
            marker.scale.x = Width;
            marker_array.markers.push_back(marker);
            marker.points.clear();
        }
        pub_traj_area.publish(marker_array);
    }
}