#include "../include/case_solver/case_path/path_planning.h"

namespace TPCAP
{
    PathPlanning::PathPlanning()
    {
        pub_start = nh.advertise<geometry_msgs::PoseStamped>("/start", 1, true);
        pub_start_area = nh.advertise<visualization_msgs::Marker>("/startArea", 1, true);
        pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/goal", 1, true);
        pub_goal_area = nh.advertise<visualization_msgs::Marker>("/goalArea", 1, true);
        // pub_astar_path = nh.advertise<nav_msgs::Path>("/revisedAStarPath", 1, true);
        // pub_rs_path = nh.advertise<nav_msgs::Path>("/reedsSheppPath", 1, true);
        pub_hybrid_path = nh.advertise<nav_msgs::Path>("/hybridAstarPath", 1, true);
        pub_path_area = nh.advertise<visualization_msgs::MarkerArray>("/pathArea", 1, true);
        if (parseCsv())
        {
            std::cout << "Path Planning initialize successfully!" << std::endl;
            pubStartAndGoal();
        }
        else
        {
            std::cout << "Path Planning initialize failed!" << std::endl;
        }
        finish_flag = false;
    }

    bool PathPlanning::parseCsv()
    {
        std::cout << std::fixed << std::setprecision(6); //设置数据流保留小数点后6位
        int case_id;
        std::string case_path;
        nh.param("/case_id", case_id, 0);
        nh.param("/case_path", case_path, std::string("/home/wsa/tpcap_ws/src/case_solver/cases"));
        case_path += ("/Case" + std::to_string(case_id) + ".csv");
        std::fstream case_reader;
        case_reader.open(case_path.c_str(), std::ios::in);
        if (!case_reader.is_open())
        {
            ROS_ERROR("%s", (std::string("Csv file is not exist: ") + case_path).c_str());
            return false;
        }
        std::string case_data;
        case_reader >> case_data;
        case_reader.close();
        int id = 0, sub_id = 0;
        for (int i = 0; i < 6; i++) //获取 csv 文件中起点与终点数据
        {
            while (case_data[id + sub_id] != ',')
            {
                sub_id++;
            }
            double temp = atof(case_data.substr(id, sub_id).c_str());
            switch (i)
            {
            case 0:
                start.x = temp;
                break;
            case 1:
                start.y = temp;
                break;
            case 2:
                start.t = temp;
                break;
            case 3:
                goal.x = temp;
                break;
            case 4:
                goal.y = temp;
                break;
            case 5:
                goal.t = temp;
            }
            id += (++sub_id);
            sub_id = 0;
        }
        double resolution;
        nh.param("/resolution", resolution, 0.1);
        CsvToGridMap ctg(resolution);
        if (!ctg.getGridMap(grid_map))
        {
            return false;
        }
        start.x -= ctg.map_info.corrd_shift.x;
        start.y -= ctg.map_info.corrd_shift.y;
        goal.x -= ctg.map_info.corrd_shift.x;
        goal.y -= ctg.map_info.corrd_shift.y;
        return true;
    }

    void PathPlanning::pubStartAndGoal()
    {
        geometry_msgs::PoseStamped start_pose, goal_pose;
        start_pose.pose.position.x = start.x;
        start_pose.pose.position.y = start.y;
        start_pose.pose.orientation = tf::createQuaternionMsgFromYaw(start.t);
        start_pose.header.frame_id = "map";
        start_pose.header.stamp = ros::Time::now();
        pub_start.publish(start_pose);
        goal_pose.pose.position.x = goal.x;
        goal_pose.pose.position.y = goal.y;
        goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal.t);
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = ros::Time::now();
        pub_goal.publish(goal_pose);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "startArea";
        marker.lifetime = ros::Duration(0.);
        marker.frame_locked = true;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.pose.orientation.w = 1.0;
        geometry_msgs::Point temp_point;
        temp_point.x = start.x + (FrontHang + WheelBase) * sin(M_PI_2 - start.t);
        temp_point.y = start.y + (FrontHang + WheelBase) * cos(M_PI_2 - start.t);
        marker.points.push_back(temp_point);
        temp_point.x = start.x + (-RearHang) * sin(M_PI_2 - start.t);
        temp_point.y = start.y + (-RearHang) * cos(M_PI_2 - start.t);
        marker.points.push_back(temp_point);
        marker.scale.x = Width;
        pub_start_area.publish(marker);

        marker.points.clear();
        marker.header.stamp = ros::Time::now();
        marker.ns = "goalArea";
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        temp_point.x = goal.x + (FrontHang + WheelBase) * sin(M_PI_2 - goal.t);
        temp_point.y = goal.y + (FrontHang + WheelBase) * cos(M_PI_2 - goal.t);
        marker.points.push_back(temp_point);
        temp_point.x = goal.x + (-RearHang) * sin(M_PI_2 - goal.t);
        temp_point.y = goal.y + (-RearHang) * cos(M_PI_2 - goal.t);
        marker.points.push_back(temp_point);
        marker.scale.x = Width;
        pub_goal_area.publish(marker);
    }

    // void PathPlanning::pubRevisedAstarPath()
    // {
    //     start_timer_clock();
    //     AstarPath astar_path = hybrid_astar.revised_astar.findPath(start, goal);
    //     ROS_INFO("RevisedAStar using time: %f seconds", end_timer_clock(SEC));
    //     ROS_INFO("astar_path length: %f", astar_path.length);
    //     nav_msgs::Path revised_astar_path;
    //     revised_astar_path.header.frame_id = "map";
    //     geometry_msgs::PoseStamped pose;
    //     pose.header.frame_id = "map";
    //     for (int i = 0; i < astar_path.path.size(); i++)
    //     {
    //         pose.pose.position.x = astar_path.path[i].x;
    //         pose.pose.position.y = astar_path.path[i].y;
    //         pose.header.stamp = ros::Time::now();
    //         revised_astar_path.poses.push_back(pose);
    //     }
    //     revised_astar_path.header.stamp = ros::Time::now();
    //     pub_astar_path.publish(revised_astar_path);
    // }

    // void PathPlanning::pubReedsSheppPath()
    // {
    //     double s[3] = {start.x, start.y, start.t};
    //     double g[3] = {goal.x, goal.y, goal.t};
    //     start_timer_clock();
    //     RSPath rs_path = hybrid_astar.r.xingshensample(s, g, 0.1);
    //     ROS_INFO("ReedsSheep using time: %f seconds", end_timer_clock(SEC));
    //     ROS_INFO("rs_path length: %f", hybrid_astar.r.distance(s, g));
    //     nav_msgs::Path reeds_shepp_path;
    //     reeds_shepp_path.header.frame_id = "map";
    //     geometry_msgs::PoseStamped pose;
    //     pose.header.frame_id = "map";
    //     for (int i = 0; i < rs_path.size(); i++)
    //     {
    //         pose.pose.position.x = rs_path[i][0];
    //         pose.pose.position.y = rs_path[i][1];
    //         pose.pose.orientation = tf::createQuaternionMsgFromYaw(rs_path[i][2]);
    //         pose.header.stamp = ros::Time::now();
    //         reeds_shepp_path.poses.push_back(pose);
    //     }
    //     reeds_shepp_path.header.stamp = ros::Time::now();
    //     pub_rs_path.publish(reeds_shepp_path);
    // }

    void PathPlanning::pubHybridAstarPath()
    {
        
        if (finish_flag == false)
        {
            ROS_INFO("Begin Algorithm!");
            HybridAstarPath path = hybrid_astar.findPath(start, goal, grid_map);
            ROS_INFO("Finish Algorithm!");
            nav_msgs::Path hybrid_astar_path;
            hybrid_astar_path.header.frame_id = "map";
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            for (int i = 0; i < path.size(); i++)
            {
                pose.pose.position.x = path[i].x;
                pose.pose.position.y = path[i].y;
                pose.pose.orientation = tf::createQuaternionMsgFromYaw(path[i].t);
                pose.header.stamp = ros::Time::now();
                hybrid_astar_path.poses.push_back(pose);
            }
            hybrid_astar_path.header.stamp = ros::Time::now();
            pub_hybrid_path.publish(hybrid_astar_path);

            visualization_msgs::MarkerArray marker_array;
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "pathArea";
            marker.lifetime = ros::Duration(0.);
            marker.frame_locked = true;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            for (int i = 0; i < path.size(); i += Skip)
            {
                marker.id = i;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 0.1;
                marker.pose.orientation.w = 1.0;
                geometry_msgs::Point temp_point;
                temp_point.x = path[i].x + (FrontHang + WheelBase) * sin(M_PI_2 - path[i].t);
                temp_point.y = path[i].y + (FrontHang + WheelBase) * cos(M_PI_2 - path[i].t);
                marker.points.push_back(temp_point);
                temp_point.x = path[i].x + (-RearHang) * sin(M_PI_2 - path[i].t);
                temp_point.y = path[i].y + (-RearHang) * cos(M_PI_2 - path[i].t);
                marker.points.push_back(temp_point);
                marker.scale.x = Width;
                marker_array.markers.push_back(marker);
                marker.points.clear();
            }
            pub_path_area.publish(marker_array);

            finish_flag = true;
        }
    }
}