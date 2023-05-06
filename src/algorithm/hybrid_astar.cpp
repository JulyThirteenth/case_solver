#include "../include/algorithm/hybrid_astar.h"
#include "../include/utils/time_counter.h"

namespace TPCAP
{
    HybridAstar::HybridAstar()
    {
        //算法参数初始化
        gama = HybridGama;
        iterations = HybridInterations;
        nrs = HybridNrs;
        nes = HybridNes;
        cdp = HybridCDP;
        rp = HybridRP;
        type = HybridSearchType;
        //可视化初始化
        pub_curr_start = nh.advertise<visualization_msgs::Marker>("/currStartArea", 1, true);
        pub_curr_goal = nh.advertise<visualization_msgs::Marker>("/currGoalArea", 1, true);
    }
    void HybridAstar::visualizeCurr(const Node3dPointer &curr_s, const Node3dPointer &curr_g)
    {
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
        temp_point.x = curr_s->pose.x + (FrontHang + WheelBase) * sin(M_PI_2 - curr_s->pose.t);
        temp_point.y = curr_s->pose.y + (FrontHang + WheelBase) * cos(M_PI_2 - curr_s->pose.t);
        marker.points.push_back(temp_point);
        temp_point.x = curr_s->pose.x + (-RearHang) * sin(M_PI_2 - curr_s->pose.t);
        temp_point.y = curr_s->pose.y + (-RearHang) * cos(M_PI_2 - curr_s->pose.t);
        marker.points.push_back(temp_point);
        marker.scale.x = Width;
        pub_curr_start.publish(marker);

        marker.points.clear();
        marker.header.stamp = ros::Time::now();
        marker.ns = "goalArea";
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        temp_point.x = curr_g->pose.x + (FrontHang + WheelBase) * sin(M_PI_2 - curr_g->pose.t);
        temp_point.y = curr_g->pose.y + (FrontHang + WheelBase) * cos(M_PI_2 - curr_g->pose.t);
        marker.points.push_back(temp_point);
        temp_point.x = curr_g->pose.x + (-RearHang) * sin(M_PI_2 - curr_g->pose.t);
        temp_point.y = curr_g->pose.y + (-RearHang) * cos(M_PI_2 - curr_g->pose.t);
        marker.points.push_back(temp_point);
        marker.scale.x = Width;
        pub_curr_goal.publish(marker);
    }

    //计算前一节点拓展到当前节点的g值，dir=true正向搜索，dir=false反向搜素
    double HybridAstar::caclG(const Node3dPointer &pred, const Node3dPointer &curr, bool dir)
    {
        double res = pred->g;
        double V = OpearationArr[curr->o][0];
        double Fi = OpearationArr[curr->o][1];
        double T = LevelArr[curr->l];
        if (Fi == 0)
            res += VMax * T;
        else if ((Fi * 2) == -FiMax || (Fi * 2) == FiMax)
            res += VMax * T * 1.118;
        else
            res += VMax * T * 1.414;
        if (curr->o != pred->o) //改变方向惩罚
            res *= cdp;
        if (dir == true && V < 0) //正向搜索倒车惩罚
            res *= rp;
        if (dir == false && V > 0) //反向搜索前进惩罚
            res *= rp;
        return res;
    }
    double HybridAstar::caclH(const Node3dPointer &curr, const Node3dPointer &goal)
    {
        double s[3] = {curr->pose.x, curr->pose.y, curr->pose.t};
        double g[3] = {goal->pose.x, goal->pose.y, goal->pose.t};
        double rs_length = r.distance(s, g);
        double astar_length = revised_astar.findPath(curr->pose, goal->pose).length;
        return (rs_length > astar_length ? rs_length : astar_length);
    }
    HybridAstarPath HybridAstar::recallSegPath(const Node3dPointer &seg)
    {
        HybridAstarPath res;
        if (!seg)
            return res;
        Vector temp_pose;
        double V, Fi, T;
        double parameter, temp_pose_theta;
        Node3dPointer curr = seg;
        Node3dPointer pred;
        while (curr->pred != nullptr)
        {
            pred = curr->pred;
            V = OpearationArr[curr->o][0];
            Fi = OpearationArr[curr->o][1];
            T = LevelArr[curr->l];
            //采用反向回溯使得当前节点到上一节点的轨迹点反向放入轨迹中
            for (double st = T; st > 0; st -= Sample_T)
            {
                if (Fi == 0)
                {
                    temp_pose.t = pred->pose.t;
                    temp_pose.x = pred->pose.x + V * cos(pred->pose.t) * st;
                    temp_pose.y = pred->pose.y + V * sin(pred->pose.t) * st;
                    res.push_back(temp_pose);
                }
                else
                {
                    parameter = V * tan(Fi) / (WheelBase);
                    temp_pose_theta = pred->pose.t + parameter * st;
                    temp_pose.t = temp_pose_theta;
                    temp_pose.x = pred->pose.x + V / parameter * (sin(temp_pose_theta) - sin(pred->pose.t));
                    temp_pose.y = pred->pose.y + V / parameter * (cos(pred->pose.t) - cos(temp_pose_theta));
                    res.push_back(temp_pose);
                }
            }
            curr = pred;
        }
        res.push_back(curr->pose);
        return res;
    }
    HybridAstarPath HybridAstar::recallPath(const Node3dPointer &curr_s, const Node3dPointer &curr_g)
    {
        // std::cout << "Get curr_to_start!" << std::endl;
        HybridAstarPath curr_to_start = recallSegPath(curr_s);
        // std::cout << "Get rs_path!" << std::endl;
        double s[3] = {curr_s->pose.x, curr_s->pose.y, curr_s->pose.t};
        double g[3] = {curr_g->pose.x, curr_g->pose.y, curr_g->pose.t};
        std::vector<std::vector<double>> path = r.xingshensample(s, g, 0.1);
        HybridAstarPath rs_path;
        Vector temp_pose;
        for (int i = 0; i < path.size(); i++)
        {
            temp_pose.x = path[i][0];
            temp_pose.y = path[i][1];
            temp_pose.t = path[i][2];
            rs_path.push_back(temp_pose);
        }
        // std::cout << "Get curr_to_goal!" << std::endl;
        HybridAstarPath curr_to_goal = recallSegPath(curr_g);
        HybridAstarPath res;
        // std::cout << "Add curr_to_start to path!" << std::endl;
        for (int i = curr_to_start.size() - 1; i > -1; i--)
        {
            res.push_back(curr_to_start[i]);
        }
        // std::cout << "Add rs_path to path!" << std::endl;
        for (int i = 0; i < rs_path.size(); i++)
        {
            res.push_back(rs_path[i]);
        }
        // std::cout << "Add curr_to_goal to path!" << std::endl;
        for (int i = 0; i < curr_to_goal.size(); i++)
        {
            res.push_back(curr_to_goal[i]);
        }
        return res;
    }
    HybridAstarPath HybridAstar::findPath(Vector &start, Vector &goal, GridMap &grid_map)
    {
        HybridAstarPath res;
        //初始化内部节点地图
        std::cout << "Begin init node map!" << std::endl;
        Node3dMap node3d_map;
        node3d_map.resize(grid_map.cols);
        for (int c = 0; c < grid_map.cols; c++)
        {
            node3d_map[c].resize(grid_map.rows);
            for (int r = 0; r < grid_map.rows; r++)
            {
                if (grid_map.occs[c][r] == false) //只对栅格地图中空闲的栅格初始化对应节点以节省内存
                {
                    node3d_map[c][r] = std::make_shared<Node3d>();
                    // node3d_map[c][r].reset(new Node3d);
                    node3d_map[c][r]->id = {c, r};
                }
            }
        }
        std::cout << "Finish init node map!" << std::endl;
        //将起始点转换为节点地图中的节点
        std::cout << "Begin init start & goal node!" << std::endl;
        std::cout << grid_map.cols << " " << grid_map.rows << std::endl;
        double resolution = grid_map.resolution;
        std::cout << int(start.x / resolution) << " " << int(start.y / resolution) << std::endl;
        Node3dPointer s = node3d_map[int(start.x / resolution)][int(start.y / resolution)];
        s->pose = start;
        std::cout << int(goal.x / resolution) << " " << int(goal.y / resolution) << std::endl;
        Node3dPointer g = node3d_map[int(goal.x / resolution)][int(goal.y / resolution)];
        g->pose = goal;
        std::cout << "Finish init start & goal node!" << std::endl;
        //构造优先队列
        std::priority_queue<Node3dPointer, Node3dList, nodeCompareByCost> start_queue, goal_queue;
        //根据不同搜索方式初始化优先队列
        std::cout << "Begin init start & goal queue!" << std::endl;
        if (type == Bothway || type == Forwardway)
        {
            s->g = 0.;
            s->h = caclH(s, g);
            s->f = s->g + gama * s->h;
            s->open = SOpen;
            start_queue.push(s);
        }
        if (type == Bothway || type == Reverseway)
        {
            g->g = 0.;
            g->h = caclH(s, g);
            g->f = g->g + gama * s->h;
            g->open = GOpen;
            goal_queue.push(g);
        }
        std::cout << "Finish init start & goal queue!" << std::endl;
        //定义并初始化当前起点、终点节点指针
        Node3dPointer curr_s = s;
        Node3dPointer curr_g = g;
        //定义当前起点、终点拓展子节点数组
        ChildNodeList child_s_list, child_g_list;
        //定义当前起点、终点拓展子节点指针
        Node3dPointer child_s, child_g;
        //定义当前起点、终点拓展子节点指针索引
        int temp_col, temp_row;
        //定义双向探索前向或反向拓展标志,先从前向探索开始
        bool forward_flag = true;
        bool reverse_flag = false;
        //定义当前迭代次数
        int iter = 0;
        //在内部节点地图上执行Hybrid Astar算法
        while (iter < iterations)
        {
            iter++;
            std::cout << "iter: " << iter << std::endl;
            //如果起点优先队列为空则停止寻找路径
            if (start_queue.empty() || goal_queue.empty())
            {
                ROS_ERROR("Start or Goal OpenPriQue is empty, stop finding path!");
                return recallPath(curr_s, curr_g);
            }
            if ((type == Bothway) && (iter % nes == 0))
            {
                // Bothway方式下每间隔nexpansion次拓展交换一次拓展方向
                forward_flag = !forward_flag;
                reverse_flag = !reverse_flag;
            }
            if ((type == Forwardway) || ((type == Bothway) && (forward_flag == true)))
            {
                //更新当前起始节点
                curr_s = start_queue.top();
                start_queue.pop();
                curr_s->open = UnOpen;
                curr_s->close = SClose;
                // std::cout << "start queue pop" << std::endl;
            }
            if ((type == Reverseway) || ((type == Bothway) && (reverse_flag == true)))
            {
                //更新当前终止节点
                curr_g = goal_queue.top();
                goal_queue.pop();
                curr_g->open = UnOpen;
                curr_g->close = GClose;
                // std::cout << "goal queue pop" << std::endl;
            }
            // start_timer_clock();
            visualizeCurr(curr_s, curr_g);
            // ROS_INFO("visualizeCurr using time: %f", end_timer_clock(SEC));
            //间隔nrs次迭代执行一次RS曲线连接
            if (iter % nrs == 0)
            {
                std::cout << "*******************************iter:" << iter
                          << "*******************************" << std::endl;
                double s[3] = {curr_s->pose.x, curr_s->pose.y, curr_s->pose.t};
                double g[3] = {curr_g->pose.x, curr_g->pose.y, curr_g->pose.t};
                std::vector<std::vector<double>> path = r.xingshensample(s, g, DiscreteD);
                Vector temp_pose;
                bool collision_flag = false;
                for (int i = 0; i < path.size(); i++)
                {
                    temp_pose = {path[i][0], path[i][1], path[i][2]};
                    if (vehicle.collisionCheck(temp_pose, grid_map))
                    {
                        collision_flag = true;
                        break;
                    }
                }
                if (collision_flag == true)
                {
                    ROS_INFO("Finding the path with RS curve failed!");
                }
                else
                {
                    ROS_INFO("Find the path with RS curve successfully!");
                    return recallPath(curr_s, curr_g);
                }
            }
            if ((type == Forwardway) || ((type == Bothway) && (forward_flag == true)))
            {
                child_s_list.clear();
                child_s_list = vehicle.kinematicsExplore(curr_s->pose, grid_map);
                // std::cout << "forward expansion size:" << child_s_list.size() << std::endl;
                for (int i = 0; i < child_s_list.size(); i++)
                {
                    temp_col = int(child_s_list[i].first.x / resolution);
                    temp_row = int(child_s_list[i].first.y / resolution);
                    child_s = node3d_map[temp_col][temp_row];
                    if (child_s->close != UnClose) //该节点已被拓展跳过该节点
                        continue;
                    if (child_s->open == UnOpen) //该节点未被拓展且未在队列中
                    {
                        child_s->pose = child_s_list[i].first;
                        child_s->o = child_s_list[i].second.first;
                        child_s->l = child_s_list[i].second.second;
                        child_s->g = caclG(curr_s, child_s, true);
                        child_s->h = caclH(child_s, curr_g);
                        child_s->f = child_s->g + gama * child_s->h;
                        child_s->pred = curr_s;
                        start_queue.push(child_s);
                        child_s->open = SOpen;
                    }
                    else if (child_s->open == SOpen) //该节点已在队列中
                    {
                        Node3dPointer temp = std::make_shared<Node3d>();
                        temp->pose = child_s_list[i].first;
                        temp->o = child_s_list[i].second.first;
                        temp->l = child_s_list[i].second.second;
                        temp->g = caclG(curr_s, temp, true);
                        temp->h = caclH(temp, curr_g);
                        temp->f = temp->g + gama * temp->h;
                        if (temp->f < child_s->f)
                        {
                            child_s->pose = temp->pose;
                            child_s->o = temp->o;
                            child_s->l = temp->l;
                            child_s->g = temp->g;
                            child_s->h = temp->h;
                            child_s->f = temp->f;
                            child_s->pred = curr_s;
                            std::priority_queue<Node3dPointer, Node3dList, nodeCompareByCost> temp_queue;
                            Node3dPointer temp_pointer;
                            while (start_queue.empty() == false) //由于此时优先队列中的元素发生改变需要对其进行重排
                            {
                                temp_pointer = start_queue.top();
                                start_queue.pop();
                                temp_queue.push(temp_pointer);
                            }
                            start_queue.swap(temp_queue);
                        }
                    }
                    else if (child_s->open == GOpen) //该节点已在反向队列中
                        continue;
                }
            }
            if ((type == Reverseway) || ((type == Bothway) && (reverse_flag == true)))
            {
                child_g_list.clear();
                child_g_list = vehicle.kinematicsExplore(curr_g->pose, grid_map);
                // std::cout << "reverse expansion size:" << child_g_list.size() << std::endl;
                for (int i = 0; i < child_g_list.size(); i++)
                {
                    temp_col = int(child_g_list[i].first.x / resolution);
                    temp_row = int(child_g_list[i].first.y / resolution);
                    child_g = node3d_map[temp_col][temp_row];
                    if (child_g->close != UnClose) //该节点已被拓展跳过该节点
                        continue;
                    if (child_g->open == UnOpen) //该节点未被拓展且未在队列中
                    {
                        child_g->pose = child_g_list[i].first;
                        child_g->o = child_g_list[i].second.first;
                        child_g->l = child_g_list[i].second.second;
                        child_g->g = caclG(curr_g, child_g, true);
                        child_g->h = caclH(child_g, curr_s);
                        child_g->f = child_g->g + gama * child_g->h;
                        child_g->pred = curr_g;
                        goal_queue.push(child_g);
                        child_g->open = SOpen;
                    }
                    else if (child_g->open == GOpen) //该节点已在队列中
                    {
                        Node3dPointer temp = std::make_shared<Node3d>();
                        temp->pose = child_g_list[i].first;
                        temp->o = child_g_list[i].second.first;
                        temp->l = child_g_list[i].second.second;
                        temp->g = caclG(curr_g, temp, true);
                        temp->h = caclH(temp, curr_s);
                        temp->f = temp->g + gama * temp->h;
                        if (temp->f < child_g->f)
                        {
                            child_g->pose = temp->pose;
                            child_g->o = temp->o;
                            child_g->l = temp->l;
                            child_g->g = temp->g;
                            child_g->h = temp->h;
                            child_g->f = temp->f;
                            child_g->pred = curr_g;
                            std::priority_queue<Node3dPointer, Node3dList, nodeCompareByCost> temp_queue;
                            Node3dPointer temp_pointer;
                            while (goal_queue.empty() == false) //由于此时优先队列中的元素发生改变需要对其进行重排
                            {
                                temp_pointer = goal_queue.top();
                                goal_queue.pop();
                                temp_queue.push(temp_pointer);
                            }
                            goal_queue.swap(temp_queue);
                        }
                    }
                    else if (child_g->open == SOpen) //该节点已在正向队列中
                        continue;
                }
            }
            // std::cout << "start queue size:" << start_queue.size() << std::endl;
            // std::cout << "goal queue size:" << goal_queue.size() << std::endl;
        }
        ROS_ERROR("Finish iterations, stop finding path!");
        return recallPath(curr_s, curr_g);
    }
}