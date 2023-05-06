#include "../include/algorithm/revised_astar.h"

namespace TPCAP
{
    RevisedAstar::RevisedAstar()
    {
        gama = AstarGama;
        eta = AstarEta;
        threshold = AstarThreshold;
        resolution = AstarResolution;
        init();
    }
    void RevisedAstar::init()
    {
        CsvToGridMap ctg(resolution);
        GridMap grid_map;
        if (ctg.getGridMap(grid_map))
        {
            node2d_map.resize(grid_map.cols);
            for (int c = 0; c < grid_map.cols; c++)
            {
                node2d_map[c].resize(grid_map.rows);
                for (int r = 0; r < grid_map.rows; r++)
                {
                    if (!grid_map.occs[c][r]) //只对栅格地图中空闲的栅格初始化对应节点以节省内存
                    {
                        node2d_map[c][r] = std::make_shared<Node2d>();
                        node2d_map[c][r]->id = {c, r};
                    }
                }
            }
            //计算简化人工势场
            std::vector<intPoint> side_grids;
            Point curr, next;
            for (int i = 0; i < ctg.map_info.obstacles.size(); i++)
            {
                for (int j = 0; j < ctg.map_info.obstacles[i].size(); j++)
                {
                    curr = ctg.map_info.obstacles[i][j];
                    next = ctg.map_info.obstacles[i][(j + 1) % ctg.map_info.obstacles[i].size()];
                    side_grids.clear();
                    gridTraversal(curr, next, resolution, side_grids);
                    for (int k = 0; k < side_grids.size(); k++)
                    {
                        calcP(side_grids[k]);
                    }
                }
            }
            std::cout << "Revised Astar initialize successfully!" << std::endl;
            initial_flag = true;
        }
        else
        {
            std::cout << "Revised Astar initialize failed!" << std::endl;
            initial_flag = false;
        }
    }

    void RevisedAstar::calcP(intPoint &grid) //计算在一个占用栅格影响下周围势的分布
    {
        int limit = (int)(threshold / resolution);
        double max_d2 = std::pow(threshold, 2);
        double temp;
        for (int count = 1; count <= limit; count++)
        {
            for (int dy = -count; dy < count + 1; dy++)
            {
                double d2 = std::pow(count, 2) + std::pow(dy, 2);
                if (d2 > max_d2)
                    continue;
                if (node2d_map[grid.x + count][grid.y + dy])
                {
                    temp = node2d_map[grid.x + count][grid.y + dy]->p;
                    node2d_map[grid.x + count][grid.y + dy]->p = std::max(temp, max_d2 - d2);
                }
                if (node2d_map[grid.x - count][grid.y + dy])
                {
                    temp = node2d_map[grid.x - count][grid.y + dy]->p;
                    node2d_map[grid.x - count][grid.y + dy]->p = std::max(temp, max_d2 - d2);
                }
            }
            for (int dx = -count + 1; dx < count; dx++)
            {
                double d2 = std::pow(count, 2) + std::pow(dx, 2);
                if (d2 > max_d2)
                    continue;
                if (node2d_map[grid.x + dx][grid.y + count])
                {
                    temp = node2d_map[grid.x + dx][grid.y + count]->p;
                    node2d_map[grid.x + dx][grid.y + count]->p = std::max(temp, max_d2 - d2);
                }
                if (node2d_map[grid.x + dx][grid.y - count])
                {
                    temp = node2d_map[grid.x + dx][grid.y - count]->p;
                    node2d_map[grid.x + dx][grid.y - count]->p = std::max(temp, max_d2 - d2);
                }
            }
        }
    }

    AstarPath RevisedAstar::findPath(Vector &start, Vector &goal)
    {
        AstarPath astar_path;
        if (initial_flag == false) //未初始化内部节点地图
            return astar_path;
        //初始化内部节点地图信息
        // std::cout << "Begin init node2d map!" << std::endl;
        for (int x = 0; x < node2d_map.size(); x++)
        {
            for (int y = 0; y < node2d_map[x].size(); y++)
            {
                if (node2d_map[x][y]) //只对空闲状态节点进行信息初始化
                {
                    node2d_map[x][y]->c = 0;
                    node2d_map[x][y]->g = 0;
                    node2d_map[x][y]->h = 0;
                    node2d_map[x][y]->o = false;
                    node2d_map[x][y]->c = false;
                    node2d_map[x][y]->pred = nullptr;
                }
            }
        }
        // std::cout << "Finish init node2d map!" << std::endl;
        //将起始点坐标转换为节点地图中的节点
        // std::cout << "Begin init start & goal node2d!" << std::endl;
        int s_col = int(start.x / resolution);
        int s_row = int(start.y / resolution);
        // std::cout << s_col << " " << s_row << std::endl;
        Node2dPointer s = node2d_map[s_col][s_row];
        if (s == nullptr) //由于分辨率不同可能导致起点落在占用栅格上需要对其进行纠正
        {
            // to do
            std::cout << "s is nullptr!" << std::endl;
            int temp_col;
            int temp_row;
            for(int i=-1; i<2; i++)
            {
                for(int j=-1; j<2; j++)
                {
                    temp_col = s_col + i;
                    temp_row = s_row + j;
                    if(temp_col < 0 || temp_col > node2d_map.size()-1)
                        continue;
                    if(temp_row < 0 || temp_row > node2d_map[0].size()-1)
                        continue;
                    s = node2d_map[temp_col][temp_row];
                    if(s != nullptr)
                        break;
                }
                if(s != nullptr)
                    break;
            }
        }
        int g_col = int(goal.x / resolution);
        int g_row = int(goal.y / resolution);
        // std::cout << g_col << " " << g_row << std::endl;
        Node2dPointer g = node2d_map[g_col][g_row];
        if (g == nullptr) //由于分辨率不同可能导致终点落在占用栅格上需要对其进行纠正
        {
            // to do
            std::cout << "g is nullptr!" << std::endl;
            int temp_col;
            int temp_row;
            for(int i=-1; i<2; i++)
            {
                for(int j=-1; j<2; j++)
                {
                    temp_col = g_col + i;
                    temp_row = g_row + j;
                    if(temp_col < 0 || temp_col > node2d_map.size()-1)
                        continue;
                    if(temp_row < 0 || temp_row > node2d_map[0].size()-1)
                        continue;
                    g = node2d_map[temp_col][temp_row];
                    if(g != nullptr)
                        break;
                }
                if(g != nullptr)
                    break;
            }
        }
        // std::cout << "Finish init start & goal node2d!" << std::endl;
        //优先队列
        std::priority_queue<Node2dPointer, Node2dList, nodeCompareByCost> open_queue;
        //将起点放入优先队列
        // std::cout << "Begin init queue!" << std::endl;
        s->g = 0;
        s->h = calcH(s->id, g->id);
        s->f = s->g + gama * s->h;
        open_queue.push(s);
        s->o = true;
        // std::cout << "Finish init queue!" << std::endl;
        //定义curr, curr_child
        Node2dPointer curr, curr_child;
        int iter =0;
        while (!open_queue.empty())
        {
            iter++;
            curr = open_queue.top();
            open_queue.pop();
            curr->o = false;
            curr->c = true;
            for (int i = -1; i < 2; i++)
            {
                for (int j = 1; j > -2; j--)
                {
                    if (i == 0 && j == 0) //跳过当前节点
                        continue;
                    //判断当前节点是否在节点地图中
                    double child_x = curr->id.x + i;
                    if (child_x < 0 || child_x >= node2d_map.size())
                        continue;
                    double child_y = curr->id.y + j;
                    if (child_y < 0 || child_y >= node2d_map[child_x].size())
                        continue;
                    curr_child = node2d_map[child_x][child_y];
                    if (curr_child) //只对空闲状态节点进行拓展
                    {
                        if (curr_child->c == true)
                            continue;
                        double cost_g = curr->g + (((abs(i) + abs(j)) == 2) ? 1.414 : 1);
                        if (curr_child->id == g->id)
                        {
                            //回溯路径信息
                            curr_child->pred = curr;
                            curr_child->g = cost_g;
                            astar_path.length = curr_child->g * resolution;
                            Point pose;
                            while (curr_child)
                            {
                                pose.x = (curr_child->id.x + 0.5) * resolution;
                                pose.y = (curr_child->id.y + 0.5) * resolution;
                                astar_path.path.push_back(pose);
                                curr_child = curr_child->pred;
                            }
                            // std::cout<< iter <<std::endl;
                            return astar_path;
                        }
                        if (curr_child->o == false)
                        {
                            curr_child->g = cost_g;
                            curr_child->h = calcH(curr_child->id, g->id);
                            curr_child->f = curr_child->g + gama * curr_child->h + eta * curr_child->p;
                            curr_child->pred = curr;
                            open_queue.push(curr_child);
                            curr_child->o = true;
                        }
                        else
                        {
                            double cost_f = cost_g + gama * curr_child->h + eta * curr_child->p;
                            if (cost_f < curr_child->f)
                            {
                                curr_child->g = cost_g;
                                curr_child->f = cost_f;
                                curr_child->pred = curr;
                                std::priority_queue<Node2dPointer, Node2dList, nodeCompareByCost> temp_queue;
                                while (!open_queue.empty())
                                {
                                    curr = open_queue.top();
                                    open_queue.pop();
                                    temp_queue.push(curr);
                                }
                                open_queue.swap(temp_queue);
                            }
                        }
                    }
                }
            }
        }
    }
}