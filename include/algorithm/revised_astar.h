#ifndef _REVISED_ASTAR_H_
#define _REVISED_ASTAR_H_

#include <queue>
#include <vector>
#include <iostream>

#include "../include/algorithm/params.h"
#include "../include/utils/grid_traversal.h"
#include "../include/utils/algorithm_utils.h"
#include "../include/case_solver/case_map/csv_to_gridmap.h"

namespace TPCAP
{
    class RevisedAstar
    {
    public:
        RevisedAstar();
        AstarPath findPath(Vector &start, Vector &goal); //在内部节点地图上执行AStar算法
    private:
        void init(); //初始化内部节点地图
        void calcP(intPoint &grid);
        double calcH(intPoint &curr, intPoint &goal) { return std::abs(curr.x - goal.x) + std::abs(curr.y - goal.y); }
    private:
        double gama;
        double eta;
        double threshold;
        double resolution;
        Node2dMap node2d_map;
        bool initial_flag;
    };
}

#endif