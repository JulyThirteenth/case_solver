#ifndef _HYBRID_ASTAR_H_
#define _HYBRID_ASTAR_H_

#include <queue>
#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "params.h"
#include "vehicle.h"
#include "reeds_shepp.h"
#include "../include/utils/algorithm_utils.h"
#include "../include/algorithm/revised_astar.h"

namespace TPCAP
{
    typedef std::vector<Vector> HybridAstarPath;
    class HybridAstar
    {
    public:
        //碰撞检测与节点拓展对象
        Vehicle vehicle;
        // Revised Astar 算法对象
        RevisedAstar revised_astar;
        // Reeds Shepp 算法对象
        ReedsSheppStateSpace r;
        ReedsSheppStateSpace::ReedsSheppPath rr;

    private:
        //算法参数
        double gama;
        int iterations;
        int nrs;
        int nes;
        double cdp;
        double rp;
        SearchType type;
        //可视化
        ros::NodeHandle nh;
        ros::Publisher pub_curr_start;
        ros::Publisher pub_curr_goal;

    public:
        HybridAstar();
        HybridAstarPath findPath(Vector &start, Vector &goal, GridMap &grid_map);

    private:
        double caclG(const Node3dPointer &pred, const Node3dPointer &curr, bool dir);
        double caclH(const Node3dPointer &curr, const Node3dPointer &goal);
        HybridAstarPath recallSegPath(const Node3dPointer &seg);
        HybridAstarPath recallPath(const Node3dPointer &curr_s, const Node3dPointer &curr_g);
        void visualizeCurr(const Node3dPointer &curr_s, const Node3dPointer &curr_g);
    };
}

#endif