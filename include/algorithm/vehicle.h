#ifndef _VEHICLE_H_
#define _VEHICLE_H_

#include <iostream>
#include "params.h"
#include "../include/utils/fill_polygon.h"
#include "../include/utils/algorithm_utils.h"

namespace TPCAP
{
    typedef std::vector<std::pair<Vector, std::pair<Operation, Level>>> ChildNodeList;
    class Vehicle
    {
    private:
        double front_hang;
        double rear_hang;
        double wheel_base;
        double vehicle_width;
    public:
        Vehicle(double fh = FrontHang, double rh = RearHang, double wb = WheelBase, double vw = Width) : 
            front_hang(fh), rear_hang(rh), wheel_base(wb), vehicle_width(vw) {}
        bool collisionCheck(const Vector &pose, GridMap &grid_map);
        ChildNodeList kinematicsExplore(const Vector &pose, GridMap &gMap);
    };
}

#endif // vehicle.h