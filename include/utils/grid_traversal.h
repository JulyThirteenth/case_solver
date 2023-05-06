#ifndef _GRID_TRAVERSAL_H_
#define _GRID_TRAVERSAL_H_

#include <cmath>
#include <cfloat>
#include <vector>
#include "../include/utils/algorithm_utils.h"

namespace TPCAP
{
    void gridTraversal(const Point& start, const Point& goal, const double resolution, std::vector<intPoint>& visited_grid);
}

#endif