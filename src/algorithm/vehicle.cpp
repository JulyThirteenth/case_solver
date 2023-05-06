#include "../include/algorithm/vehicle.h"

namespace TPCAP
{
    bool Vehicle::collisionCheck(const Vector &pose, GridMap &grid_map)
    {
        std::vector<Point> polyg_vertexes;
        Point polyg_vertex;
        double t = M_PI_2 - pose.t;
        polyg_vertex.x = (-vehicle_width / 2) * cos(t) + (front_hang + wheel_base) * sin(t) + pose.x;
        polyg_vertex.y = (vehicle_width / 2) * sin(t) + (front_hang + wheel_base) * cos(t) + pose.y;
        polyg_vertexes.push_back(polyg_vertex);
        polyg_vertex.x = (vehicle_width / 2) * cos(t) + (front_hang + wheel_base) * sin(t) + pose.x;
        polyg_vertex.y = (-vehicle_width / 2) * sin(t) + (front_hang + wheel_base) * cos(t) + pose.y;
        polyg_vertexes.push_back(polyg_vertex);
        polyg_vertex.x = (vehicle_width / 2) * cos(t) + (-rear_hang) * sin(t) + pose.x;
        polyg_vertex.y = (-vehicle_width / 2) * sin(t) + (-rear_hang) * cos(t) + pose.y;
        polyg_vertexes.push_back(polyg_vertex);
        polyg_vertex.x = (-vehicle_width / 2) * cos(t) + (-rear_hang) * sin(t) + pose.x;
        polyg_vertex.y = (vehicle_width / 2) * sin(t) + (-rear_hang) * cos(t) + pose.y;
        polyg_vertexes.push_back(polyg_vertex);
        std::vector<intPoint> polyg_area;
        fillPolygon(polyg_vertexes, grid_map.resolution, polyg_area);
        for (int i = 0; i < polyg_area.size(); i++)
        {
            if (!grid_map.isInMap(polyg_area[i].x, polyg_area[i].y))
            {
                continue;
            }
            if (grid_map.occs[polyg_area[i].x][polyg_area[i].y] == true)
            {
                return true;
            }
        }
        return false;
    }

    ChildNodeList Vehicle::kinematicsExplore(const Vector &pose, GridMap &grid_map)
    {
        ChildNodeList child_nodes;
        double simulator_T;
        double simulator_V;
        double simulator_Fi;
        Vector curr_pose;
        Vector temp_pose;
        for (int o = 0; o <= ExploreDirNums;)
        {
            if (Operation(o) == Nop)
            {
                o++;
                continue;
            }
            simulator_V = OpearationArr[o][0];
            simulator_Fi = OpearationArr[o][1];
            for (int ol = 0; ol < ExploreLevelNums; ol++)
            {
                simulator_T = LevelArr[ol];
                if (simulator_Fi == 0)
                {
                    curr_pose.t = pose.t;
                    curr_pose.x = pose.x + simulator_V * cos(pose.t) * simulator_T;
                    curr_pose.y = pose.y + simulator_V * sin(pose.t) * simulator_T;
                }
                else
                {
                    double parameter = simulator_V * tan(simulator_Fi) / (WheelBase);
                    curr_pose.t = pose.t + parameter * simulator_T;
                    curr_pose.x = pose.x + simulator_V / parameter * (sin(curr_pose.t) - sin(pose.t));
                    curr_pose.y = pose.y + simulator_V / parameter * (cos(pose.t) - cos(curr_pose.t));
                }
                bool is_collision = false;
                if (grid_map.isInMap(curr_pose.x, curr_pose.y))
                {
                    if (collisionCheck(curr_pose, grid_map))
                    {
                        is_collision = true;
                    }
                    else
                    {
                        for (double st = CollisionCheck_T; st < simulator_T; st += CollisionCheck_T)
                        {
                            if (simulator_Fi == 0)
                            {
                                temp_pose.t = curr_pose.t;
                                temp_pose.x = curr_pose.x + simulator_V * cos(curr_pose.t) * st;
                                temp_pose.y = curr_pose.y + simulator_V * sin(curr_pose.t) * st;
                            }
                            else
                            {
                                double parameter = simulator_V * tan(simulator_Fi) / (WheelBase);
                                temp_pose.t = curr_pose.t + parameter * st;
                                temp_pose.x = curr_pose.x + simulator_V / parameter * (sin(temp_pose.t) - sin(curr_pose.t));
                                temp_pose.y = curr_pose.y + simulator_V / parameter * (cos(curr_pose.t) - cos(temp_pose.t));
                            }
                            if (collisionCheck(temp_pose, grid_map))
                            {
                                is_collision = true;
                                break;
                            }
                        }
                    }
                }
                else
                    continue;
                if (is_collision == false)
                {
                    child_nodes.push_back(std::make_pair(curr_pose, std::make_pair(Operation(o), Level(ol))));
                    break;
                }
            }
            if (SearchDirNums == 6)
                o += 2;
            else
                o++;
        }
        return child_nodes;
    }
}