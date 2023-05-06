#ifndef _ALGORITHM_UTILS_H_
#define _ALGORITHM_UTILS_H_

#include <vector>
#include <memory>
#include "../include/algorithm/params.h"

namespace TPCAP
{
    template <typename T>
    struct TemplatePoint
    {
        T x, y;
        TemplatePoint(T x_, T y_) : x(x_), y(y_) {}
        TemplatePoint() : x(0), y(0) {}
    };
    template <typename T>
    bool operator==(const struct TemplatePoint<T> &l, const struct TemplatePoint<T> &r)
    {
        return (l.x == r.x) && (l.y == r.y);
    }
    typedef struct TemplatePoint<double> Point;
    typedef struct TemplatePoint<int> intPoint;

    template <typename T1, typename T2>
    struct TemplateVector
    {
        T1 x, y;
        T2 t;
        TemplateVector(T1 x_, T1 y_, T2 t_) : x(x_), y(y_), t(t_) {}
        TemplateVector() : x(0), y(0), t(0) {}
        struct TemplateVector<T1,T2>& operator = (const struct TemplateVector<T1,T2> &vec)
        {
            if(this != &vec)
            {
                this->x = vec.x;
                this->y = vec.y;
                this->t = vec.t;
            }
            return *this;
        }
    };

    typedef struct TemplateVector<double, double> Vector;

    typedef struct node2d
    {
        intPoint id;
        bool o;
        bool c;
        double f;
        double g;
        double h;
        double p;
        std::shared_ptr<struct node2d> pred;
        node2d() : id(), o(false), c(false), f(0), g(0), h(0), p(0), pred(nullptr) {}
    } Node2d;
    typedef std::shared_ptr<Node2d> Node2dPointer;
    typedef std::vector<Node2dPointer> Node2dList;
    typedef std::vector<Node2dList> Node2dMap;

    enum OpenStatus
    {
        SOpen,
        GOpen,
        UnOpen
    };
    enum CloseStatus
    {
        SClose,
        GClose,
        UnClose
    };
    typedef struct node3d
    {
        intPoint id;
        Vector pose;
        OpenStatus open;
        CloseStatus close;
        Level l;
        Operation o;
        double f;
        double g;
        double h;
        std::shared_ptr<struct node3d> pred;
        node3d() : id(), pose(), open(UnOpen), close(UnClose), 
            l(Level1), o(Nop), f(0), g(0), h(0), pred(nullptr) {}
    } Node3d;
    typedef std::shared_ptr<Node3d> Node3dPointer;
    typedef std::vector<Node3dPointer> Node3dList;
    typedef std::vector<Node3dList> Node3dMap;

    struct nodeCompareByCost
    {
        bool operator()(Node2dPointer &l, Node2dPointer &r)
        {
            return l->f > r->f;
        }
        bool operator()(Node3dPointer &l, Node3dPointer &r)
        {
            return l->f > r->f;
        }
    };

    class GridMap
    {
    public:
        int cols;
        int rows;
        double resolution;
        std::vector<std::vector<bool>> occs;
        inline bool isInMap(const int col, const int row) { return (col >= 0 && col < cols) && (row >= 0 && row < rows); }
        inline bool isInMap(const double x, const double y) { return isInMap((int)(x / resolution), (int)(y / resolution)); }
    };

    typedef struct astarPath
    {
        std::vector<Point> path;
        double length = -1;
    } AstarPath;
}

#endif