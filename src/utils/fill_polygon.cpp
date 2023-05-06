#include "../include/utils/fill_polygon.h"

namespace TPCAP
{
    //输入：多边形顶点坐标、地图的分辨率以及用来保存多边形覆盖栅格的数组
    //输出：保存有多边形覆盖栅格的数组
    void fillPolygon(const std::vector<Point> &vertexes, double resolution, std::vector<intPoint> &polygArea)
    {
        if (vertexes.size() < 3)
            return;
        std::vector<intPoint> intVertexes;
        for (int i = 0; i < vertexes.size(); i++)
        {
            intPoint intVertex = {(int)(vertexes[i].x / resolution), (int)(vertexes[i].y / resolution)};
            intVertexes.push_back(intVertex);
        }
        std::vector<int> tempY;
        int minY, maxY, scanLineNums;
        for (int i = 0; i < intVertexes.size(); i++)
        {
            tempY.push_back(intVertexes[i].y);
        }
        minY = *std::min_element(tempY.begin(), tempY.end());
        maxY = *std::max_element(tempY.begin(), tempY.end());
        scanLineNums = maxY - minY + 1;
        /*建立边表*/
        EdgeTable ET = new Edge *[scanLineNums];
        for (int y = 0; y < scanLineNums; y++)
        {
            ET[y] = new Edge;
            ET[y]->next = NULL;
        }
        intPoint predP, currP1, currP2, nextP;
        /*初始化边表*/
        for (int i = 0; i < intVertexes.size(); i++)
        {
            currP1.y = intVertexes[i].y;
            currP2.y = intVertexes[(i + 1) % intVertexes.size()].y;
            if (currP1.y == currP2.y) //舍弃平行X轴的边
                continue;
            currP1.x = intVertexes[i].x;
            currP2.x = intVertexes[(i + 1) % intVertexes.size()].x;
            predP.x = intVertexes[(i - 1 + intVertexes.size()) % intVertexes.size()].x;
            predP.y = intVertexes[(i - 1 + intVertexes.size()) % intVertexes.size()].y;
            nextP.x = intVertexes[(i + 2) % intVertexes.size()].x;
            nextP.y = intVertexes[(i + 2) % intVertexes.size()].y;
            int ymin = std::min(currP1.y, currP2.y);
            int ymax = std::max(currP1.y, currP2.y);
            double x = currP1.y > currP2.y ? currP2.x : currP1.x;
            double dx = (double)(currP1.x - currP2.x) / (double)(currP1.y - currP2.y);
            if (((currP2.y >= currP1.y) && (currP1.y >= predP.y)) || ((currP1.y >= currP2.y) && (currP2.y >= nextP.y)))
            {
                ymin++;
                x += dx;
            }
            Edge *tempE = new Edge;
            tempE->ymax = ymax;
            tempE->x = x;
            tempE->dx = dx;
            tempE->next = ET[ymin - minY]->next;
            ET[ymin - minY]->next = tempE;
        }
        /*建立活动边表*/
        ActiveEdgeTable AET = new Edge;
        AET->next = NULL;
        /*扫描线扫描*/
        for (int y = minY; y < maxY + 1; y++)
        {
            /*取出ET中当前扫描行的所有边并按x的递增顺序（若x相等则按dx的递增顺序）插入AET*/
            while (ET[y - minY]->next)
            {
                Edge *tempE = ET[y - minY]->next;
                Edge *tempAET = AET;
                while (tempAET->next)
                {
                    if ((tempE->x > tempAET->next->x) || ((tempE->x == tempAET->next->x) && (tempE->dx > tempAET->next->dx)))
                    {
                        tempAET = tempAET->next;
                        continue;
                    }
                    break;
                }
                ET[y - minY]->next = tempE->next;
                tempE->next = tempAET->next;
                tempAET->next = tempE;
            }
            /*将AET中的边两两配对并将中间点添加到polygArea中*/
            Edge *tempAET = AET;
            while (tempAET->next && tempAET->next->next)
            {
                for (int x = tempAET->next->x; x < tempAET->next->next->x; x++)
                {
                    intPoint occ = {x, y};
                    polygArea.push_back(occ);
                }
                tempAET = tempAET->next->next;
            }
            /*删除AET中满足y=ymax的边*/
            tempAET = AET;
            while (tempAET->next)
            {
                if (tempAET->next->ymax == y)
                {
                    Edge *tempE = tempAET->next;
                    tempAET->next = tempE->next;
                    tempE->next = NULL;
                    delete tempE;
                }
                else
                {
                    tempAET = tempAET->next;
                }
            }
            /*更新AET中边的x值，进入下一循环*/
            tempAET = AET;
            while (tempAET->next)
            {
                tempAET->next->x += tempAET->next->dx;
                tempAET = tempAET->next;
            }
        }
        /*释放边表内存*/
        for (int y = 0; y < scanLineNums; y++)
        {
            while (ET[y]->next)
            {
                Edge *next = ET[y]->next->next;
                delete ET[y]->next;
                ET[y]->next = next;
            }
            delete ET[y];
        }
        delete[] ET;
        /*释放活动边表内存*/
        while (AET)
        {
            Edge *next = AET->next;
            delete AET;
            AET = next;
        }
    }

}