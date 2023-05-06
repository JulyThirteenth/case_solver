#ifndef _PARAMS_H_
#define _PARAMS_H_

#include <cmath>

namespace TPCAP
{
    // vehicle params
    const static double FrontHang = 0.960;
    const static double WheelBase = 2.800;
    const static double RearHang = 0.929;
    const static double Width = 1.942;
    const static double MinRadius = 6.;
    const static double VMax = 2.5;
    const static double FiMax = 0.75;
    // astar algorithm
    const static double AstarGama = 1.;
    const static double AstarEta = 10.;
    const static double AstarThreshold = 4.;
    const static double AstarResolution = 0.5;
    // collision check
    const static double CollisionCheck_T = 0.1;
    // kinematics explore
    const static double Level1_T = 1;
    const static double Level2_T = 0.5;
    const static double Level3_T = 0.2;
    const static double Sample_T = 0.01;
    const static int SearchDirNums = 6; // 6 or 10
    const static int ExploreLevelNums = 3;
    enum Level
    {
        Level1,
        Level2,
        Level3
    };
    const static double LevelArr[3] = {Level1_T, Level2_T, Level3_T};
    const static int ExploreDirNums = 10;
    enum Operation
    {
        LeftForward,
        HalfLeftForward,
        Forward,
        HalfRightForward,
        RightForward,
        Nop,
        RightBack,
        HalfRightBack,
        Back,
        HalfLeftBack,
        LeftBack
    };
    const static double OpearationArr[11][2] = 
    {
        {VMax, FiMax}, 
        {VMax, FiMax / 2}, 
        {VMax, 0}, 
        {VMax, -FiMax / 2}, 
        {VMax, -FiMax}, 
        {0, 0}, 
        {-VMax, -FiMax}, 
        {-VMax, -FiMax / 2}, 
        {-VMax, 0}, 
        {-VMax, FiMax / 2}, 
        {-VMax, FiMax}
    };
    enum SearchType
    {
        Forwardway,
        Reverseway,
        Bothway
    };
    const static SearchType HybridSearchType = Bothway;
    const static double HybridGama = 1.;
    const static int HybridInterations = 2000;
    const static int HybridNrs = 10;
    const static int HybridNes = 2;
    const static double HybridCDP = 1.75; //变向惩罚
    const static double HybridRP = 1.75; //倒车惩罚
    // smooth path params
    const static double WeightData = 0.1;
    const static double WeightSmooth = 0.45;
    const static double Tolerance = 0.05;
    // recording params
    const static double DiscreteT = 0.1; //单位s
    const static double DiscreteD = 0.05; //单位m
    // visualize params
    const static int Skip = 20;
} // params.h

#endif