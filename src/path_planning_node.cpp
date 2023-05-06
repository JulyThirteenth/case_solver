#include "../include/case_solver/case_path/path_planning.h"
#include "../include/case_solver/case_path/path_recording.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planning_node");
    TPCAP::PathPlanning pp;
    TPCAP::PathRecording pr;
    while(ros::ok())
    {
        pp.pubHybridAstarPath();
        ros::spinOnce();
    }
    
    return 0;
}