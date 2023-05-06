#include "../include/case_solver/case_traj/traj_planning.h"
#include "../include/case_solver/case_traj/traj_recording.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_planning_node");
    TPCAP::TrajPlanning tp;
    TPCAP::TrajRecording tr;
    while(ros::ok())
    {
        ros::spinOnce();
    }
    
    return 0;
}