#include "../include/case_solver/case_map/csv_to_occmap.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "csv_to_occmap_node");
    
    TPCAP::CsvToOccmap cto;
    while(ros::ok())
    {
        cto.pubMapAndShift();
    }

    return 0;
}