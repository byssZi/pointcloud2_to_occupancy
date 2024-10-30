#include <ros/ros.h>
#include "pointcloud2_to_occupancy/occupancy_grid_generator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "occupancy_grid_generator");
    ros::NodeHandle nh;

    OccupancyGridGenerator occupancy_grid_generator(nh);

    ros::spin();
    return 0;
}
