#ifndef OCCUPANCY_GRID_GENERATOR_H
#define OCCUPANCY_GRID_GENERATOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include "freeSpaceGridMap.h"
#include <cmath> // for exp()

class OccupancyGridGenerator
{
public:
    OccupancyGridGenerator(ros::NodeHandle& nh);
    ~OccupancyGridGenerator();

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void radarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void calFreeSpaceHandler(pcl::PointCloud<pcl::PointXYZ> cloud_nonground);
    void publishFreeSpaceGridMap(Eigen::MatrixXi freeSpaceGridMap);
    
private:


    ros::Subscriber lidar_sub_;
    ros::Subscriber radar_sub_;
    ros::Publisher occupancy_grid_pub_;
    pcl::PointCloud<pcl::PointXYZ> merged_cloud_;
    bool lidar_received_;
    bool radar_received_;
    FreeSpaceMapParams param_localMap_;

};

#endif // OCCUPANCY_GRID_GENERATOR_H
