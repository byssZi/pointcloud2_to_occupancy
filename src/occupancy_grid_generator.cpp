#include "pointcloud2_to_occupancy/occupancy_grid_generator.h"

OccupancyGridGenerator::OccupancyGridGenerator(ros::NodeHandle& nh)
{
    ros::NodeHandle private_nh("~");
    std::string radar_points_topic;
    std::string lidar_points_topic;
    std::string occupancy_topic;

    ROS_ASSERT(private_nh.getParam("radar_points_topic", radar_points_topic));
    ROS_ASSERT(private_nh.getParam("lidar_points_topic", lidar_points_topic));
    ROS_ASSERT(private_nh.getParam("occupancy_topic", occupancy_topic));

    lidar_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(lidar_points_topic, 1, &OccupancyGridGenerator::lidarCallback, this);
    radar_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(radar_points_topic, 1, &OccupancyGridGenerator::radarCallback, this);
    occupancy_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(occupancy_topic, 1);

    lidar_received_ = false;
    radar_received_ = false;

}

OccupancyGridGenerator::~OccupancyGridGenerator() = default;

void OccupancyGridGenerator::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (!lidar_received_) {
        pcl::PointCloud<pcl::PointXYZ> lidar_cloud;
        pcl::fromROSMsg(*msg, lidar_cloud);
        merged_cloud_ += lidar_cloud;
        lidar_received_ = true; // 标记激光雷达数据已接收
        calFreeSpaceHandler(merged_cloud_);
    }

}

void OccupancyGridGenerator::radarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (!radar_received_) {
        pcl::PointCloud<pcl::PointXYZ> radar_cloud;
        pcl::fromROSMsg(*msg, radar_cloud);
        merged_cloud_ += radar_cloud;
        radar_received_ = true; // 标记毫米波雷达数据已接收
        calFreeSpaceHandler(merged_cloud_);
    }
}

void OccupancyGridGenerator::calFreeSpaceHandler(pcl::PointCloud<pcl::PointXYZ> cloud_nonground)
{
    if(lidar_received_ && radar_received_)
    {
        // 进行道路空闲可行驶区域点云计算
        common::Tictoc time_freespace;

        // 去除离群点
        pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
        voxelFilter.setLeafSize(0.1, 0.1, 0.1);
        voxelFilter.setMinimumPointsNumberPerVoxel(0);
        
        // 使用传入的副本进行滤波
        pcl::PointCloud<pcl::PointXYZ> cloud; // 创建一个新的点云对象来存储滤波后的结果
        voxelFilter.setInputCloud(cloud_nonground.makeShared()); // 将副本转换为共享指针
        voxelFilter.filter(cloud);

        // 使用 radius outlier removal 进行离群点过滤
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud.makeShared()); // 使用滤波后的点云
        outrem.setRadiusSearch(param_localMap_.radius);
        outrem.setMinNeighborsInRadius(param_localMap_.number);
        
        pcl::PointCloud<pcl::PointXYZ> filtered_cloud; // 创建一个新的点云对象来存储滤波后的结果
        outrem.filter(filtered_cloud); // 过滤后的点云将存储在 filtered_cloud 中

        ///进行道路空闲可行驶区域栅格地图计算
 

        Eigen::MatrixXi freeGridMap = Eigen::MatrixXi::Zero(param_localMap_.length * 2 / param_localMap_.resolution,
                                                            param_localMap_.length * 2 /param_localMap_.resolution);
        genLocalFreeMap::FreeSpaceMapHandler(filtered_cloud, param_localMap_, freeGridMap);

        publishFreeSpaceGridMap(freeGridMap);

        cout << "freespace time cost: " << time_freespace.toc() << endl;
        merged_cloud_.clear();
        lidar_received_ = false;
        radar_received_ = false;
    }
}

void OccupancyGridGenerator::publishFreeSpaceGridMap(Eigen::MatrixXi freeSpaceGridMap)
{
    nav_msgs::OccupancyGrid rosMap;

    rosMap.info.resolution = param_localMap_.resolution;
    rosMap.info.origin.position.x = - param_localMap_.length;
    rosMap.info.origin.position.y = - param_localMap_.length;
    rosMap.info.origin.position.z = 0.0;
    //注意占据栅格地图的坐标系与ros右手坐标系相差180度的旋转
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);// Y X Z
    rosMap.info.origin.orientation.x = q.x();
    rosMap.info.origin.orientation.y = q.y();
    rosMap.info.origin.orientation.z = q.z();//-1.0;
    rosMap.info.origin.orientation.w = q.w();
        // rosMap.info.origin.orientation = tf::createQuaternionMsgFromYaw(-3.14/2);

    rosMap.info.width = param_localMap_.length * 2 / param_localMap_.resolution;
    rosMap.info.height = param_localMap_.length * 2 / param_localMap_.resolution;
    int data_size = rosMap.info.width * rosMap.info.height;
    rosMap.data.resize(data_size);

    size_t k = 0;
    for(size_t c = 0; c < freeSpaceGridMap.cols(); c++)
    {
        for(size_t r = 0; r < freeSpaceGridMap.rows(); r++)
        {
            // rosMap.data[k] = k % 256;
            if (freeSpaceGridMap(r,c) == 0)
            {
                rosMap.data[k] = 0;
            }
            else if (freeSpaceGridMap(r,c) == 1)
            {
                rosMap.data[k] = 1;
            }
            else if (freeSpaceGridMap(r,c) == 2)
            {
                rosMap.data[k] = 25;
            }
            else if (freeSpaceGridMap(r,c) == 3)
            {
                rosMap.data[k] = 100;
            }
            else if (freeSpaceGridMap(r,c) == 5)
            {
                rosMap.data[k] = 150;
            }
            else if (freeSpaceGridMap(r,c) == 10)
            {
                rosMap.data[k] = 200;
            }
            else if (freeSpaceGridMap(r,c) == 11)
            {
                rosMap.data[k] = 200;
            }
            else if (freeSpaceGridMap(r,c) == 12)
            {
                rosMap.data[k] = 200;
            }
            else if (freeSpaceGridMap(r,c) == 13)
            {
                rosMap.data[k] = 200;
            }
            k++;
        }
    }

    rosMap.header.stamp = ros::Time::now();
    rosMap.header.frame_id = "ego_vehicle/lidar";

    occupancy_grid_pub_.publish(rosMap);
    ROS_INFO("pubFreeSpaceMap!");
}


