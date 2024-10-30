***
# 说明
这是一个接收激光雷达非地面点云，毫米波雷达聚类点云，生成占据栅格地图的工程。</br>
接收点云格式均为sensor_msgs::PointCloud2,发布栅格地图格式为nav_msgs::OccupancyGrid。
***
# Step1
根据激光雷达非地面点云话题，毫米波雷达聚类点云话题名称调整`launch/run.launch`文件里的话题名称</br>
根据实际情况调整`include/pointcloud2_to_occupancy/types.h`下的`FreeSpaceMapParams`栅格地图参数。
***
# Step2
```bash
catkin_make
source devel/setup.bash
roslaunch pointcloud2_to_occupancy run.launch
```
***
# Step3
打开Rivz观察栅格地图结果
***