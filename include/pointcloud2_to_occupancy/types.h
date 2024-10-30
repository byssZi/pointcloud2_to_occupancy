//
// Created by laoli on 2021/12/7.
//
/*
 * 本文件描述pcl点云的类型名称以及程序运行的相关参数
 */

#ifndef COMMON_INCLUDE_TYPES_H_
#define COMMON_INCLUDE_TYPES_H_

#include <Eigen/Core>
#include <string>
#include <tuple>
#include <vector>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"


//用于可通行区域检测中的栅格/像素状态检测
static int filter_x[28]={-1,0,1,-3,-2,2,3,-4,4,-4,4,-5,5,-5,5,-5,5,-1,0,1,-3,-2,2,3,-4,4,-4,4};
static int filter_y[28]={-5,-5,-5,-4,-4,-4,-4,-3,-3,-2,-2,-1,-1,0,0,1,1,5,5,5,4,4,4,4,3,3,2,2};
static int all_x[89]={-1,0,1,
                      -3,-2,-1,0,1,2,3,
                      -4,-3,-2,-1,0,1,2,3,4,
                      -4,-3,-2,-1,0,1,2,3,4,
                      -5,-4,-3,-2,-1,0,1,2,3,4,5,
                      -5,-4,-3,-2,-1,0,1,2,3,4,5,
                      -5,-4,-3,-2,-1,0,1,2,3,4,5,
                      -1,0,1,
                      -3,-2-1,0,1,2,3,
                      -4,-3,-2,-1,0,1,2,3,4,
                      -4,-3,-2,-1,0,1,2,3,4};
static int all_y[89]={-5,-5,-5,
                      -4,-4,-4,-4,-4,-4,-4,
                      -3,-3,-3,-3,-3,-3,-3,-3,-3,
                      -2,-2,-2,-2,-2,-2,-2,-2,-2,
                      -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
                      0,0,0,0,0,0,0,0,0,0,0,
                      1,1,1,1,1,1,1,1,1,1,1,
                      5,5,5,
                      4,4,4,4,4,4,4,
                      3,3,3,3,3,3,3,3,3,
                      2,2,2,2,2,2,2,2,2};

static int smaller_filter_x[16] = {-1, 0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2};
static int smaller_filter_y[16] = {3, 3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1, 0, 1, 2};
static int smaller_all_x[37] = {-1, 0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2,
                                                            -1, 0, 1, 2, 2, 2, 1, 0, -1, -2, -2, -2,
                                                            -1, 0, 1, 1, 1, 0, -1, -1,
                                                            0};
static int smaller_all_y[37] = {3, 3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1, 0, 1, 2,
                                                            2, 2, 2, 1, 0, -1, -2, -2, -2, -1, 0, 1,
                                                            1, 1, 1, 0, -1, -1, -1, 0,
                                                            0};

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;


    /*------------------------------ Parameter type
     * ------------------------------*/
struct FreeSpaceMapParams
{
    //离群点去除
    double radius = 1;
    int number = 5;
    //地图分辨率大小
    double resolution = 0.25;
    //右手参考系，激光雷达有效扫描区域角度范围
    double scan_angle_r = -60.0;
    double scan_angle_l = 60.0;
    //地图的长与宽 尽量选择正方形区域
    int length = 100;
    int width = 100;

    //需要采集的非地面最近激光点的数目，为360的整数倍
    int pointNum = 360;

    //地图中心偏置
    float offset_x = length / 2;
    float offset_y = width / 2;

    //斜坡检测坡度范围
    float angle_min = 10.0;
    float angle_max = 45.0;
};

#endif //_LIDAR_PERCEPTION_PCL_TYPES_H_
