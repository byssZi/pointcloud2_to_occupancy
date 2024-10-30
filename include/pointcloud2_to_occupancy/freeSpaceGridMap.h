
#ifndef LIDAR_PERCEPTION_FREE_SPACE_GRID_MAP_H
#define LIDAR_PERCEPTION_FREE_SPACE_GRID_MAP_H

#include <vector>
#include "types.h"
#include "common.h"

#include <algorithm>
using namespace std;

#define FREE_PI 3.14159265
//#define EDGE_PROCESS

namespace genLocalFreeMap{

    static void computeFreeSpacePoints(const PointCloud& pointCloudIn,float *free_space,
                                       int free_space_n = 360)
    {
        int thetaId;
        float distance_cur;
        size_t pointsNum = pointCloudIn.points.size();

        float alpha = free_space_n / 360;
        for(size_t pid = 0; pid < pointsNum; pid++) //遍历每一个非地面激光点
        {

            if(pointCloudIn.points[pid].z < 2.4)
            {
                distance_cur = std::pow(pointCloudIn.points[pid].x,2) + std::pow(pointCloudIn.points[pid].y,2);

            
                //atan2(y,x) x前方为0度，逆时针0～180，顺时针0～-180
                thetaId = int((atan2f(pointCloudIn.points[pid].y,pointCloudIn.points[pid].x) + FREE_PI) * 180.0 * alpha /FREE_PI  + 0.5); //当前激光点对应方向角度 × alpha = 当前激光点对应需要采集的非地面激光点id 
                thetaId = thetaId % free_space_n;
                if(free_space[thetaId] > distance_cur && distance_cur > 1)
                {
                    free_space[thetaId] = distance_cur; //得到最小距离
                }
            }
        }

    }

    static void freeGridMapFilter(float* freeSpacePoints,const FreeSpaceMapParams& mapParams,Eigen::MatrixXi &dst)
    {
        float pixel_size = mapParams.resolution;
        float delta_r = pixel_size * 0.75;
        float delta_d_in_r = pixel_size * 0.65;
        int maxlength = mapParams.length * 2 / pixel_size, maxwidth = mapParams.width/pixel_size;
        
        Eigen::MatrixXi src = Eigen::MatrixXi::Zero(maxlength, maxlength);

        int pointNum = mapParams.pointNum;
        float max_dis = mapParams.length;//(mapParams.length > mapParams.width ? mapParams.length : mapParams.width) / 2;
        float alpha = mapParams.pointNum / 360.0;
        float theta_border = FREE_PI / mapParams.pointNum * 1.2;
        std::vector<float> delta_t;
        for (float j = 0.0001; j < max_dis; j += delta_r) // Prepare the delta theta of different radius
        {
            delta_t.push_back(delta_d_in_r/j);//不同半径下，弧长分辨率对应的角度分辨率
        }

        
        for (int i = 0; i < pointNum; i++)//遍历每一个需要采集的非地面激光点
        {

         
            float r = min(freeSpacePoints[i], freeSpacePoints[(i + 1) % pointNum]);
            r = min(r, freeSpacePoints[(i - 1 + pointNum) % pointNum]);
            r = sqrt(r);                   
            int k = 0;
            for (float j = 0; j < r - 0.5; j += delta_r)
            {
                float dt = delta_t[k++];
                float theta = (i / alpha - 180)*FREE_PI/180.0;                   
       
                for (float t = theta - theta_border; t < theta + theta_border; t+=dt)
                {
                    float x = j*cos(t);
                    float y = j*sin(t);
                    int m = int((mapParams.length +x) / pixel_size);//int((mapParams.offset_x - x) / pixel_size);               
                    int n = int((mapParams.length + y) / pixel_size);//int((mapParams.offset_y - y) / pixel_size);
                    // if (m >= 0 && m < maxlength && n >= 0 && n < maxwidth) 
                      src(m, n) = 1;
#ifndef EDGE_PROCESS
                      dst(m, n) = 2;
#endif
                }
            }
        }
#ifdef EDGE_PROCESS
        for(int i = 0; i < mapParams.pointNum; i++)
        {
           
            float angle = (i / alpha - 180);
            if(angle < mapParams.scan_angle_r || angle > mapParams.scan_angle_l)
                continue;

    
            for(float j = 0; j < max_dis -1; j += delta_r)
            {
                float x = j * cos((i / alpha - 180) * FREE_PI /180.0);
                float y = j * sin((i / alpha - 180) * FREE_PI /180.0);
                int m = int((mapParams.length + x) / pixel_size);//int((mapParams.offset_x - x) / pixel_size);
                int n = int((mapParams.length + y) / pixel_size);//int((mapParams.offset_y - y) / pixel_size);
                int theta = int(atan2f(y, x) * 180.0 / FREE_PI + 180.0 + 0.5);
                theta = theta % pointNum;
                float r = std::min(freeSpacePoints[theta],freeSpacePoints[(theta+1) % pointNum]);
                r = std::min(r,freeSpacePoints[(theta-1+pointNum) % pointNum]);

                if(r > j*j +1)   
                {
                    int result = 0;
                    for(int k = 0; k < 16; k++)
                    {
                      // if ((m  + filter_x[k]) >= 0 && (m  + filter_x[k]) < maxlength && 
                      //     (n + filter_y[k]) >= 0 && (n + filter_y[k]) < maxwidth) 
                        result += src(m + smaller_filter_x[k], n + smaller_filter_y[k]);
                    }
                    if(result < 16)
                        break;
                    for (int k = 0; k < 37; k++)               
                    {
                      // if ((m + all_x[k]) >= 0 && (m + all_x[k]) < maxlength &&
                      //     (n + all_y[k]) >= 0 && (n + all_y[k]) < maxwidth)
                        dst(m+smaller_all_x[k], n+smaller_all_y[k]) = max(1, dst(m+smaller_all_x[k], n+smaller_all_y[k]));
                    }
                    dst(m,n) = 2;
                }
            }
        } 
#endif
    }

    static void FreeSpaceMapHandler(const PointCloud& pointCloudIn,const FreeSpaceMapParams& mapParams,
                             Eigen::MatrixXi &freeGridMap)
    {
      
        float *freeSpacePoints = (float*) calloc(mapParams.pointNum,sizeof(float));

        if(freeSpacePoints != nullptr)
        {
           
            float max_dis = std::pow(mapParams.length,2);

            for(int i = 0; i < mapParams.pointNum; i++)
            {
                freeSpacePoints[i] = max_dis;
            }
        }
        else{
            std::cerr<<"WARNING : memory allocation error!"<<std::endl;
            return;
        }

        computeFreeSpacePoints(pointCloudIn,freeSpacePoints,mapParams.pointNum);
        
        freeGridMapFilter(freeSpacePoints,mapParams,freeGridMap);//可同行区域栅格地图待修改
        
        free(freeSpacePoints);
    }
}   //genLocalFreeMap





#endif