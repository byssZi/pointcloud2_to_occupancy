//
// Created by laoli on 2021/12/27.
//

#ifndef LIDAR_PERCEPTION_COMMON_H
#define LIDAR_PERCEPTION_COMMON_H

#include "types.h"
#include <chrono>

namespace common{

    //用于计算程序耗时的程序
    class Tictoc
    {
    public:
        Tictoc()
        {
            tic();
        }

        void tic()
        {
            start = std::chrono::system_clock::now();
        }
        double toc()
        {
            end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            return elapsed_seconds.count() * 1000;      //单位：ms
        }


    private:
        std::chrono::time_point<std::chrono::system_clock> start,end;
    };



}



#endif //LIDAR_PERCEPTION_COMMON_H
