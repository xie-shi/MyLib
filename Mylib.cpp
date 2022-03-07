#include <iostream>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"

//选择任意连续线序的激光点
bool  SelectPoint (    pcl::PointCloud<pcl::PointXYZI>::Ptr& InCloud
                                        ,  pcl::PointCloud<pcl::PointXYZI>::Ptr& OutCloud
                                        , int  StepStart,  int   StepEnd)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        //将同一线的点连续排列。宽度为width
        for(auto step = 0; step<16; step++){
                for(auto iter = InCloud->points.begin()+step; iter<InCloud->points.end();){
                    cloud->points.push_back(*iter);
                    iter +=16 ;
                }
        }
        cloud->height=16;
        cloud->width = cloud->size()/cloud->height;
        

        OutCloud->height = StepEnd-StepStart+1;
        OutCloud->width = cloud->width;
        auto IterStart = cloud->begin()+cloud->width*StepStart;
        auto IterEnd = cloud->begin()+ cloud->width*(StepEnd+1);
        OutCloud->points.assign(IterStart , IterEnd);

        return true;
    }
