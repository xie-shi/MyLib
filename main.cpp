#include <iostream>
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"
#include "pcl/filters/passthrough.h"

#include <pcl/filters/voxel_grid.h> //体素下采样

#include "pcl/filters/statistical_outlier_removal.h"
#include <pcl/kdtree/kdtree_flann.h>  //kd-tree搜索对象的类定义的头文件
#include <pcl/surface/mls.h>        //最小二乘法平滑处理类定义头文件

#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h> //边界提取

#include <pcl/visualization/pcl_visualizer.h>
#include "pcl/ModelCoefficients.h"
#include "pcl/filters/project_inliers.h"

//直通滤波
int Passthrough(pcl::PointCloud<pcl::PointXYZI>::Ptr& InputCloud,pcl::PointCloud<pcl::PointXYZI>::Ptr& OutCloud){
    pcl::PassThrough<pcl::PointXYZI> pass(true); // 创建滤波器对象
    pass.setInputCloud(InputCloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.25, 10.0);
    std::vector<int> indices_x;
    pass.filter(indices_x); // indices_x 保存的是在(-0.25,10.0)范围点的索引

    //智能指针指向索引
    pcl::IndicesPtr  Index_ptr_x = boost::make_shared<std::vector<int>>(indices_x);
    //pcl::IndicesPtr Index_ptr_x(std::vector<int>) = std::make_shared<std::vector<int>>(indices_x);

    pass.setIndices(Index_ptr_x);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-5.0, 5.0);
    std::vector<int> indices_y;
    pass.filter(indices_y);
    pcl::IndicesPtr  Index_ptr_y = boost::make_shared<std::vector<int>>(indices_y);

    pass.setIndices(Index_ptr_y);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1.0, DBL_MAX);
    pcl::PointCloud<pcl::PointXYZI>::Ptr VoxeCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pass.filter(*VoxeCloud);

    pcl::VoxelGrid<pcl::PointXYZI> Voxe_sor;
    Voxe_sor.setInputCloud(VoxeCloud); //设置需要过滤的点云给滤波对象
    Voxe_sor.setLeafSize (0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
    Voxe_sor.filter(*OutCloud);

    //Voxe_sor.filter (*cloud_filtered);           //执行滤波处理，存储输出
//    std::vector<int> indices_z;
//    pass.filter(indices_z);
//    pcl::IndicesPtr  Index_ptr_z = boost::make_shared<std::vector<int>>(indices_y);
    return 0;
}

bool  SelectPoint (    pcl::PointCloud<pcl::PointXYZI>::Ptr& InCloud
                                        ,  pcl::PointCloud<pcl::PointXYZI>::Ptr& OutCloud
                                        , int  StepStart,  int   StepEnd)
    {
         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        
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

                            

int main() {
    // ----------------------------加载点云-----------------------------
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>); 
    if (pcl::io::loadPCDFile<pcl::PointXYZI>("/home/xie/桌面/PCD_File/hz2070.pcd", *cloud) == -1)
    {
        PCL_ERROR("读取源标点云失败 \n");
        return (-1);
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
SelectPoint(cloud, pcl_pointcloud, 2 , 6);

// ----------------------------直通滤波-----------------------------
    pcl::PointCloud<pcl::PointXYZI>::Ptr  pcl_pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    Passthrough(pcl_pointcloud,pcl_pointcloud_filtered);

    /*
     * statistical_outlier_removal */
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    pcl::PointCloud<pcl::PointXYZI>::Ptr  pcl_pointcloud_sort(new pcl::PointCloud<pcl::PointXYZI>);
    sor.setInputCloud(pcl_pointcloud_filtered);
    sor.setMeanK(100);
    sor.setStddevMulThresh(1.0);
    sor.filter(*pcl_pointcloud_sort);

//    // 创建KD-Tree
//    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
//
//    // Output has the PointNormal type in order to store the normals calculated by MLS
//    pcl::PointCloud<pcl::PointXYZINormal> mls_points;
//
//    // 定义最小二乘实现的对象mls
//    pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZINormal> mls;
//
//    mls.setComputeNormals (true);  //设置在最小二乘计算中需要进行法线估计
//
//    // Set parameters
//    mls.setInputCloud (pcl_pointcloud_sort);
//    mls.setPolynomialFit (true);
//    mls.setSearchMethod (tree);
//    mls.setSearchRadius (0.03);
//
//
//    mls.process(mls_points);
//    //pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);
//    cout << "从点云中读取 " << mls_points.size() << " 个点" << endl;




    //------------------------计算法向量---------------------------
        pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> mls;
        pcl::search::KdTree<pcl::PointXYZI>::Ptr Tree(new pcl::search::KdTree<pcl::PointXYZI>);
        mls.setInputCloud(pcl_pointcloud_sort);
        mls.setSearchMethod(Tree);
        mls.setRadiusSearch(0.01);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals(new pcl::PointCloud<pcl::PointXYZINormal>);

        mls.compute(*normals);
        //-----------------------边界特征估计--------------------------
        pcl::BoundaryEstimation<pcl::PointXYZI, pcl::PointXYZINormal, pcl::Boundary> boundEst;
        boundEst.setInputCloud(pcl_pointcloud_sort);
        boundEst.setInputNormals(normals);
        boundEst.setRadiusSearch(0.02);
        boundEst.setAngleThreshold(M_PI / 2);//边界判断时的角度阈值

        boundEst.setSearchMethod(Tree);
        pcl::PointCloud<pcl::Boundary> boundaries;
        boundEst.compute(boundaries);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZI>);
        for (int i = 0; i < pcl_pointcloud_sort->points.size(); i++)
        {

            if (boundaries[i].boundary_point > 0)
            {
                cloud_boundary->push_back(pcl_pointcloud_sort->points[i]);
            }
        }
        cout << "边界点个数:" << cloud_boundary->points.size() << endl;
        //pcl::io::savePCDFileASCII("YY11.pcd", *cloud_boundary);

        //-------------------------可视化-----------------------------
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("ShowCloud"));
        int v1(0);
        viewer->setWindowName("边界提取");
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer->setBackgroundColor(0, 0, 0, v1);
        viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);
        int v2(0);
        viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
        viewer->setBackgroundColor(0.5, 0.5, 0.5, v2);
        viewer->addText("Boudary point clouds", 10, 10, "v2_text", v2);

        viewer->addPointCloud<pcl::PointXYZI>(pcl_pointcloud_sort, "sample cloud", v1);
        viewer->addPointCloud<pcl::PointXYZI>(cloud_boundary, "cloud_boundary", v2);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_boundary", v2);
        //view->addCoordinateSystem(1.0);
        //view->initCameraParameters();

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("cloud show"));
//        int v1 = 0;
//        viewer->createViewPort(0, 0, 0.5, 1, v1);
//        viewer->setBackgroundColor(0, 0, 0, v1);
//
//        //原始点云绿色
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> src_h(pcl_pointcloud_sort, 0, 255, 0);
//
//        //viewer->setBackgroundColor(255, 255, 255);
//        viewer->addPointCloud(pcl_pointcloud_sort, src_h, "cloud", v1);


//        for(auto iter:*pcl_pointcloud_sort)
//            std::cout<<"( "<<iter.x<<" "<< iter.y << " "<<iter.z<<" "<<iter.intensity<<")\n";
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(10000));
        }

        return 0;
    }



