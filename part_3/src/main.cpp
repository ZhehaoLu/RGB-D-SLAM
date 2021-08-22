#include <iostream>
#include "../include/slamBase.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
 
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>

void test01(){

    ParameterReader pd;

    cout<<pd.getData("camera.cx")<<endl;
    cout<<pd.getData("detector")<<endl;
}

void test02(){
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    pcl::io::loadPCDFile("result.pcd",*cloud);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;//滤波器处理对象
     sor.setInputCloud(cloud);//设置输入点云
     sor.setLeafSize(0.01f, 0.01f, 0.01f);//设置滤波器处理时采用的体素大小的参数，体素大小是长宽高均为0.01
     sor.filter(*cloud_filtered);//执行下采样，下采样之后的点云数据保存到 cloud_filtered 中
     std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
          << " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";
     //保存转换的输入点云
     pcl::PCDWriter writer;
    writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
    Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

}

int main(int argc, char** argv)
{
    std::cout<<"Hello SLAM!"<<std::endl;
    
    //test01();
    //test02();
    return 0;
}


