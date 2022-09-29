#include "../include/slamBase.h"
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

void test01() {

  ParameterReader pd;

  cout << pd.getData("camera.cx") << endl;
  cout << pd.getData("detector") << endl;
}

void test02() {
  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
  pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
  pcl::io::loadPCDFile("result.pcd", *cloud);
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*cloud_filtered);
  std::cerr << "PointCloud after filtering: "
            << cloud_filtered->width * cloud_filtered->height
            << " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";
  pcl::PCDWriter writer;
  writer.write("table_scene_lms400_downsampled.pcd", *cloud_filtered,
               Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);
}

int main(int argc, char **argv) {
  std::cout << "Hello SLAM!" << std::endl;

  // test01();
  // test02();
  return 0;
}
