#include <iostream>
using namespace std;

#include "../include/slamBase.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace cv;

int main(int argc, char **argv) {

  // read rgb and depth .png file
  FRAME frame1, frame2;
  frame1.rgb = imread("./data/rgb1.png");
  frame1.depth = imread("./data/depth1.png", -1);
  frame2.rgb = imread("./data/rgb2.png");
  frame2.depth = imread("./data/depth2.png", -1);

  // feature extraction and calculation
  computeKeyPointAndDesp(frame1);
  computeKeyPointAndDesp(frame2);

  // camera intrinsic parameters
  ParameterReader pd;
  CAMERA_INTRINSIC_PARAMETERS C;
  C.cx = atof(pd.getData("camera.cx").c_str());
  C.cy = atof(pd.getData("camera.cy").c_str());
  C.fx = atof(pd.getData("camera.fx").c_str());
  C.fy = atof(pd.getData("camera.fy").c_str());
  C.scale = atof(pd.getData("camera.scale").c_str());

  // estimate motion by pnp algorithm
  RESULT_OF_PNP result = estimateMotion(frame1, frame2, C);

  // transformation to rotational matrix
  cv::Mat R;
  cv::Rodrigues(result.rvec, R);
  Eigen::Matrix3d r;
  cv::cv2eigen(R, r);

  cout << R << endl;

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  Eigen::AngleAxisd angle(r);
  cout << "translation" << endl;
  Eigen::Translation<double, 3> trans(result.tvec.at<double>(0, 0),
                                      result.tvec.at<double>(0, 1),
                                      result.tvec.at<double>(0, 2));
  T = angle;
  T(0, 3) = result.tvec.at<double>(0, 0);
  T(1, 3) = result.tvec.at<double>(0, 1);
  T(2, 3) = result.tvec.at<double>(0, 2);

  cout << T.matrix() << endl;

  // transformation into pointclouds
  PointCloud::Ptr cloud1 = image2PointCloud(frame1.rgb, frame1.depth, C);
  PointCloud::Ptr cloud2 = image2PointCloud(frame2.rgb, frame2.depth, C);

  // pointcloud fusion
  PointCloud::Ptr output(new PointCloud());

  pcl::transformPointCloud(*cloud1, *output, T.matrix());

  *output += *cloud2;
  pcl::io::savePCDFile("./result.pcd", *output);
  pcl::visualization::CloudViewer viewer("viewer");
  viewer.showCloud(output);
  while (!viewer.wasStopped()) {
  }

  return 0;
}