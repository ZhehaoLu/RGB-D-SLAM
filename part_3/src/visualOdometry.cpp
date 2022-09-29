

#include <fstream>
#include <iostream>
#include <sstream>
using namespace std;

#include "../include/slamBase.h"

#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

FRAME readFrame(int index, ParameterReader &pd);
double normofTransform(cv::Mat rvec, cv::Mat tvec);

int main(int argc, char **argv) {
  ParameterReader pd;
  int startIndex = atoi(pd.getData("start_index").c_str());
  int endIndex = atoi(pd.getData("end_index").c_str());

  // initialize
  cout << "Initializing ..." << endl;
  int currIndex = startIndex;
  FRAME lastFrame = readFrame(currIndex, pd);
  CAMERA_INTRINSIC_PARAMETERS camera;
  camera.cx = atof(pd.getData("camera.cx").c_str());
  camera.cy = atof(pd.getData("camera.cy").c_str());
  camera.fx = atof(pd.getData("camera.fx").c_str());
  camera.fy = atof(pd.getData("camera.fy").c_str());
  camera.scale = atof(pd.getData("camera.scale").c_str());
  computeKeyPointAndDesp(lastFrame);
  PointCloud::Ptr cloud =
      image2PointCloud(lastFrame.rgb, lastFrame.depth, camera);

  pcl::visualization::CloudViewer viewer("viewer");

  bool visualize = pd.getData("visualize_pointcloud") == string("yes");

  int min_inliers = atoi(pd.getData("min_inliers").c_str());
  double max_norm = atof(pd.getData("max_norm").c_str());

  for (currIndex = startIndex + 1; currIndex < endIndex; currIndex++) {
    cout << "Reading files " << currIndex << endl;
    FRAME currFrame = readFrame(currIndex, pd);
    computeKeyPointAndDesp(currFrame);
    RESULT_OF_PNP result = estimateMotion(lastFrame, currFrame, camera);
    if (result.inliers < min_inliers)
      continue;
    double norm = normofTransform(result.rvec, result.tvec);
    cout << "norm = " << norm << endl;
    if (norm >= max_norm)
      continue;
    Eigen::Isometry3d T = cvMat2Eigen(result.rvec, result.tvec);
    cout << "T=" << T.matrix() << endl;

    cloud = joinPointCloud(cloud, currFrame, T, camera);

    if (visualize == true)
      viewer.showCloud(cloud);

    lastFrame = currFrame;
  }

  pcl::io::savePCDFile("result.pcd", *cloud);
  return 0;
}

FRAME readFrame(int index, ParameterReader &pd) {
  FRAME f;
  string rgbDir = pd.getData("rgb_dir");
  string depthDir = pd.getData("depth_dir");

  string rgbExt = pd.getData("rgb_extension");
  string depthExt = pd.getData("depth_extension");

  stringstream ss;
  ss << rgbDir << index << rgbExt;
  string filename;
  ss >> filename;
  f.rgb = cv::imread(filename);
  ss.clear();
  filename.clear();
  ss << depthDir << index << depthExt;
  ss >> filename;
  f.depth = cv::imread(filename, -1);
  return f;
}

double normofTransform(cv::Mat rvec, cv::Mat tvec) {
  return fabs(min(cv::norm(rvec), 2 * M_PI - cv::norm(rvec))) +
         fabs(cv::norm(tvec));
}
