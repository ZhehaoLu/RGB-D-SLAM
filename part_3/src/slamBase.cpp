#include "../include/slamBase.h"

PointCloud::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth,
                                 CAMERA_INTRINSIC_PARAMETERS &camera) {
  PointCloud::Ptr cloud(new PointCloud);

  for (int m = 0; m < depth.rows; m++)
    for (int n = 0; n < depth.cols; n++) {
      ushort d = depth.ptr<ushort>(m)[n];
      if (d == 0)
        continue;
      PointT p;
      p.z = double(d) / camera.scale;
      p.x = (n - camera.cx) * p.z / camera.fx;
      p.y = (m - camera.cy) * p.z / camera.fy;

      p.b = rgb.ptr<uchar>(m)[n * 3];
      p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
      p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

      cloud->points.push_back(p);
    }
  cloud->height = 1;
  cloud->width = cloud->points.size();
  cloud->is_dense = false;

  return cloud;
}

cv::Point3f point2dTo3d(cv::Point3f &point,
                        CAMERA_INTRINSIC_PARAMETERS &camera) {
  cv::Point3f p;
  p.z = double(point.z) / camera.scale;
  p.x = (point.x - camera.cx) * p.z / camera.fx;
  p.y = (point.y - camera.cy) * p.z / camera.fy;
  return p;
}
