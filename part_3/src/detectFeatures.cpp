#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>

#include "../include/slamBase.h"
#include <iostream>
using namespace std;
using namespace cv;

int main(int argc, char **argv) {

  Mat img_1, img_2, depth1, depth2;

  img_1 = imread("./data/rgb1.png");
  img_2 = imread("./data/rgb2.png");
  depth1 = imread("./data/depth1.png", -1);
  depth2 = imread("./data/depth2.png", -1);

  Ptr<FeatureDetector> detector = cv::ORB::create();
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  Ptr<DescriptorMatcher> matcher =
      DescriptorMatcher::create("BruteForce-Hamming");

  vector<KeyPoint> keypoint_1, keypoint_2;

  Mat descriptor_1, descriptor_2;

  detector->detect(img_1, keypoint_1);
  detector->detect(img_2, keypoint_2);

  cout << "Key points of two images: " << keypoint_1.size() << ", "
       << keypoint_2.size() << endl;

  descriptor->compute(img_1, keypoint_1, descriptor_1);
  descriptor->compute(img_2, keypoint_2, descriptor_2);

  vector<DMatch> matches;

  matcher->match(descriptor_1, descriptor_2, matches);

  Mat img_out1;

  drawKeypoints(img_1, keypoint_1, img_out1, Scalar::all(-1),
                DrawMatchesFlags::DEFAULT);

  Mat img_match;

  drawMatches(img_1, keypoint_1, img_2, keypoint_2, matches, img_match);

  imshow("match", img_match);

  vector<cv::DMatch> good_matches;

  double minDis = 9999;
  for (size_t i = 0; i < matches.size(); i++) {
    if (matches[i].distance < minDis)
      minDis = matches[i].distance;
  }

  for (size_t i = 0; i < matches.size(); i++) {
    if (matches[i].distance < 10 * minDis)
      good_matches.push_back(matches[i]);
  }

  Mat img_goodmatch;

  drawMatches(img_1, keypoint_1, img_2, keypoint_2, good_matches,
              img_goodmatch);

  imshow("goodmatch", img_goodmatch);

  CAMERA_INTRINSIC_PARAMETERS C;
  C.cx = 325.5;
  C.cy = 253.5;
  C.fx = 518.0;
  C.fy = 519.0;
  C.scale = 1000.0;

  vector<Point3f> pts_obj;
  vector<Point2f> pts_img;
  for (size_t i = 0; i < good_matches.size(); i++) {
    Point2f p = keypoint_1[good_matches[i].queryIdx].pt;
    ushort d = depth1.ptr<ushort>(int(p.y))[int(p.x)];

    if (d == 0)
      continue;

    pts_img.push_back(Point2f(keypoint_2[good_matches[i].trainIdx].pt));

    Point3f pt(p.x, p.y, d);
    Point3f pd = point2dTo3d(pt, C);

    pts_obj.push_back(pd);
  }

  double camera_matrix_data[3][3] = {
      {C.fx, 0, C.cx}, {0, C.fy, C.cy}, {0, 0, 1}};

  cv::Mat cameraMatrix(3, 3, CV_64F, camera_matrix_data);

  cv::Mat rvec, tvec;

  cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec,
                     false, 100, 1.0, 0.99);

  waitKey(0);

  return 0;
}