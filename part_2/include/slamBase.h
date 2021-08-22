# pragma once

#include <fstream>
#include <vector>
using namespace std;

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// camera intrinsic parameters
struct CAMERA_INTRINSIC_PARAMETERS 
{ 
    double cx, cy, fx, fy, scale;
};

// image2PonitCloud: transform image into pointcloud
PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera );

cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera );

class ParameterReader{
    public:
    ParameterReader(string filename="./parameters.txt");
    string getData(string key);

    public:
    map<string,string> data;
};

struct FRAME{
    cv::Mat rgb,depth;
    cv::Mat desp;
    vector<cv::KeyPoint> kp; 
};

struct RESULT_OF_PNP{
    cv::Mat rvec,tvec;
    int inliers;
};

inline void computeKeyPointAndDesp(FRAME& frame){

    Ptr<FeatureDetector> detector=cv::ORB::create();
    Ptr<DescriptorExtractor> descriptor=ORB::create();

    detector->detect(frame.rgb,frame.kp);
    descriptor->compute(frame.rgb,frame.kp,frame.desp);
    
    return;
};

inline RESULT_OF_PNP estimateMotion(FRAME& frame1,FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS & camera){
    
    Ptr<DescriptorMatcher> matcher=DescriptorMatcher::create("BruteForce-Hamming");
    vector<DMatch> matches;
    matcher->match(frame1.desp,frame2.desp,matches);

    vector< cv::DMatch > good_matches;

    double minDis = 9999;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }
    //cout<<"min dis = "<<minDis<<endl;

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < max(30.0,3*minDis))
            good_matches.push_back( matches[i] );
    }

    
 

    vector<Point3f> pts_obj;
    vector<Point2f> pts_img;
    for(size_t i=0;i<good_matches.size();i++){
        Point2f p=frame1.kp[good_matches[i].queryIdx].pt;
        ushort d=frame1.depth.ptr<ushort>(int(p.y))[int(p.x)];

        if (d==0) continue;

        pts_img.push_back(Point2f(frame2.kp[good_matches[i].trainIdx].pt));

        Point3f pt(p.x,p.y,d);
        Point3f pd=point2dTo3d(pt,camera);

        pts_obj.push_back(pd);
        
    }

    cout<<"good matches = "<<pts_obj.size()<<endl;

    double camera_matrix_data[3][3]={
        {camera.fx,0,camera.cx},
        {0,camera.fy,camera.cy},
        {0,0,1}
    };
    
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );

    //cout<<cameraMatrix<<endl;

    cv::Mat rvec, tvec,inliers;

    cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99,inliers);

    RESULT_OF_PNP result;
    result.rvec=rvec;
    result.tvec=tvec;
    result.inliers=inliers.rows;

    return result;

}