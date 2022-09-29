# pragma once

#include <fstream>
#include <vector>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
using namespace Eigen;

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

struct CAMERA_INTRINSIC_PARAMETERS 
{ 
    double cx, cy, fx, fy, scale;
};

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
    int frameID;
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
    if ( minDis < 10 ) 
        minDis = 10;

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < max(30.0,3*minDis))
            good_matches.push_back( matches[i] );
    }

    RESULT_OF_PNP result;

    if (good_matches.size() <= 5) 
    {
        result.inliers = -1;
        return result;
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

    if (pts_obj.size() <5 || pts_img.size()<5)
    {
        result.inliers = -1;
        return result;
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

    
    result.rvec=rvec;
    result.tvec=tvec;
    result.inliers=inliers.rows;

    return result;

};

static Eigen::Isometry3d cvMat2Eigen(cv::Mat& rvec,cv::Mat& tvec){
    cv::Mat R;
    cv::Rodrigues( rvec, R );
    Eigen::Matrix3d r;
    for ( int i=0; i<3; i++ )
        for ( int j=0; j<3; j++ ) 
            r(i,j) = R.at<double>(i,j);


    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    cout<<"translation"<<endl;
    Eigen::Translation<double,3> trans(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2));
    T = angle;
    T(0,3) = tvec.at<double>(0,0); 
    T(1,3) = tvec.at<double>(0,1); 
    T(2,3) = tvec.at<double>(0,2);

    cout<<T.matrix()<<endl;
    return T;
};

static PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS camera){
    
    PointCloud::Ptr newcloud=image2PointCloud(newFrame.rgb,newFrame.depth,camera);

    PointCloud::Ptr output (new PointCloud ());
    
    pcl::transformPointCloud(*original,*output,T.matrix());

    *newcloud+=*output;
    static pcl::VoxelGrid<PointT> voxel;
    static ParameterReader pd;
    double gridsize = atof( pd.getData("voxel_grid").c_str() );
    voxel.setLeafSize( gridsize, gridsize, gridsize );
    voxel.setInputCloud( newcloud );
    PointCloud::Ptr tmp( new PointCloud() );
    voxel.filter( *tmp );
    return tmp;
    
}

inline static CAMERA_INTRINSIC_PARAMETERS getDefaultCamera()
{
    ParameterReader pd;
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    camera.scale = atof( pd.getData( "camera.scale" ).c_str() );
    return camera;
}