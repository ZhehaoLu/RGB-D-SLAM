# pragma once


// 各种头文件 
// C++标准库
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
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/impl/search.hpp>


// 类型定义
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参结构
struct CAMERA_INTRINSIC_PARAMETERS 
{ 
    double cx, cy, fx, fy, scale;
};

// 函数接口
// image2PonitCloud 将rgb图转换为点云
PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera );

// point2dTo3d 将单个点从图像坐标转换为空间坐标
// input: 3维点Point3f (u,v,d)
cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera );

class ParameterReader{
    public:
    ParameterReader( string filename="./parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }

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

static void computeKeyPointAndDesp(FRAME& frame){

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

    cout<<"matches = "<<matches.size()<<endl;

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }
    cout<<"min dis = "<<minDis<<endl;

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < max(30.0,10*minDis))
            good_matches.push_back( matches[i] );
    }

    cout<<"good matches 1 = "<<good_matches.size()<<endl;


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

    cout<<"good matches 2 = "<<pts_obj.size()<<endl;

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

inline Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec )
{
    cv::Mat R;
    cv::Rodrigues( rvec, R );
    Eigen::Matrix3d r;
    for ( int i=0; i<3; i++ )
        for ( int j=0; j<3; j++ ) 
            r(i,j) = R.at<double>(i,j);
  
    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    T = angle;
    T(0,3) = tvec.at<double>(0,0); 
    T(1,3) = tvec.at<double>(1,0); 
    T(2,3) = tvec.at<double>(2,0);
    return T;
}

// joinPointCloud 
// 输入：原始点云，新来的帧以及它的位姿
// 输出：将新来帧加到原始帧后的图像
static PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera ) 
{
    PointCloud::Ptr newCloud = image2PointCloud( newFrame.rgb, newFrame.depth, camera );

    // 合并点云
    PointCloud::Ptr output (new PointCloud);
    pcl::transformPointCloud( *original, *output, T.matrix() );
    *newCloud += *output;

    //return newCloud;
    // Voxel grid 滤波降采样
    static pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud( newCloud );
    voxel.setLeafSize( 0.01f,0.01f,0.01f );
    PointCloud::Ptr tmp( new PointCloud);
    voxel.filter( *tmp );
    return tmp;
}