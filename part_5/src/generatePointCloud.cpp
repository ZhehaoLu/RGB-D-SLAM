#include <iostream>
#include <string>
using namespace std;


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

const double camera_factor=1000;
const double camera_cx=325.5;
const double camera_cy=253.5;
const double camera_fx=518.0;
const double camera_fy=519.0;

int main(int argc, char ** argv){

    Mat depth,rgb;
    rgb=imread("./data/rgb.png");
    depth=imread("./data/depth.png",-1);

    PointCloud::Ptr cloud (new PointCloud);

    for(int m=0;m<depth.rows;m++){
        for(int n=0;n<depth.cols;n++){

            ushort d=depth.ptr<ushort>(m)[n];

            if(d==0) continue;

            PointT p;
            p.z=double(d)/camera_factor;
            p.x=(n-camera_cx)*p.z/camera_fx;
            p.y=(m-camera_cy)*p.z/camera_fy;

            p.b=rgb.ptr<uchar>(m)[n*3];
            p.g=rgb.ptr<uchar>(m)[n*3+1];
            p.r=rgb.ptr<uchar>(m)[n*3+2];

            cloud->points.push_back(p);
        }
    }

    cloud->height=1;
    cloud->width=cloud->points.size();
    cout<<"point cloud size = "<<cloud->points.size()<<endl;

    cloud->is_dense-false;
    pcl::io::savePCDFile("./pointcloud.pcd",*cloud);
    cloud->points.clear();
    cout<<"Point cloud saved."<<endl;
    return 0;
}

