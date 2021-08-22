#include "../include/slamBase.h"

// integrate generatePointCloud.cpp into a function
PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    PointCloud::Ptr cloud ( new PointCloud );

    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            ushort d = depth.ptr<ushort>(m)[n];
            if (d == 0)
                continue;
            PointT p;

            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;
            
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            cloud->points.push_back( p );
        }

    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}


cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera )
{
    cv::Point3f p; // 3D 点
    p.z = double( point.z ) / camera.scale;
    p.x = ( point.x - camera.cx) * p.z / camera.fx;
    p.y = ( point.y - camera.cy) * p.z / camera.fy;
    return p;
}
ParameterReader::ParameterReader(string filename){

        ifstream ifs(filename.c_str());
        if(!ifs){
            cout<<"parameter file does not exist."<<endl;
            return;
        }

        while(!ifs.eof()){
            string str;
            getline(ifs,str);
            if(str[0]=='#') continue;

            int pos=str.find('=');
            if(pos==-1) continue;

            string key=str.substr(0,pos);
            string value=str.substr(pos+1,str.size());

            data[key]=value;

            if(!ifs.good()) break;
        }
    }

string ParameterReader::getData(string key){
        map<string,string>::iterator itr=data.find(key);
        if(itr==data.end()){
            cout<<"parameter not found."<<endl;
            return string("not found.");
        }  
        return itr->second;
    }