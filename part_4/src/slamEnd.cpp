#include <iostream>
#include <fstream>
#include <sstream>

#include "../include/slamBase.h"
#include <pcl/visualization/cloud_viewer.h>
using namespace std;

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

FRAME readFrame(int index, ParameterReader & pd){

    // string rgbDir=pd.getData("rgb_dir");
    // string depthDir=pd.getData("depth_dir");

    // string rgbExt=pd.getData("rgb_extension");
    // string depthExt=pd.getData("depth_extension");

    // stringstream ss;
    // ss<<rgbDir<<index<<rgbExt;

    // string filename1;
    // ss>>filename1;
    // FRAME f;
    
    // f.rgb=cv::imread(filename1);

    // ss.clear();
    // filename1.clear();

    // string filename2;
    // ss<<depthDir<<index<<depthExt;
    // ss>>filename2;

    // f.depth=cv::imread(filename2,-1);

    // ss.clear();
    // filename2.clear();

    // return f;

    FRAME f;
    string rgbDir   =   pd.getData("rgb_dir");
    string depthDir =   pd.getData("depth_dir");
    
    string rgbExt   =   pd.getData("rgb_extension");
    string depthExt =   pd.getData("depth_extension");

    stringstream ss;
    ss<<rgbDir<<index<<rgbExt;
    string filename;
    ss>>filename;
    f.rgb = cv::imread( filename );
    ss.clear();
    filename.clear();
    ss<<depthDir<<index<<depthExt;
    ss>>filename;
    f.depth = cv::imread( filename, -1 );
    return f;

}

double normofTransform (cv::Mat rvec, cv::Mat tvec){
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}

int main(int argc, char** argv){
    // ParameterReader pd;

    // CAMERA_INTRINSIC_PARAMETERS C;
    // C.cx = atof(pd.getData("camera.cx").c_str());
    // C.cy = atof(pd.getData("camera.cy").c_str());
    // C.fx = atof(pd.getData("camera.fx").c_str());
    // C.fy = atof(pd.getData("camera.fy").c_str());
    // C.scale = atof(pd.getData("camera.scale").c_str());

    // int startIndex=atoi(pd.getData("start_index").c_str());
    // int endIndex=atoi(pd.getData("end_index").c_str());
    // int currIndex=startIndex;

    // FRAME lastFrame=readFrame(currIndex,pd);
    // computeKeyPointAndDesp(lastFrame);
    // PointCloud::Ptr cloud=image2PointCloud(lastFrame.rgb,lastFrame.depth,C);

    // pcl::visualization::CloudViewer viewer("viewer");
    // cout<<"initializing viewer.."<<endl;

    // bool visualize=pd.getData("visualize_pointcloud")==string("yes");
    // int min_inliers=atoi(pd.getData("min_inliers").c_str());
    // double max_norm=atoi(pd.getData("max_norm").c_str());
    
    ParameterReader pd;
    int startIndex  =   atoi( pd.getData( "start_index" ).c_str() );
    int endIndex    =   atoi( pd.getData( "end_index"   ).c_str() );

    // initialize
    cout<<"Initializing ..."<<endl;
    int currIndex = startIndex; // 当前索引为currIndex
    FRAME lastFrame = readFrame( currIndex, pd ); // 上一帧数据
    // 我们总是在比较currFrame和lastFrame
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.cx = atof(pd.getData("camera.cx").c_str());
    camera.cy = atof(pd.getData("camera.cy").c_str());
    camera.fx = atof(pd.getData("camera.fx").c_str());
    camera.fy = atof(pd.getData("camera.fy").c_str());
    camera.scale = atof(pd.getData("camera.scale").c_str());
    computeKeyPointAndDesp(lastFrame);
    PointCloud::Ptr cloud = image2PointCloud( lastFrame.rgb, lastFrame.depth, camera );
    
    pcl::visualization::CloudViewer viewer("viewer");

    // 是否显示点云
    bool visualize = pd.getData("visualize_pointcloud")==string("yes");

    int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    double max_norm = atof( pd.getData("max_norm").c_str() );
    
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    unique_ptr<SlamBlockSolver::LinearSolverType> linearSolver (new SlamLinearSolver());
    unique_ptr<SlamBlockSolver> blockSolver (new SlamBlockSolver(move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(move(blockSolver));

    g2o::SparseOptimizer globalOptimizer;  // 最后用的就是这个东东
    globalOptimizer.setAlgorithm( solver ); 
    // 不要输出调试信息
    globalOptimizer.setVerbose( false );

    // 向globalOptimizer增加第一个顶点
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId( currIndex );
    v->setEstimate( Eigen::Isometry3d::Identity() ); //估计为单位矩阵
    v->setFixed( true ); //第一个顶点固定，不用优化
    globalOptimizer.addVertex( v );

    int lastIndex=currIndex;

    for(currIndex=startIndex+1;currIndex<endIndex;currIndex++){
        
        // FRAME currFrame=readFrame(currIndex,pd);
        // computeKeyPointAndDesp(currFrame);
        // RESULT_OF_PNP result=estimateMotion(lastFrame,currFrame,C);
        // if(result.inliers<min_inliers) continue;
        // if(normofTransform(result.rvec,result.tvec)>max_norm) continue;
        // cloud=joinPointCloud(cloud,currFrame,cvMat2Eigen(result.rvec,result.tvec),C);
        // if(visualize==true) viewer.showCloud(cloud);
        // lastFrame=currFrame;
        cout<<"Reading files "<<currIndex<<endl;
        FRAME currFrame = readFrame( currIndex,pd ); // 读取currFrame
        computeKeyPointAndDesp( currFrame);
        // 比较currFrame 和 lastFrame
        RESULT_OF_PNP result = estimateMotion( lastFrame, currFrame, camera );
        if ( result.inliers < min_inliers ) //inliers不够，放弃该帧
            continue;
        // 计算运动范围是否太大
        double norm = normofTransform(result.rvec, result.tvec);
        cout<<"norm = "<<norm<<endl;
        if ( norm >= max_norm )
            continue;
        Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );
        cout<<"T="<<T.matrix()<<endl;
        
        //cloud = joinPointCloud( cloud, currFrame, T, camera );
        
        if ( visualize == true )
            viewer.showCloud( cloud );

        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( currIndex );
        v->setEstimate( Eigen::Isometry3d::Identity() );
        globalOptimizer.addVertex(v);
        // 边部分
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        // 连接此边的两个顶点id
        edge->vertices() [0] = globalOptimizer.vertex( lastIndex );
        edge->vertices() [1] = globalOptimizer.vertex( currIndex );
        // 信息矩阵
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
        // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
        // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
        // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
        information(0,0) = information(1,1) = information(2,2) = 100;
        information(3,3) = information(4,4) = information(5,5) = 100;
        // 也可以将角度设大一些，表示对角度的估计更加准确
        edge->setInformation( information );
        // 边的估计即是pnp求解之结果
        edge->setMeasurement( T );
        // 将此边加入图中
        globalOptimizer.addEdge(edge);

        lastFrame = currFrame;
        lastIndex = currIndex;
    }

    // pcl::io::savePCDFile( "result.pcd", *cloud);
    // cout<<"file saved."<<endl;
    cout<<"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.save("./data/result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); //可以指定优化步数
    globalOptimizer.save( "./data/result_after.g2o" );
    cout<<"Optimization done."<<endl;

    globalOptimizer.clear();
    return 0;

}