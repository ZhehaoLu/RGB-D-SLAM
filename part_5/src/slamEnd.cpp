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
    
    ParameterReader pd;
    int startIndex  =   atoi( pd.getData( "start_index" ).c_str() );
    int endIndex    =   atoi( pd.getData( "end_index"   ).c_str() );

    // initialize
    cout<<"Initializing ..."<<endl;
    int currIndex = startIndex; 
    FRAME lastFrame = readFrame( currIndex, pd );

    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.cx = atof(pd.getData("camera.cx").c_str());
    camera.cy = atof(pd.getData("camera.cy").c_str());
    camera.fx = atof(pd.getData("camera.fx").c_str());
    camera.fy = atof(pd.getData("camera.fy").c_str());
    camera.scale = atof(pd.getData("camera.scale").c_str());
    computeKeyPointAndDesp(lastFrame);
    PointCloud::Ptr cloud = image2PointCloud( lastFrame.rgb, lastFrame.depth, camera );
    
    pcl::visualization::CloudViewer viewer("viewer");

    bool visualize = pd.getData("visualize_pointcloud")==string("yes");

    int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    double max_norm = atof( pd.getData("max_norm").c_str() );
    
    //initialize the optimization solver
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    unique_ptr<SlamBlockSolver::LinearSolverType> linearSolver (new SlamLinearSolver());
    unique_ptr<SlamBlockSolver> blockSolver (new SlamBlockSolver(move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(move(blockSolver));

    g2o::SparseOptimizer globalOptimizer;  
    globalOptimizer.setAlgorithm( solver ); 
    globalOptimizer.setVerbose( false );

    // add the 1st vertex
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId( currIndex );
    v->setEstimate( Eigen::Isometry3d::Identity() ); 
    v->setFixed( true ); 
    globalOptimizer.addVertex( v );

    int lastIndex=currIndex;

    for(currIndex=startIndex+1;currIndex<endIndex;currIndex++){

        // calculate the transformation matrix
        
        cout<<"Reading files "<<currIndex<<endl;
        FRAME currFrame = readFrame( currIndex,pd ); 
        computeKeyPointAndDesp( currFrame);

        RESULT_OF_PNP result = estimateMotion( lastFrame, currFrame, camera );
        if ( result.inliers < min_inliers ) 
            continue;
        double norm = normofTransform(result.rvec, result.tvec);
        cout<<"norm = "<<norm<<endl;
        if ( norm >= max_norm )
            continue;
        Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );
        cout<<"T="<<T.matrix()<<endl;
        
        //cloud = joinPointCloud( cloud, currFrame, T, camera );
        
        if ( visualize == true )
            viewer.showCloud( cloud );

        // optimize the transformation matrix
        // 1. add a new vertex
        // 2. link the last vertex and the current one

        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( currIndex );
        v->setEstimate( Eigen::Isometry3d::Identity() );
        globalOptimizer.addVertex(v);

        g2o::EdgeSE3* edge = new g2o::EdgeSE3();

        edge->vertices() [0] = globalOptimizer.vertex( lastIndex );
        edge->vertices() [1] = globalOptimizer.vertex( currIndex );

        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
        
        // the information matrix is the inverse of the covariance matrix which represents the pre-estimatation
        information(0,0) = information(1,1) = information(2,2) = 100;
        information(3,3) = information(4,4) = information(5,5) = 100;

        edge->setInformation( information );
        // the initial value of optmization is the calculation results of pnp algorithm
        edge->setMeasurement( T );
    
        globalOptimizer.addEdge(edge);

        lastFrame = currFrame;
        lastIndex = currIndex;
    }

    // pcl::io::savePCDFile( "result.pcd", *cloud);
    // cout<<"file saved."<<endl;
    cout<<"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.save("./data/result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); 
    globalOptimizer.save( "./data/result_after.g2o" );
    cout<<"Optimization done."<<endl;

    globalOptimizer.clear();
    return 0;

}