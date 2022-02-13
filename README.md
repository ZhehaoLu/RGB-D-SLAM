# RGB-D-SLAM

The basic elements of this project can be summarized as:
1. Feature Extraction
2. Visual Odometry Construction
3. Graph Optimization
4. Loop Detecture

After browsing the codes from part_1 to part_5, you can have some knowledge of how to build a complete SLAM project from scratch, from the very simple function of feature detection of two consecutive frames to a relatively sophisticated solution of RGB-D SLAM.  

Prerequisites
---
OpenCV2, Eigen, PCL, g2o

Demo
---
Step1: Make sure you've installed correctly all of the library packages and dependencies.

Step2: Build the project by VisualStudio Codes (or any other similar IDEs).

Step3: $ cd part_5

Step4: $ bin/slam

Then you can see all of the on-going computational results when the images are being processed. After everything is completed, a result.pcd file will be automatically generated in the file folder of data. 

Step5: $ pcl_viewer result.pcd

The fused pointcloud can be shown by a PCD viewer.

![rgb_slam](https://user-images.githubusercontent.com/85860671/153776131-0f5d19c4-6019-4f57-93d2-53288b7f438a.png)

Description of each part
----
1. Organization form of the entire project

In each part, the SLAM project can be devided into 4 basic folders, which are: bin,include,lib and src. Bin folder is exactly where we store all of the executable files. Include folder includes the basic functions that we may use during the following process. Lib folder contains the library files that we make. All of the .cpp files are placed in the Src file.

2. Part1

generatePointCloud.cpp: To make it simple, I make use of just one pair of rgb and depth image to generate a pointcloud. The traversal of the depth image aims at calculating the spatial coordinates of each points in the pointcloud, the color of these points are obtained by rgb image. Other parameters of the pointcloud is necessary to set in advance to prevent bugs from occurring, such as height, width and dense.  

slamBase.cpp: After the generatePointCloud.cpp is completed and verified by testing, the generation of pointcloud is packaged into a function called image2PointCloud() in slamBase.cpp.

detectFeatures.cpp: To obtain the transformation matrix of adjacent images, I first get the good matches of ORB features, then use the function of solvePnPRansac() in OpenCV to calculate the R and t of transformation matrix.

3. Part2

slamBase.cpp: After the detectFeatures.cpp is completed and verified by testing, it is divided into several separate functions and place into slamBase.cpp. ParameterReader() is a function that used to read the parameter.txt file which includes most of the variables that needed. With this function, the source codes are unnecessary to be changed and made once again even if parameters are unsuitable.

joinPointCloud.cpp: In this cpp file, the function that joins the pointclouds which represent two adjacent images is completed.

4. Part3

slamBase.cpp: To make future work smoother and easier, cvMat2Eigen() and joinPointCloud() functions are implemented. The former transforms r and t into transformation matrix of Eigen format, the latter integrates the function of joining two adjacent pointclouds.

visualOdometry.cpp: the basic functions of visual odometry are implemented here. What needs to be emphasize is that, I apply three methods to examine detection failure: (i) erase the frame that contains too little good matches; (ii) erase the frame that contains too little inliers; (iii) erase the situation that the transformation matrix is unreasonably too large.

5. Part4

slamEnd.cpp: On the basis of visualOdometry.cpp, I apply graph optimization (g2o) to optimize the transformation matrix that have already been calculated before generating the pointcloud. To run a g2o optimization successfully, the solver should be initialized at first, then the first vertex should be added to the solver which should be set fixed. For the image read loop, the current frame should be added to the solver, as well as a new edge which bridges the current and last frame should be generated, with the initial measurement T (transformation matrix that calculated by visual odometry).

6. Part5

Loop Detection is implemented.



