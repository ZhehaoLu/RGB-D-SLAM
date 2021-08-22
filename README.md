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

Then you can see all of the on-going computational results when the images are being processed. After everything is completed, a result.pcd file will be automatically generated in data. 

$ pcl_viewer result.pcd
The fused pointcloud can be shown by a PCD viewer.

