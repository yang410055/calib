
#ifndef COMPUTEHOMOGRAPH
#define COMPUTEHOMOGRAPH


#include <iostream>
#include <stdio.h>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>



//计算图像序列的单应性矩阵
int computeH(vector< vector< cv::Point3f > > &obj_points, vector< vector<cv::Point2f> >, vector<cv::Mat H> H_set );
