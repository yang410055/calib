#ifndef COMPUTEV
#define COMPUTEV


#include <iostream>
#include <stdio.h>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>



//计算张正友论文中的V矩阵
int computeV(vector<cv::Mat> &H_set, cv::Mat &V);




#endif