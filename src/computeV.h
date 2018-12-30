#ifndef COMPUTEV
#define COMPUTEV


#include <iostream>
#include <stdio.h>
#include <math.h>
using namespace std;



#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>



//计算张正友论文中的V矩阵
int computeV(vector<cv::Mat> &H_set, cv::Mat &V);

//计算含内参矩阵B
int computeB(cv::Mat V, cv::Mat &B);

//计算内参矩阵A
int computeA(cv::Mat B, cv::Mat &A);

//计算外参
int computeR_t( float namuda, cv::Mat A, vector<cv::Mat>H_set, vector<cv::Mat>&R_set, vector<cv::Mat>&t_set  );


#endif