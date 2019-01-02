#ifndef COMPUTE2CAMERART
#define COMPUTE2CAMERART


#include <iostream>
#include <stdio.h>
#include <math.h>
using namespace std;



#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>



int compute_2camera_R_t( vector<cv::Mat> R_set1,
	vector<cv::Mat> t_set1,
	vector<cv::Mat> R_set2,
	vector<cv::Mat> t_set2,
	cv::Mat &R,
	cv::Mat &t );





#endif