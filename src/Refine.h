#ifndef REFINE
#define REFINE


#include <iostream>
#include <stdio.h>
#include <math.h>
using namespace std;



#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>




int projective2D(cv::Mat A, cv::Mat R, cv::Mat t, vector<cv::Point3f> single_obj_points, vector<cv::Point2f> &reprojective_points);



#endif