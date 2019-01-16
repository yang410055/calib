#ifndef REFINE
#define REFINE


#include <iostream>
#include <stdio.h>
#include <math.h>
using namespace std;



#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>


int projective2D(cv::Mat A, cv::Mat R, cv::Mat t, vector<cv::Point3f> single_obj_points, vector<cv::Point2f> &reprojective_points);


int normalizePoints(vector<cv::Point2f> single_img_points, vector< cv::Point2f >&single_img_points_normalize, int w, int h);

int computeInitDistortion(vector< vector< cv::Point2f > > img_points, vector< vector< cv::Point3f > >obj_points,
	cv::Mat A, vector<cv::Mat> R_set, vector<cv::Mat> t_set, vector<pair<int, int>>w_h , cv::Mat &k );



float Funcx(const cv::Point3f &single_obj_points_every , cv::Mat A, const cv::Mat &R, const cv::Mat &t, const cv::Mat &k, int w, int h);
float Funcy(const cv::Point3f &single_obj_points_every, cv::Mat A, const cv::Mat &R, const cv::Mat &t, const cv::Mat &k, int w, int h);

float Deriv(float(*Func)(const cv::Point3f &single_obj_points_every, cv::Mat A, \
	const cv::Mat &R, const cv::Mat &t, const cv::Mat &k, int w, int h), \
	const cv::Point3f &single_obj_points_every, \
	cv::Mat A, cv::Mat R, cv::Mat t, cv::Mat k,int w, int h, int n);


#endif