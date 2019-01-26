
#ifndef CERES_REFINE
#define CERES_REFINE

#include <iostream>
#include <stdio.h>
#include <math.h>
using namespace std;


#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>



#include "ceres/ceres.h"
#include "glog/logging.h"


using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

struct ProjectiveResidual {
	ProjectiveResidual(float x, float y, int index_cam,int num_img)
		:x_(x),y_(y),index_cam_(index_cam),num_img_(num_img){}

	template<typename T> bool operator()(const T* const params,
		T* residual) const
	{
		cv::Mat A = cv::Mat::zeros(3,3,CV_32FC1);
		cv::Mat Rv = cv::Mat::zeros(3, 1, CV_32FC1);
		cv::Mat R = cv::Mat::zeros(3, 3, CV_32FC1);
		cv::Mat t = cv::Mat::zeros(3, 1, CV_32FC1);
		cv::Mat k = cv::Mat::zeros(2, 1, CV_32FC1);
		
		int num_Rv = 3;
		int num_R = 9;
		int num_t = 3;

		A.at<T>(0,0) = params[0];
		A.at<T>(1,1) = params[1];
		A.at<T>(0,1) = params[2];
		A.at<T>(0,2) = params[3];
		A.at<T>(1,2) = params[4];

		Rv.at<T>(0, 0) = params[5 + index_cam_ * 3];
		Rv.at<T>(1, 0) = params[6 + index_cam_ * 3];
		Rv.at<T>(2, 0) = params[7 + index_cam_ * 3];
		cv::Rodrigues(Rv, R);

		t.at<T>(0, 0) = params[5 + index_cam_ * 3 + num_Rv * num_img_];
		t.at<T>(1, 0) = params[6 + index_cam_ * 3 + num_Rv * num_img_];
		t.at<T>(2, 0) = params[7 + index_cam_ * 3 + num_Rv * num_img_];

		k.at<T>(0,0) = params

	}
		
private:
	const float x_;
	const float y_;
	const int index_cam_;
	const int num_img_;
};


int ceres_nonlinear_op(const vector<vector<cv::Point3f>> &obj_points, \
	const vector<vector<cv::Point2f>> &img_points, \
	cv::Mat A, vector<cv::Mat> R_set, vector<cv::Mat> t_set, cv::Mat k, vector<pair<int, int>> w_h,
	int MAX_ITER);


#endif


//int num_R = 3 * 3;
//int num_t = 3;
//for (int i = 0; i < obj_points.size(); i++)
//{
//	params.at<float>(5 + i * num_R, 0) = R_set[i].at<float>(0, 0);
//	params.at<float>(6 + i * num_R, 0) = R_set[i].at<float>(0, 1);
//	params.at<float>(7 + i * num_R, 0) = R_set[i].at<float>(0, 2);
//	params.at<float>(8 + i * num_R, 0) = R_set[i].at<float>(1, 0);
//	params.at<float>(9 + i * num_R, 0) = R_set[i].at<float>(1, 1);
//	params.at<float>(10 + i * num_R, 0) = R_set[i].at<float>(1, 2);
//	params.at<float>(11 + i * num_R, 0) = R_set[i].at<float>(2, 0);
//	params.at<float>(12 + i * num_R, 0) = R_set[i].at<float>(2, 1);
//	params.at<float>(13 + i * num_R, 0) = R_set[i].at<float>(2, 2);
//
//	params.at<float>(14 + i * num_t + num_R * (obj_points.size() - 1), 0) = t_set[i].at<float>(0, 0);
//	params.at<float>(15 + i * num_t + num_R * (obj_points.size() - 1), 0) = t_set[i].at<float>(1, 0);
//	params.at<float>(16 + i * num_t + num_R * (obj_points.size() - 1), 0) = t_set[i].at<float>(2, 0);
//
//}
//
//params.at<float>(17 + num_R * (obj_points.size() - 1) + num_t * (obj_points.size() - 1), 0) = k.at<float>(0, 0);
//params.at<float>(18 + num_R * (obj_points.size() - 1) + num_t * (obj_points.size() - 1), 0) = k.at<float>(1, 0);