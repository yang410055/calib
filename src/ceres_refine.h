
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


#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

//struct ProjectiveResidual {
//	ProjectiveResidual(cv::Point3f single_obj_points_every,cv::Point2f single_img_points_every, int index_cam,int num_img)
//		:single_obj_points_every_(single_obj_points_every),single_img_points_every_(single_img_points_every),index_cam_(index_cam),num_img_(num_img){}
//
//	template<typename T> bool operator()(const T* const params,
//		T* residual) const
//	{
//		cv::Mat A = cv::Mat::zeros(3,3,CV_32FC1);
//		cv::Mat Rv = cv::Mat::zeros(3, 1, CV_32FC1);
//		cv::Mat R = cv::Mat::zeros(3, 3, CV_32FC1);
//		cv::Mat t = cv::Mat::zeros(3, 1, CV_32FC1);
//		cv::Mat k = cv::Mat::zeros(2, 1, CV_32FC1);
//		
//		int num_Rv = 3;
//		int num_R = 9;
//		int num_t = 3;
//
//		A.at<T>(0,0) = params[0];
//		A.at<T>(1,1) = params[1];
//		A.at<T>(0,1) = params[2];
//		A.at<T>(0,2) = params[3];
//		A.at<T>(1,2) = params[4];
//
//		Rv.at<T>(0, 0) = params[5 + index_cam_ * 3];
//		Rv.at<T>(1, 0) = params[6 + index_cam_ * 3];
//		Rv.at<T>(2, 0) = params[7 + index_cam_ * 3];
//		cv::Rodrigues(Rv, R);
//
//		t.at<T>(0, 0) = params[5 + index_cam_ * 3 + num_Rv * num_img_];
//		t.at<T>(1, 0) = params[6 + index_cam_ * 3 + num_Rv * num_img_];
//		t.at<T>(2, 0) = params[7 + index_cam_ * 3 + num_Rv * num_img_];
//
//		k.at<T>(0, 0) = params[5 + num_Rv * num_img_ + num_t * num_img_];
//		k.at<T>(1, 0) = params[6 + num_Rv * num_img_ + num_t * num_img_];
//
//		//转为齐次坐标
//		cv::Point3f homo_single_obj_points_every;
//		homo_single_obj_points_every = cv::Point3f(single_obj_points_every_.x, single_obj_points_every_.y, 1);
//
//		R.col(2) = 0;
//		R.col(2) = 0 + t;
//
//		cv::Mat ARt = A * R;
//
//		///////重投影
//		cv::Mat reprojective_p = ARt * cv::Mat(homo_single_obj_points_every);
//		cv::Point3f re_p = cv::Point3f(reprojective_p);
//		cv::Point2f re_pp = cv::Point2f(re_p.x / re_p.z, re_p.y / re_p.z);
//
//		float fx = A.at<float>(0, 0);
//		float fy = A.at<float>(1, 1);
//		float skew = A.at<float>(0, 1);
//		float cx = A.at<float>(0, 2);
//		float cy = A.at<float>(1, 2);
//
//		//归一化
//		re_pp = cv::Point2f(re_pp.x - cx, re_pp.y - cy);
//		cv::Point2f re_n = cv::Point2f((re_pp.x - skew * re_pp.y) / fx, re_pp.y / fy);
//
//		float r2 = re_n.x*re_n.x + re_n.y*re_n.y;
//
//		float params_dis;
//		switch (k.rows)
//		{
//		case 2:
//			params_dis = 1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2;
//			break;
//		case 3:
//			params_dis = 1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2 + k.at<float>(2)*r2*r2*r2;
//			break;
//		case 4:
//			params_dis = 1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2 + k.at<float>(2)*r2*r2*r2 + k.at<float>(3)*r2*r2*r2*r2;
//			break;
//		case 5:
//			params_dis = 1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2 + k.at<float>(2)*r2*r2*r2 + k.at<float>(3)*r2*r2*r2*r2 + k.at<float>(4)*r2*r2*r2*r2*r2;
//			break;
//		default:
//			cout << " k dimension is not the choosed range" << endl;
//		}
//
//		//加入畸变
//		cv::Point2f re_d = cv::Point2f(re_n.x*params_dis, re_n.y*params_dis);
//
//		//坐标转变回来
//		re_d = cv::Point2f(re_d.x*fx + cx + skew * re_d.y, re_d.y*fy + cy);
//
//		residual[0] = single_img_points_every_.x;
//		//residual[0] = single_img_points_every_.x - re_d.x;
//		//residual[1] = single_img_points_every_.y - re_d.y;
//
//		return true;
//
//	}
//		
//private:
//	const cv::Point3f single_obj_points_every_;
//	const cv::Point2f single_img_points_every_;
//	const int index_cam_;
//	const int num_img_;
//};
#include <opencv2/core/eigen.hpp>

struct ProjectiveResidual {
	ProjectiveResidual(double ox, double oy, double ix, double iy,  int index_cam, int num_img)
		:ox_(ox), oy_(oy),ix_(ix),iy_(iy), index_cam_(index_cam), num_img_(num_img) {}

	template<typename T> bool operator()(const T* const params,
		T* residual) const
	{

		Eigen::Matrix<T, 3, 1> single_obj_points_every_e;
		Eigen::Matrix<T, 2, 1> single_img_points_every_e;

		//cv::Point3d single_obj_points_every;
		//cv::Point2d single_img_points_every;

		//for (int p_i = 0; p_i < 67; p_i++)
		//{
		//	cout << params[p_i] << endl;
		//}
		//cout << params << endl;

		single_obj_points_every_e(0) = T(ox_);
		single_obj_points_every_e(1) = T(oy_);
		single_obj_points_every_e(2) = T(0);
		single_img_points_every_e(0) = T(ix_);
		single_img_points_every_e(1) = T(iy_);

		//single_obj_points_every.x = ox_;
		//single_obj_points_every.y = oy_;
		//single_obj_points_every.z = 0;
		//single_img_points_every.x = ix_;
		//single_img_points_every.y = iy_;

		//Eigen::Matrix3 AA;
		Eigen::Matrix<T, 3, 3>A_e;
		Eigen::Matrix<T, 3, 1>Rv_e;
		Eigen::Matrix<T, 3, 3>R_e;
		Eigen::Matrix<T, 3, 1>t_e;
		Eigen::Matrix<T, 2, 1>k_e;

		//cv::Mat A = cv::Mat::zeros(3, 3, CV_64FC1);
		//cv::Mat Rv; // = cv::Mat::zeros(3, 1, CV_64FC1);
		//cv::Mat R; // = cv::Mat::zeros(3, 3, CV_64FC1);
		//cv::Mat t; // = cv::Mat::zeros(3, 1, CV_64FC1);
		//cv::Mat k; // = cv::Mat::zeros(2, 1, CV_64FC1);
		
		int num_Rv = 3;
		int num_R = 9;
		int num_t = 3;

		
		A_e(0, 0) = params[0];
		A_e(1, 1) = params[1];
		A_e(0, 1) = params[2];
		A_e(0, 2) = params[3];
		A_e(1, 2) = params[4];
		A_e(2, 2) = T(1);
		A_e(1, 0) = T(0);
		A_e(2, 0) = T(0);
		A_e(2, 1) = T(0);

		//A.at<T>(0, 0) = params[0];
		////cout << params[0] << endl;
		//A.at<T>(1, 1) = params[1];
		//A.at<T>(0, 1) = params[2];
		//A.at<T>(0, 2) = params[3];
		//A.at<T>(1, 2) = params[4];
		//A.at<T>(2, 2) = T(1);

		//cout << A_e.cols() << endl;
		//cout << A_e << endl;

		Rv_e(0, 0) = params[5 + index_cam_ * 3];
		Rv_e(1, 0) = params[6 + index_cam_ * 3];
		Rv_e(2, 0) = params[7 + index_cam_ * 3];
		
		//R_e = Eigen::AngleAxis(Rv_e(2), Eigen::Matrix<T,3,1>::UnitZ())* Eigen::AngleAxis(Rv_e(1),Eigen::Matrix<T,3,1>::UnitY())
		//	*Eigen::AngleAxis(Rv_e(0), Eigen::Matrix<T,3,1>::UnitX());
		R_e = Eigen::AngleAxis<T>(Rv_e(2), Eigen::Matrix<T, 3, 1>::UnitZ())*Eigen::AngleAxis<T>(Rv_e(1), Eigen::Matrix<T, 3, 1>::UnitY())
			*Eigen::AngleAxis<T>(Rv_e(0), Eigen::Matrix<T, 3, 1>::UnitX());
		//cout << R_e << endl;

		//Rv.at<T>(0, 0) = params[5 + index_cam_ * 3];
		//Rv.at<T>(1, 0) = params[6 + index_cam_ * 3];
		//Rv.at<T>(2, 0) = params[7 + index_cam_ * 3];
		//cv::Rodrigues(Rv, R);
		//cout << R << endl;


		t_e(0) = params[5 + index_cam_ * 3 + num_Rv * num_img_];
		t_e(1) = params[6 + index_cam_ * 3 + num_Rv * num_img_];
		t_e(2) = params[7 + index_cam_ * 3 + num_Rv * num_img_];

		//t.at<T>(0, 0) = params[5 + index_cam_ * 3 + num_Rv * num_img_];
		//t.at<T>(1, 0) = params[6 + index_cam_ * 3 + num_Rv * num_img_];
		//t.at<T>(2, 0) = params[7 + index_cam_ * 3 + num_Rv * num_img_];

		k_e(0) = params[5 + num_Rv * num_img_ + num_t * num_img_];
		k_e(1) = params[6 + num_Rv * num_img_ + num_t * num_img_];

		//k.at<T>(0, 0) = params[5 + num_Rv * num_img_ + num_t * num_img_];
		//k.at<T>(1, 0) = params[6 + num_Rv * num_img_ + num_t * num_img_];

		Eigen::Matrix<T, 3, 1> homo_single_obj_point_every_e;
		homo_single_obj_point_every_e(0) = single_obj_points_every_e(0);
		homo_single_obj_point_every_e(1) = single_obj_points_every_e(1);
		homo_single_obj_point_every_e(2) = T(1);

		////转为齐次坐标
		//cv::Point3d homo_single_obj_points_every;
		//homo_single_obj_points_every = cv::Point3d(single_obj_points_every.x, single_obj_points_every.y, 1);

		R_e.col(2) = t_e;
		
		//R.col(2) = 0;
		//R.col(2) = 0 + t;

		Eigen::Matrix<T, 3, 3>ARt_e = A_e * R_e;

		//cv::Mat ARt = A * R;

		Eigen::Matrix<T, 3, 1> reprojective_p_e;
		reprojective_p_e = ARt_e * homo_single_obj_point_every_e;
		Eigen::Matrix<T, 2, 1>re_pp_e;
		re_pp_e(0) = reprojective_p_e(0) / reprojective_p_e(2);
		re_pp_e(1) = reprojective_p_e(1) / reprojective_p_e(2);


		///////重投影
		//cv::Mat reprojective_p = ARt * cv::Mat(homo_single_obj_points_every);
		//cv::Point3d re_p = cv::Point3d(reprojective_p);
		//cv::Point2d re_pp = cv::Point2d(re_p.x / re_p.z, re_p.y / re_p.z);

		T fx_e = A_e(0, 0);
		T fy_e = A_e(1, 1);
		T skew_e = A_e(0, 1);
		T cx_e = A_e(0, 2);
		T cy_e = A_e(1, 2);

		//double fx = A.at<double>(0, 0);
		//double fy = A.at<double>(1, 1);
		//double skew = A.at<double>(0, 1);
		//double cx = A.at<double>(0, 2);
		//double cy = A.at<double>(1, 2);

		re_pp_e(0) = re_pp_e(0) - cx_e;
		re_pp_e(1) = re_pp_e(1) - cy_e;
		Eigen::Matrix<T, 2, 1> re_n_e;
		re_n_e(0) = (re_pp_e(0) - skew_e * re_pp_e(1)) / fx_e;
		re_n_e(1) = re_pp_e(1) / fy_e;


		////归一化
		//re_pp = cv::Point2d(re_pp.x - cx, re_pp.y - cy);
		//cv::Point2d re_n = cv::Point2d((re_pp.x - skew * re_pp.y) / fx, re_pp.y / fy);

		T r2_e = re_n_e(0)*re_n_e(0) + re_n_e(1)*re_n_e(1);

		//double r2 = re_n.x*re_n.x + re_n.y*re_n.y;

		T params_dis_e;
		switch (k_e.rows())
		{
		case 2:
			params_dis_e = T(1) + k_e(0)*r2_e + k_e(1)*r2_e*r2_e; break;
		case 3:
			params_dis_e = T(1) + k_e(0)*r2_e + k_e(1)*r2_e*r2_e + k_e(2)*r2_e*r2_e*r2_e; break;
		case 4:
			params_dis_e = T(1) + k_e(0)*r2_e + k_e(1)*r2_e*r2_e + k_e(2)*r2_e*r2_e*r2_e + k_e(3)*r2_e*r2_e*r2_e*r2_e; break;
		case 5:
			params_dis_e = T(1) + k_e(0)*r2_e + k_e(1)*r2_e*r2_e + k_e(2)*r2_e*r2_e*r2_e + k_e(3)*r2_e*r2_e*r2_e*r2_e + k_e(4)*r2_e*r2_e*r2_e*r2_e*r2_e; break;
		default:
			break;
		}

		//double params_dis;
		//switch (k.rows)
		//{
		//case 2:
		//	params_dis = 1 + k.at<double>(0)*r2 + k.at<double>(1)*r2*r2;
		//	break;
		//case 3:
		//	params_dis = 1 + k.at<double>(0)*r2 + k.at<double>(1)*r2*r2 + k.at<double>(2)*r2*r2*r2;
		//	break;
		//case 4:
		//	params_dis = 1 + k.at<double>(0)*r2 + k.at<double>(1)*r2*r2 + k.at<double>(2)*r2*r2*r2 + k.at<double>(3)*r2*r2*r2*r2;
		//	break;
		//case 5:
		//	params_dis = 1 + k.at<double>(0)*r2 + k.at<double>(1)*r2*r2 + k.at<double>(2)*r2*r2*r2 + k.at<double>(3)*r2*r2*r2*r2 + k.at<double>(4)*r2*r2*r2*r2*r2;
		//	break;
		//default:
		//	cout << " k dimension is not in the choosed range" << endl;
		//}

		Eigen::Matrix<T, 2, 1> re_d_e;
		re_d_e(0) = re_n_e(0)*params_dis_e;
		re_d_e(1) = re_n_e(1)*params_dis_e;

		////加入畸变
		//cv::Point2d re_d = cv::Point2d(re_n.x*params_dis, re_n.y*params_dis);

		re_d_e(0) = re_d_e(0)*fx_e + cx_e + skew_e * re_d_e(1);
		re_d_e(1) = re_d_e(1)*fy_e + cy_e;
		//cout << re_d_e(0) << endl;

		////坐标转变回来
		//re_d = cv::Point2d(re_d.x*fx + cx + skew * re_d.y, re_d.y*fy + cy);

		residual[0] = single_img_points_every_e(0) - re_d_e(0);
		//cout << residual[0] << endl;
		residual[1] = single_img_points_every_e(1) - re_d_e(1);
		//cout << residual[1] << endl;

		//residual[0] = T(single_img_points_every.x) - T(re_d.x);
		//residual[1] = T(single_img_points_every.y) - T(re_d.y);
		//int aaa = 0;

		return true;

	}

private:
	const double ox_;
	const double oy_;
	const double ix_;
	const double iy_;
	const int index_cam_;
	const int num_img_;
};



//struct ExponentialResidual {
//	ExponentialResidual(float x, float y, int index)
//		: x_(x), y_(y), index_(index) {}
//
//	template <typename T> bool operator()(const T* const m,
//		const T* const c,
//		T* residual) const {
//		//residual[0] = T(T(y_) - exp(m[0] * T(x_) + c[0]) + double(index_) );
//		residual[0] = T(T(y_) - exp(m[0] * T(x_) + c[0]));
//		//residual[1] = T(T(y_) - exp(m[0] * T(x_) + c[0]) );
//		return true;
//	}
//
//private:
//	 float x_;
//	 float y_;
//
//	 int index_;
//};








int ceres_nonlinear_op(const vector<vector<cv::Point3f>> &obj_points, \
	const vector<vector<cv::Point2f>> &img_points, \
	cv::Mat A, vector<cv::Mat> R_set, vector<cv::Mat> t_set, cv::Mat k, vector<pair<int, int>> w_h,
	int MAX_ITER);

#endif





