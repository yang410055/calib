
#include "ceres_refine.h"





int ceres_nonlinear_op(const vector<vector<cv::Point3f>> &obj_points, \
	const vector<vector<cv::Point2f>> &img_points, \
	cv::Mat &A, vector<cv::Mat> &R_set, vector<cv::Mat> &t_set, cv::Mat &k, vector<pair<int, int>> w_h,
	int MAX_ITER)
{


	//%%%%%%%参数初始化

	int num_R = 3;
	int num_t = 3;
	int num_k = k.rows;

	int num_img = img_points.size();

	int num_params = 5 + num_R * num_img + num_t * num_img + num_k;

	//cv::Mat params(num_params, 1, CV_32FC1);
	//
	//params.at<float>(0, 0) = A.at<float>(0, 0);  //fx
	//params.at<float>(1, 0) = A.at<float>(1, 1);  //fy
	//params.at<float>(2, 0) = A.at<float>(0, 1);  //倾斜因子
	//params.at<float>(3, 0) = A.at<float>(0, 2);  //u0
	//params.at<float>(4, 0) = A.at<float>(1, 2);  //v0

	//cv::Mat Rv;
	//for (int i = 0; i < num_img; i++)
	//{
	//	cv::Rodrigues(R_set[i],Rv);

	//	params.at<float>(5 + i * num_R) = Rv.at<float>(0);
	//	params.at<float>(6 + i * num_R) = Rv.at<float>(1);
	//	params.at<float>(7 + i * num_R) = Rv.at<float>(2);

	//	params.at<float>(5 + i * num_t + num_img * num_R) = t_set[i].at<float>(0);
	//	params.at<float>(6 + i * num_t + num_img * num_R) = t_set[i].at<float>(1);
	//	params.at<float>(7 + i * num_t + num_img * num_R) = t_set[i].at<float>(2);

	//}

	//params.at<float>(5 + num_img * num_t + num_img * num_R) = k.at<float>(0);
	//params.at<float>(6 + num_img * num_t + num_img * num_R) = k.at<float>(1);

	//cout << A << endl;
	//cout << "original params:" << endl;
	double params[67];
	params[0] = A.at<float>(0, 0);  //fx
	//cout << params[0] << endl;
    params[1] = A.at<float>(1, 1);  //fy
    params[2] = A.at<float>(0, 1);  //倾斜因子
    params[3] = A.at<float>(0, 2);  //u0
    params[4] = A.at<float>(1, 2);  //v0

    cv::Mat Rv;
	for (int i = 0; i < num_img; i++)
	{
		cv::Rodrigues(R_set[i], Rv);
		cout << R_set[i] << endl;
		cout << Rv << endl;
		params[5 + i * num_R] = Rv.at<float>(0);
		params[6 + i * num_R] = Rv.at<float>(1);
		params[7 + i * num_R] = Rv.at<float>(2);

		params[5 + i * num_R + num_img * num_R] = t_set[i].at<float>(0);
		params[6 + i * num_R + num_img * num_R] = t_set[i].at<float>(1);
		params[7 + i * num_R + num_img * num_R] = t_set[i].at<float>(2);
	}
	
	params[5 + num_img * num_t + num_img * num_R] = k.at<float>(0);
	params[6 + num_img * num_t + num_img * num_R] = k.at<float>(1);




	//%%%%%%%%%%%%%%求解

	Problem problem;

	int kNumObservation = img_points.size()*img_points[0].size();

	for (int i = 0; i < img_points.size(); i++)
		for (int j = 0; j < img_points[i].size(); j++)
		{

			//problem.AddResidualBlock(
			//	new AutoDiffCostFunction<ProjectiveResidual, 2, 67 >(
			//		new ProjectiveResidual(obj_points[i][j], img_points[i][j], i, num_img)),
			//	NULL,
			//	&params);

			ProjectiveResidual *p = new ProjectiveResidual(double(obj_points[i][j].x),
				double(obj_points[i][j].y),
				double(img_points[i][j].x),
				double(img_points[i][j].y),
				i, num_img);
			//
			problem.AddResidualBlock( new AutoDiffCostFunction<ProjectiveResidual,2,67>(p), NULL, params );

			//auto *pp = new AutoDiffCostFunction<ProjectiveResidual, 2, 67>(p);

			
			//problem.AddResidualBlock(pp, NULL, params);
		}
	
	Solver::Options options;
	options.max_num_iterations = 100;
	//options.linear_solver_type = ceres::DENSE_QR;
	//options.linear_solver_type = ceres::DENSE_SCHUR;
	options.minimizer_progress_to_stdout = true;

	//options.nonlinear_conjugate_gradient_type = ceres::FLETCHER_REEVES;
	//options.use_nonmonotonic_steps = true;
	options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
	//options.initial_trust_region_radius = 1e-6;  //迭代次数多了，结果没什么变化
	options.min_trust_region_radius = 1e-16;   
	//options.dogleg_type = ceres::TRADITIONAL_DOGLEG;

	Solver::Summary summary;
	Solve(options, &problem, &summary);
	//std::cout << summary.BriefReport() << "\n";

	//cout << *params << endl;
	//for (int i = 0; i < 67; i++)
	//{
	//	cout << params[i] << endl;
	//}



	A.convertTo(A, CV_64FC1);
	Rv.convertTo(Rv, CV_64FC1);
	for (int i = 0; i < num_img; i++)
	{
		R_set[i].convertTo(R_set[i],CV_64FC1);
		t_set[i].convertTo(t_set[i],CV_64FC1);
	}
	k.convertTo(k,CV_64FC1);

	A.at<double>(0, 0) = params[0];
	A.at<double>(1, 1) = params[1];
	A.at<double>(0, 1) = params[2];
	A.at<double>(0, 2) = params[3];
	A.at<double>(1, 2) = params[4];

	//cout << "comparision:" << endl;
	//cout << A.at<double>(0, 0) << endl;
	//cout << params[0] << endl;

	//cout << "A:" << A << endl;
	//cout << "A:" << endl;
	//cout << A.at<double>(0, 0) << endl;
	//cout << A.at<double>(1, 1) << endl;
	//cout << A.at<double>(0, 1) << endl;
	//cout << A.at<double>(0, 2) << endl;
	//cout << A.at<double>(1, 2) << endl;

	for (int i = 0; i < num_img; i++)
	{
		Rv.at<double>(0) = params[5 + i * num_R];
		Rv.at<double>(1) = params[6 + i * num_R];
		Rv.at<double>(2) = params[7 + i * num_R];
		//cout << Rv << endl;

		cv::Rodrigues(Rv,R_set[i]);

		t_set[i].at<double>(0) = params[5 + i * num_R + num_img * num_R];
		t_set[i].at<double>(1) = params[6 + i * num_R + num_img * num_R];
		t_set[i].at<double>(2) = params[7 + i * num_R + num_img * num_R];
		//cout << t_set[i] << endl;
	}

	k.at<double>(0) = params[5 + num_img * num_t + num_img * num_R];
	k.at<double>(1) = params[6 + num_img * num_t + num_img * num_R];
	//cout << k << endl;

	//由于之前计算误差的函数是float类型的，转变回去
	A.convertTo(A, CV_32FC1);
	//cout << "A float:" << endl;
	//cout << A.at<float>(0, 0) << endl;
	//cout << A.at<float>(1, 1) << endl;
	//cout << A.at<float>(0, 1) << endl;
	//cout << A.at<float>(0, 2) << endl;
	//cout << A.at<float>(1, 2) << endl;
	Rv.convertTo(Rv, CV_32FC1);
	for (int i = 0; i < num_img; i++)
	{
		R_set[i].convertTo(R_set[i], CV_32FC1);
		t_set[i].convertTo(t_set[i], CV_32FC1);
	}
	k.convertTo(k, CV_32FC1);


	//std::cout << "Final params: "  << "\n";
	//for (int i = 0; i < 67; i++)
	//{
	//	cout << params[i] << endl;
	//}

//	params[0] = A.at<float>(0, 0);  //fx
////cout << params[0] << endl;
//	params[1] = A.at<float>(1, 1);  //fy
//	params[2] = A.at<float>(0, 1);  //倾斜因子
//	params[3] = A.at<float>(0, 2);  //u0
//	params[4] = A.at<float>(1, 2);  //v0
//
//	cv::Mat Rv;
//	for (int i = 0; i < num_img; i++)
//	{
//		cv::Rodrigues(R_set[i], Rv);
//		cout << R_set[i] << endl;
//		cout << Rv << endl;
//		params[5 + i * num_R] = Rv.at<float>(0);
//		params[6 + i * num_R] = Rv.at<float>(1);
//		params[7 + i * num_R] = Rv.at<float>(2);
//
//		params[5 + i * num_R + num_img * num_R] = t_set[i].at<float>(0);
//		params[6 + i * num_R + num_img * num_R] = t_set[i].at<float>(1);
//		params[7 + i * num_R + num_img * num_R] = t_set[i].at<float>(2);
//	}
//
//	params[5 + num_img * num_t + num_img * num_R] = k.at<float>(0);
//	params[6 + num_img * num_t + num_img * num_R] = k.at<float>(1);
			

	

	return true;

}
