
#include "ceres_refine.h"





int ceres_nonlinear_op(const vector<vector<cv::Point3f>> &obj_points, \
	const vector<vector<cv::Point2f>> &img_points, \
	cv::Mat A, vector<cv::Mat> R_set, vector<cv::Mat> t_set, cv::Mat k, vector<pair<int, int>> w_h,
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

	cout << *params << endl;
	for (int i = 0; i < 67; i++)
	{
		cout << params[i] << endl;
	}


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
			//problem.AddResidualBlock( new AutoDiffCostFunction<ProjectiveResidual,2,67>(p), NULL, &params );

			auto *pp = new AutoDiffCostFunction<ProjectiveResidual, 2, 67>(p);

			
			problem.AddResidualBlock(pp, NULL, params);
		}
	
	Solver::Options options;
	options.max_num_iterations = 100;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	Solver::Summary summary;
	Solve(options, &problem, &summary);
	//std::cout << summary.BriefReport() << "\n";
	std::cout << "Final params: "  << "\n";
	std::cout << params << endl;
			


/*	int kNumObservations = 67;

	double data[] = {
	  0.000000e+00, 1.133898e+00,
	  7.500000e-02, 1.334902e+00,
	  1.500000e-01, 1.213546e+00,
	  2.250000e-01, 1.252016e+00,
	  3.000000e-01, 1.392265e+00,
	  3.750000e-01, 1.314458e+00,
	  4.500000e-01, 1.472541e+00,
	  5.250000e-01, 1.536218e+00,
	  6.000000e-01, 1.355679e+00,
	  6.750000e-01, 1.463566e+00,
	  7.500000e-01, 1.490201e+00,
	  8.250000e-01, 1.658699e+00,
	  9.000000e-01, 1.067574e+00,
	  9.750000e-01, 1.464629e+00,
	  1.050000e+00, 1.402653e+00,
	  1.125000e+00, 1.713141e+00,
	  1.200000e+00, 1.527021e+00,
	  1.275000e+00, 1.702632e+00,
	  1.350000e+00, 1.423899e+00,
	  1.425000e+00, 1.543078e+00,
	  1.500000e+00, 1.664015e+00,
	  1.575000e+00, 1.732484e+00,
	  1.650000e+00, 1.543296e+00,
	  1.725000e+00, 1.959523e+00,
	  1.800000e+00, 1.685132e+00,
	  1.875000e+00, 1.951791e+00,
	  1.950000e+00, 2.095346e+00,
	  2.025000e+00, 2.361460e+00,
	  2.100000e+00, 2.169119e+00,
	  2.175000e+00, 2.061745e+00,
	  2.250000e+00, 2.178641e+00,
	  2.325000e+00, 2.104346e+00,
	  2.400000e+00, 2.584470e+00,
	  2.475000e+00, 1.914158e+00,
	  2.550000e+00, 2.368375e+00,
	  2.625000e+00, 2.686125e+00,
	  2.700000e+00, 2.712395e+00,
	  2.775000e+00, 2.499511e+00,
	  2.850000e+00, 2.558897e+00,
	  2.925000e+00, 2.309154e+00,
	  3.000000e+00, 2.869503e+00,
	  3.075000e+00, 3.116645e+00,
	  3.150000e+00, 3.094907e+00,
	  3.225000e+00, 2.471759e+00,
	  3.300000e+00, 3.017131e+00,
	  3.375000e+00, 3.232381e+00,
	  3.450000e+00, 2.944596e+00,
	  3.525000e+00, 3.385343e+00,
	  3.600000e+00, 3.199826e+00,
	  3.675000e+00, 3.423039e+00,
	  3.750000e+00, 3.621552e+00,
	  3.825000e+00, 3.559255e+00,
	  3.900000e+00, 3.530713e+00,
	  3.975000e+00, 3.561766e+00,
	  4.050000e+00, 3.544574e+00,
	  4.125000e+00, 3.867945e+00,
	  4.200000e+00, 4.049776e+00,
	  4.275000e+00, 3.885601e+00,
	  4.350000e+00, 4.110505e+00,
	  4.425000e+00, 4.345320e+00,
	  4.500000e+00, 4.161241e+00,
	  4.575000e+00, 4.363407e+00,
	  4.650000e+00, 4.161576e+00,
	  4.725000e+00, 4.619728e+00,
	  4.800000e+00, 4.737410e+00,
	  4.875000e+00, 4.727863e+00,
	  4.950000e+00, 4.669206e+00,
	};

	double m = 0.0;
	double c = 0.0;
	int inde = 2;

	for (int i = 0; i < kNumObservations; ++i) {
		problem.AddResidualBlock(
			new AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>(
				new ExponentialResidual(data[2 * i], data[2 * i + 1], inde)),
			NULL,
			&m, &c);
	}

	Solver::Options options;
	options.max_num_iterations = 100;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	Solver::Summary summary;
	Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << "\n";
	std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
	std::cout << "Final   m: " << m << " c: " << c << "\n";        */

	int aaa = 0;

	return true;

}
