
#include "compute2cameraRt.h"


int compute_2camera_R_t(vector<cv::Mat> R_set1,
	vector<cv::Mat> t_set1,
	vector<cv::Mat> R_set2,
	vector<cv::Mat> t_set2,
	cv::Mat &R,
	cv::Mat &t) {


	vector<cv::Mat> R_set;
	vector<cv::Mat> t_set;
	cv::Mat single_R_set;
	cv::Mat single_t_set;

	cv::Mat sum_R = cv::Mat::zeros(3,3,CV_32FC1);
	cv::Mat sum_t = cv::Mat::zeros(3,1,CV_32FC1);


	for (int i = 0; i < R_set1.size(); i++)
	{
		single_R_set = R_set2[i] * R_set1[i].t();
		single_t_set = t_set2[i] - single_R_set * t_set1[i];
		R_set.push_back(single_R_set);
		t_set.push_back(single_t_set);

		//cout << i << ":" << endl;
		//cout << single_R_set << endl;
		//cout << single_t_set << endl;

		//中值

		//均值
		sum_R += single_R_set;
		sum_t += single_t_set;
	}

	R = sum_R / R_set1.size();
	t = sum_t / t_set1.size();

	cout << "R_init:" << R << endl;
	cout << "t_init:" << t << endl;

	return 1;

}
