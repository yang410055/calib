
# include "computeV.h"

int computeV(vector<cv::Mat> &H_set, cv::Mat &V)
{
	int num_H = H_set.size();
	V = cv::Mat::zeros(num_H*2, 6, CV_32FC1);

	for (int i = 0; i < num_H; i++)
	{
		cv::Mat H_single = H_set[i];
		cv::Mat H_s_t;
		cv::transpose(H_single, H_s_t);

		//cv::Mat v11(1, 6, float);
		//v11.at<float>(1, 1) = H_s_t.at<float>(1, 1) * H_s_t.at<float>(1, 1);
		//v11.at<float>(1, 2) = H_s_t.at<float>(1, 1) * H_s_t.at<float>(1, ) + 


		V.rowRange(i * 2, i * 2 + 1) = computeSmallv( H_s_t, 1-1, 2-1 );
		V.rowRange(i * 2 + 1, i * 2 + 2) = computeSmallv(H_s_t, 1-1, 1-1) - computeSmallv(H_s_t, 2-1, 2-1 );

	}

	return 1;

}


cv::Mat computeSmallv( cv::Mat H, int i, int j )
{
	cv::Mat v = cv::Mat::zeros(1,6, CV_32FC1);
	v.rowRange(0,1) = (cv::Mat_<double>(1, 6) << H.at<float>(i,1)*H.at<float>(j,1), H.at<float>(i,1)*H.at<float>(j,2)+H.at<float>(i,2)*H.at<float>(j,1),\
		            H.at<float>(i,2)*H.at<float>(j,2), H.at<float>(i,3)*H.at<float>(j,1)+H.at<float>(i,1)*H.at<float>(j,3),\
		            H.at<float>(i,3)*H.at<float>(j,2)+H.at<float>(i,2)*H.at<float>(j,3), H.at<float>(i,3)*H.at<float>(j,3) );
	return v;
 }