
#include "Refine.h"




int projective2D(cv::Mat A, cv::Mat R, cv::Mat t, vector<cv::Point3f> single_obj_points, vector<cv::Point2f> &reprojective_points)
{

	/////ת��Ϊ�������
	vector<cv::Point3f> homo_single_obj_points;
	for (int i = 0; i < single_obj_points.size(); i++)
	{
		homo_single_obj_points.push_back(cv::Point3f( single_obj_points[i].x, single_obj_points[i].y, 1 ) );
	}



	///// ��ת������
	R.col(2) = 0;
	//cout << "t:" << t << endl;
	R.col(2) = 0 + t;
	//cout << "R:" << R << endl;
	A.at<float>(0, 1) = 0;          ///////////////����б������Ϊ0,��ֱ���Ͽ��о�������Ӧ�üӻ��߲���
	//cout << "A:" << A << endl;
	//cout <<"A type:"<< A.type() << endl;

	cv::Mat ARt = A * R;
	//cout <<"ARt:" <<ARt << endl;
	

	///////��ͶӰ
	for (int i = 0; i < single_obj_points.size(); i++)
	{
		cv::Mat reprojective_p = ARt * cv::Mat(homo_single_obj_points[i]);
		cv::Point3f re_p = cv::Point3f(reprojective_p);
		cv::Point2f re_pp = cv::Point2f(re_p.x/re_p.z, re_p.y/re_p.z);

		reprojective_points.push_back(re_pp);
	}


	return 1;
}