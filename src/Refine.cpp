
#include "Refine.h"


int projective2D(cv::Mat A, cv::Mat R, cv::Mat t, vector<cv::Point3f> single_obj_points, vector<cv::Point2f> &reprojective_points)
{

	/////转变为齐次坐标
	vector<cv::Point3f> homo_single_obj_points;
	for (int i = 0; i < single_obj_points.size(); i++)
	{
		homo_single_obj_points.push_back(cv::Point3f( single_obj_points[i].x, single_obj_points[i].y, 1 ) );
	}



	///// 求转换矩阵
	R.col(2) = 0;
	//cout << "t:" << t << endl;
	R.col(2) = 0 + t;
	//cout << "R:" << R << endl;
	//A.at<float>(0, 1) = 0;          ///////////////把倾斜因子设为0,从直观上看感觉不出来应该加或者不加
	//cout << "A:" << A << endl;
	//cout <<"A type:"<< A.type() << endl;

	cv::Mat ARt = A * R;
	//cout <<"ARt:" <<ARt << endl;
	

	///////重投影
	for (int i = 0; i < single_obj_points.size(); i++)
	{
		cv::Mat reprojective_p = ARt * cv::Mat(homo_single_obj_points[i]);
		cv::Point3f re_p = cv::Point3f(reprojective_p);
		cv::Point2f re_pp = cv::Point2f(re_p.x/re_p.z, re_p.y/re_p.z);

		reprojective_points.push_back(re_pp);
	}


	return 1;
}

int normalizePoints(vector<cv::Point2f> single_img_points, vector< cv::Point2f >&single_img_points_normalize, int w, int h)
{

	cv::Point2f p;
	cv::Point2f p_n;
	for (int i = 0; i < single_img_points.size(); i++)
	{
		p = single_img_points[i];
		p_n = cv::Point2f(     ((p.x-w/2)/(w/2)), (p.y-h/2)/(h/2)     );

		single_img_points_normalize.push_back(p_n);
	}


	return 1;
}


int computeInitDistortion(vector< vector< cv::Point2f > > img_points, vector< vector< cv::Point3f > >obj_points,
	cv::Mat A, vector<cv::Mat> R_set, vector<cv::Mat> t_set, vector<pair<int,int>>w_h, cv::Mat &k)
{
	vector<cv::Point2f> single_img_points;
	vector<cv::Point2f> single_img_points_normalize;
	vector<cv::Point2f> reprojective_points;  //重投影点


	cv::Mat D = cv::Mat::zeros(img_points.size()*img_points[0].size()*2,2, CV_32FC1);
	cv::Mat d = cv::Mat::zeros(img_points.size()*img_points[0].size() * 2, 1, CV_32FC1);

	float u0 = A.at<float>(0,2);
	float v0 = A.at<float>(1,2);


	for (int i = 0; i < img_points.size(); i++)
	{
		single_img_points = img_points[i];

		//cout << A.type() << endl;


       //把single_img_points变为cv::Point3f 去调用projective2D

		projective2D(A, R_set[i], t_set[i], obj_points[i], reprojective_points);

		normalizePoints(single_img_points, single_img_points_normalize, w_h[i].first, w_h[i].second);

		//for (int jj = 0; jj < single_img_points_normalize.size(); jj++)
		//{
		//	cout << single_img_points_normalize[jj].x << " "<< single_img_points_normalize[jj].y << endl;
		//}

		for (int j = 0; j < single_img_points.size(); j++)
		{
			float r2 = single_img_points_normalize[j].x*single_img_points_normalize[j].x + single_img_points_normalize[j].y*single_img_points_normalize[j].y;
			float r4 = r2 * r2;

			D.at<float>( ( j + i*img_points.size() ) * 2, 0) = (reprojective_points[j].x - u0) * r2;
			D.at<float>( ( j + i*img_points.size() ) * 2, 1) = (reprojective_points[j].x - u0) * r4;
			D.at<float>( (j + i * img_points.size()) * 2 + 1, 0) = (reprojective_points[j].y - v0) * r2;
			D.at<float>( (j + i * img_points.size()) * 2 + 1, 1) = (reprojective_points[j].y - v0) * r4;

			d.at<float>((j + i * img_points.size()) * 2, 0) = single_img_points[j].x - reprojective_points[j].x;
			d.at<float>((j + i * img_points.size()) * 2 +1, 0) = single_img_points[j].y - reprojective_points[j].y;

		}

	}


	auto kk = (D.t()*D).inv()*D.t()*d;
	//cout << kk << endl;
	k = kk;


	/* *************这儿用opencv解方程异常 **************************************************************/
	//cv::Mat kk1;// = cv::Mat::zeros(2, 1, CV_32FC1);

	//cv::Mat D1 = (cv::Mat_<float>(2, 2) << 1, 1, 2, 1);
	//cv::Mat d1 = (cv::Mat_<float>(2, 1) << 1, 2);

	//kk1 = (D1.t()*D1).inv()*D1.t()*d1;
	//cout << kk1 << endl;
	//cv::solve( D, d, kk1, cv::DECOMP_CHOLESKY);

	//cout << kk1<<endl;
	
	return 1;
}


float Funcx(const cv::Point3f &single_obj_points_every, cv::Mat A, const cv::Mat &R, const cv::Mat &t, const cv::Mat &k, int w, int h)
{
	float x;


	/////转变为齐次坐标
	cv::Point3f homo_single_obj_points_every;
	homo_single_obj_points_every = cv::Point3f(single_obj_points_every.x, single_obj_points_every.y, 1);
	
	///// 求转换矩阵
	R.col(2) = 0;
	//cout << "t:" << t << endl;
	R.col(2) = 0 + t;
	//A.at<float>(0, 1) = 0;          ///////////////把倾斜因子设为0,从直观上看感觉不出来应该加或者不加
	cv::Mat ARt = A * R;


	
	///////重投影
	cv::Mat reprojective_p = ARt * cv::Mat(homo_single_obj_points_every);
	cv::Point3f re_p = cv::Point3f(reprojective_p);
	cv::Point2f re_pp = cv::Point2f(re_p.x / re_p.z, re_p.y / re_p.z);

	//////加畸变

	// 归一化  p_n = cv::Point2f(((p.x - w / 2) / (w / 2)), (p.y - h / 2) / (h / 2)  
	cv::Point2f re_pp_n = cv::Point2f((re_pp.x - w / 2) / (w / 2), (re_pp.y - h / 2) / (h / 2));

	float r2 = re_pp_n.x*re_pp_n.x + re_pp_n.y*re_pp_n.y;
	float dr = 1 + k.at<float>(0,0)*r2 + k.at<float>(1,0)*r2*r2;
	re_pp_n.x *= dr;

	float re_pp_nn_x = re_pp_n.x*(w / 2) + w / 2;
	
	x = re_pp_nn_x;
	return x;
}

float Funcy(const cv::Point3f &single_obj_points_every, cv::Mat A, const cv::Mat &R, const cv::Mat &t, const cv::Mat &k, int w, int h)
{
	float y;


	/////转变为齐次坐标
	cv::Point3f homo_single_obj_points_every;
	homo_single_obj_points_every = cv::Point3f(single_obj_points_every.x, single_obj_points_every.y, 1);

	///// 求转换矩阵
	R.col(2) = 0;
	//cout << "t:" << t << endl;
	R.col(2) = 0 + t;
	//A.at<float>(0, 1) = 0;          ///////////////把倾斜因子设为0,从直观上看感觉不出来应该加或者不加
	cv::Mat ARt = A * R;



	///////重投影
	cv::Mat reprojective_p = ARt * cv::Mat(homo_single_obj_points_every);
	cv::Point3f re_p = cv::Point3f(reprojective_p);
	cv::Point2f re_pp = cv::Point2f(re_p.x / re_p.z, re_p.y / re_p.z);

	//////加畸变

	// 归一化  p_n = cv::Point2f(((p.x - w / 2) / (w / 2)), (p.y - h / 2) / (h / 2)  
	cv::Point2f re_pp_n = cv::Point2f((re_pp.x - w / 2) / (w / 2), (re_pp.y - h / 2) / (h / 2));

	float r2 = re_pp_n.x*re_pp_n.x + re_pp_n.y*re_pp_n.y;
	float dr = 1 + k.at<float>(0, 0)*r2 + k.at<float>(1, 0)*r2*r2;
	re_pp_n.y *= dr;

	float re_pp_nn_y = re_pp_n.y*(h / 2) + h / 2;

	y = re_pp_nn_y;
	return y;
}


float Deriv(float(*Func)(const cv::Point3f &single_obj_points_every, cv::Mat A, \
	const cv::Mat &R, const cv::Mat &t, const cv::Mat &k, int w, int h), \
	const cv::Point3f &single_obj_points_every, \
    cv::Mat A, cv::Mat R, cv::Mat t, cv::Mat k, int w, int h, int n)   //n是选择取第几个参数的雅克比元素
{
	int num_params = (A.rows*A.cols-4) + R.rows*R.cols + t.rows*t.cols + k.rows*k.cols;
	cv::Mat params(num_params, 1, CV_32FC1);

float DERIV_STEP = 0.0001;   //这儿这个值设太小不行？

//自己定义放参数的顺序
params.at<float>(0, 0) = A.at<float>(0, 0);  //fx
params.at<float>(1, 0) = A.at<float>(1, 1);  //fy
params.at<float>(2, 0) = A.at<float>(0, 1);  //倾斜因子
params.at<float>(3, 0) = A.at<float>(0, 2);  //u0
params.at<float>(4, 0) = A.at<float>(1, 2);  //v0

params.at<float>(5, 0) = R.at<float>(0, 0);
params.at<float>(6, 0) = R.at<float>(0, 1);
params.at<float>(7, 0) = R.at<float>(0, 2);
params.at<float>(8, 0) = R.at<float>(1, 0);
params.at<float>(9, 0) = R.at<float>(1, 1);
params.at<float>(10, 0) = R.at<float>(1, 2);
params.at<float>(11, 0) = R.at<float>(2, 0);
params.at<float>(12, 0) = R.at<float>(2, 1);
params.at<float>(13, 0) = R.at<float>(2, 2);

params.at<float>(14, 0) = t.at<float>(0, 0);
params.at<float>(15, 0) = t.at<float>(1, 0);
params.at<float>(16, 0) = t.at<float>(2, 0);

params.at<float>(17, 0) = k.at<float>(0, 0);
params.at<float>(18, 0) = k.at<float>(1, 0);


cv::Mat params1 = params.clone();
cv::Mat params2 = params.clone();

params1.at<float>(n, 0) -= DERIV_STEP;
params2.at<float>(n, 0) += DERIV_STEP;

cout << "params" << params << endl;
cout << "params1:" << params1 << endl;
cout << params.at<float>(0, 0) - params1.at<float>(0, 0) << endl;

//把改变的参数放回去
A.at<float>(0, 0) = params1.at<float>(0, 0);
A.at<float>(1, 1) = params1.at<float>(1, 0);
A.at<float>(0, 1) = params1.at<float>(2, 0);
A.at<float>(0, 2) = params1.at<float>(3, 0);
A.at<float>(1, 2) = params1.at<float>(4, 0);

R.at<float>(0, 0) = params1.at<float>(5, 0);
R.at<float>(0, 1) = params1.at<float>(6, 0);
R.at<float>(0, 2) = params1.at<float>(7, 0);
R.at<float>(1, 0) = params1.at<float>(8, 0);
R.at<float>(1, 1) = params1.at<float>(9, 0);
R.at<float>(1, 2) = params1.at<float>(10, 0);
R.at<float>(2, 0) = params1.at<float>(11, 0);
R.at<float>(2, 1) = params1.at<float>(12, 0);
R.at<float>(2, 2) = params1.at<float>(13, 0);

t.at<float>(0, 0) = params1.at<float>(14, 0);
t.at<float>(1, 0) = params1.at<float>(15, 0);
t.at<float>(2, 0) = params1.at<float>(16, 0);

k.at<float>(0, 0) = params1.at<float>(17, 0);
k.at<float>(1, 0) = params1.at<float>(18, 0);
float p1 = Func(single_obj_points_every, A, R, t, k, w, h);
//cout << "A:"<<A << endl;


A.at<float>(0, 0) = params2.at<float>(0, 0);
A.at<float>(1, 1) = params2.at<float>(1, 0);
A.at<float>(0, 1) = params2.at<float>(2, 0);
A.at<float>(0, 2) = params2.at<float>(3, 0);
A.at<float>(1, 2) = params2.at<float>(4, 0);

R.at<float>(0, 0) = params2.at<float>(5, 0);
R.at<float>(0, 1) = params2.at<float>(6, 0);
R.at<float>(0, 2) = params2.at<float>(7, 0);
R.at<float>(1, 0) = params2.at<float>(8, 0);
R.at<float>(1, 1) = params2.at<float>(9, 0);
R.at<float>(1, 2) = params2.at<float>(10, 0);
R.at<float>(2, 0) = params2.at<float>(11, 0);
R.at<float>(2, 1) = params2.at<float>(12, 0);
R.at<float>(2, 2) = params2.at<float>(13, 0);

t.at<float>(0, 0) = params2.at<float>(14, 0);
t.at<float>(1, 0) = params2.at<float>(15, 0);
t.at<float>(2, 0) = params2.at<float>(16, 0);

k.at<float>(0, 0) = params2.at<float>(17, 0);
k.at<float>(1, 0) = params2.at<float>(18, 0);
float p2 = Func(single_obj_points_every, A, R, t, k, w, h);
//cout << "A:" << A << endl;

float result = (p2 - p1) / (2 * DERIV_STEP);

return result;
}



void LM(float(*Funcx)(const cv::Point3f &single_obj_points_every, cv::Mat A, \
	const cv::Mat &R, const cv::Mat &t, const cv::Mat &k, int w, int h), \
	float(*Funcy)(const cv::Point3f &single_obj_points_every, cv::Mat A, \
		const cv::Mat &R, const cv::Mat &t, const cv::Mat &k, int w, int h), \
	const vector<vector<cv::Point3f>> &obj_points, \
	const vector<vector<cv::Point2f>> &img_points, \
	cv::Mat A, vector<cv::Mat> R_set, vector<cv::Mat> t_set, cv::Mat k, vector<pair<int, int>> w_h,
	int MAX_ITER)
{

	int m = obj_points.size()*obj_points[0].size() * 2;
	int n = 2;   //输入的维度 这儿是点的横纵坐标两个维度，也可以用一个Point2f表示

	//int num_params = (A.rows*A.cols - 4) + R_set[0].rows*R_set[0].cols + t_set[0].rows*t_set[0].cols + k.rows*k.cols; 
	//这儿R_set和t_set 包含了不同的旋转平移参数，所以不应该是统一的
	int num_params = (A.rows*A.cols - 4) + R_set[0].rows*R_set[0].cols*w_h.size() + t_set[0].rows*t_set[0].cols*w_h.size() + k.rows*k.cols;

	cv::Mat r(m,1,CV_32FC1);  //residual
	cv::Mat r_tmp(m, 1, CV_32FC1);

	cv::Mat J(m, num_params, CV_32FC1);


	cv::Point3f single_obj_points_every;

	cv::Mat params(num_params, 1, CV_32FC1);
	params.at<float>(0, 0) = A.at<float>(0, 0);  //fx
	params.at<float>(1, 0) = A.at<float>(1, 1);  //fy
	params.at<float>(2, 0) = A.at<float>(0, 1);  //倾斜因子
	params.at<float>(3, 0) = A.at<float>(0, 2);  //u0
	params.at<float>(4, 0) = A.at<float>(1, 2);  //v0

	int col = 5;
	int num_R = 3 * 3;
	for (int i = 0; i++; i < w_h.size())
	{
		params.at<float>(5 + i*num_R, 0) = R_set[i].at<float>(0, 0);
		params.at<float>(6 + i*num_R, 0) = R_set[i].at<float>(0, 1);
		params.at<float>(7 + i * num_R, 0) = R_set[i].at<float>(0, 2);
		params.at<float>(8 + i * num_R, 0) = R_set[i].at<float>(1, 0);
		params.at<float>(9 + i * num_R, 0) = R_set[i].at<float>(1, 1);
		params.at<float>(10 + i * num_R, 0) = R_set[i].at<float>(1, 2);
		params.at<float>(11 + i * num_R, 0) = R_set[i].at<float>(2, 0);
		params.at<float>(12 + i * num_R, 0) = R_set[i].at<float>(2, 1);
		params.at<float>(13 + i * num_R, 0) = R_set[i].at<float>(2, 2);

		params.at<float>(14, 0) = t.at<float>(0, 0);
		params.at<float>(15, 0) = t.at<float>(1, 0);
		params.at<float>(16, 0) = t.at<float>(2, 0);

	}


	params.at<float>(5, 0) = R.at<float>(0, 0);
	params.at<float>(6, 0) = R.at<float>(0, 1);
	params.at<float>(7, 0) = R.at<float>(0, 2);
	params.at<float>(8, 0) = R.at<float>(1, 0);
	params.at<float>(9, 0) = R.at<float>(1, 1);
	params.at<float>(10, 0) = R.at<float>(1, 2);
	params.at<float>(11, 0) = R.at<float>(2, 0);
	params.at<float>(12, 0) = R.at<float>(2, 1);
	params.at<float>(13, 0) = R.at<float>(2, 2);
	params.at<float>(14, 0) = t.at<float>(0, 0);
	params.at<float>(15, 0) = t.at<float>(1, 0);
	params.at<float>(16, 0) = t.at<float>(2, 0);
	params.at<float>(17, 0) = k.at<float>(0, 0);
	params.at<float>(18, 0) = k.at<float>(1, 0);
	cv::Mat params_tmp = params.clone();


	float last_mse = 0;
	float u = 1, v = 2;

	cv::Mat I = cv::Mat::eye(num_params, num_params, CV_32FC1);

	cv::Point3f single_obj_points_every;
	for (int i = 0; i < MAX_ITER; i++)
	{
		float mse = 0;
		float mse_temp = 0;

		for (int j = 0; j < obj_points.size(); j++)
		{


			for (int kk = 0; kk < obj_points[j].size(); kk++)
			{
				single_obj_points_every = obj_points[j][kk];

				r.at<float>() = Funcx(single_obj_points_every, A, \
					R_set[j], t_set[j], k, w_h[j].first, w_h[j].second) - img_points[j][kk].x;
			}
		}

	}

}
	