
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

int computeInitDistortion1(vector< vector< cv::Point2f > > img_points, vector< vector< cv::Point3f > >obj_points,
	cv::Mat A, vector<cv::Mat> R_set, vector<cv::Mat> t_set, vector<pair<int, int>>w_h, cv::Mat &k)
{
	vector<cv::Point2f> single_img_points;
	vector<cv::Point2f> single_img_points_normalize;
	vector<cv::Point2f> reprojective_points;  //重投影点


	cv::Mat D = cv::Mat::zeros(img_points.size()*img_points[0].size() * 2, 2, CV_32FC1);
	cv::Mat d = cv::Mat::zeros(img_points.size()*img_points[0].size() * 2, 1, CV_32FC1);

	float u0 = A.at<float>(0, 2);
	float v0 = A.at<float>(1, 2);

	float fx = A.at<float>(0, 0);
	float fy = A.at<float>(1, 1);
	float skew = A.at<float>(0, 1);



	for (int i = 0; i < img_points.size(); i++)
	{
		single_img_points = img_points[i];

		//cout << A.type() << endl;


	   //把single_img_points变为cv::Point3f 去调用projective2D

		projective2D(A, R_set[i], t_set[i], obj_points[i], reprojective_points);


		//normalizePoints(single_img_points, single_img_points_normalize, w_h[i].first, w_h[i].second);




		for (int j = 0; j < single_img_points.size(); j++)
		{

			cv::Point2f re_pp = cv::Point2f(reprojective_points[j].x - u0, reprojective_points[j].y - v0);
			cv::Point2f re_n = cv::Point2f((re_pp.x - skew * re_pp.y) / fx, re_pp.y / fy);
			float r2 = re_n.x*re_n.x + re_n.y*re_n.y;

			//float r2 = single_img_points_normalize[j].x*single_img_points_normalize[j].x + single_img_points_normalize[j].y*single_img_points_normalize[j].y;
			float r4 = r2 * r2;

			D.at<float>((j + i * img_points.size()) * 2, 0) = (reprojective_points[j].x - u0) * r2;
			D.at<float>((j + i * img_points.size()) * 2, 1) = (reprojective_points[j].x - u0) * r4;
			D.at<float>((j + i * img_points.size()) * 2 + 1, 0) = (reprojective_points[j].y - v0) * r2;
			D.at<float>((j + i * img_points.size()) * 2 + 1, 1) = (reprojective_points[j].y - v0) * r4;

			d.at<float>((j + i * img_points.size()) * 2, 0) = single_img_points[j].x - reprojective_points[j].x;
			d.at<float>((j + i * img_points.size()) * 2 + 1, 0) = single_img_points[j].y - reprojective_points[j].y;

		}

	}


	auto kk = (D.t()*D).inv()*D.t()*d;
	//cout << kk << endl;
	k = kk;



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
	float dr = 1 + k.at<float>(0, 0)*r2 + k.at<float>(1, 0)*r2*r2;
	//float dr = 1 + k.at<float>(0, 0)*r2 + k.at<float>(1, 0)*r2*r2 + k.at<float>(2, 0)*r2*r2*r2 + k.at<float>(3, 0)*r2*r2*r2*r2 + k.at < float >(4,0)*r2*r2*r2*r2*r2 ;
	re_pp_n.x *= dr;

	float re_pp_nn_x = re_pp_n.x*(w / 2) + w / 2;
	
	x = re_pp_nn_x;
	return x;
}

//不同的投影方法，先外参，计算了再内参
float Funcxx(const cv::Point3f &single_obj_points_every, cv::Mat A, const cv::Mat &R, const cv::Mat &t, const cv::Mat &k, int w, int h)
{
	//变为齐次坐标
	cv::Point3f homo_single_obj_points_every;
	homo_single_obj_points_every = cv::Point3f(single_obj_points_every.x, single_obj_points_every.y, 1);

	R.col(2) = 0;
	R.col(2) = 0 + t;

	cv::Mat p1 = R * cv::Mat(homo_single_obj_points_every);
	//cout << p1 << endl;
	p1 /= p1.at<float>(2);
	//cout << p1 << endl;


	float r2 = p1.at<float>(0)*p1.at<float>(0) + p1.at<float>(1)*p1.at<float>(1) + 1;
	float d_r = 1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2;
	//float d_r = (1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2 + k.at<float>(2)*r2*r2*r2 + k.at<float>(3)*r2*r2*r2*r2 + k.at<float>(4)*r2*r2*r2*r2*r2);

	p1.at<float>(0) = p1.at<float>(0)*d_r;
	p1.at<float>(1) = p1.at<float>(1)*d_r;

	cv::Mat p2 = A * p1;
	p2 = p2 / p2.at<float>(2);

	return p2.at<float>(0);

}

// 投影方法再次变化，直接用带R，t的外参矩阵
float Funcxxx(const cv::Point3f &single_obj_points_every, cv::Mat A, const cv::Mat &R, const cv::Mat &t, const cv::Mat &k, int w, int h)
{
	//变为齐次坐标
	cv::Mat homo_single_obj_points_every;
	homo_single_obj_points_every = (cv::Mat_<float>(4, 1) << single_obj_points_every.x, single_obj_points_every.y, single_obj_points_every.z, 1);
	//cout << homo_single_obj_points_every << endl;

	cv::Mat Rt = cv::Mat::zeros(3,4,CV_32FC1);
	R.copyTo(Rt.colRange(0,3));
	//Rt.colRange(0, 3) = R.clone();
	t.copyTo(Rt.colRange(3, 4));
	//Rt.colRange(3, 4) = t.clone();
	//cout << Rt << endl;

	cv::Mat p1 = Rt * cv::Mat(homo_single_obj_points_every);
	p1 /= p1.at<float>(2);
	//cout << p1 << endl;

	float r2 = p1.at<float>(0)*p1.at<float>(0) + p1.at<float>(1)*p1.at<float>(1);// +1;
	float d_r = 1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2;
	//float d_r = (1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2 + k.at<float>(2)*r2*r2*r2 + k.at<float>(3)*r2*r2*r2*r2 + k.at<float>(4)*r2*r2*r2*r2*r2);

	p1.at<float>(0) = p1.at<float>(0)*d_r;
	p1.at<float>(1) = p1.at<float>(1)*d_r;

	cv::Mat p2 = A * p1;
	p2 = p2 / p2.at<float>(2);

	//cout << p2 << endl;

	return p2.at<float>(0);

}

//投影方法按照张正友论文
float Func4x(const cv::Point3f &single_obj_points_every, cv::Mat A, const cv::Mat &R, const cv::Mat &t, const cv::Mat &k, int w, int h)
{
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

	float fx = A.at<float>(0,0);
	float fy = A.at<float>(1,1);
	float skew = A.at<float>(0, 1);
	float cx = A.at<float>(0, 2);
	float cy = A.at<float>(1, 2);

	//归一化
	re_pp = cv::Point2f(re_pp.x - cx, re_pp.y - cy);
	cv::Point2f re_n = cv::Point2f((re_pp.x - skew * re_pp.y) / fx, re_pp.y / fy);

	float r2 = re_n.x*re_n.x + re_n.y*re_n.y;

	float params_dis;
	switch (k.rows)
	{
	case 2:
		params_dis = 1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2;
		break;
	case 3:
		params_dis = 1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2 + k.at<float>(2)*r2*r2*r2;
		break;
	case 4:
		params_dis = 1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2 + k.at<float>(2)*r2*r2*r2 + k.at<float>(3)*r2*r2*r2*r2;
		break;
	case 5:
		params_dis = 1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2 + k.at<float>(2)*r2*r2*r2 + k.at<float>(3)*r2*r2*r2*r2 + k.at<float>(4)*r2*r2*r2*r2*r2;
		break;
	default:
		cout << " k dimension is not the choosed range" << endl; 
	}

	cv::Point2f re_d = cv::Point2f(re_n.x*params_dis, re_n.y*params_dis);

	re_d = cv::Point2f(re_d.x*fx + cx + skew * re_d.y, re_d.y*fy + cy);

	return re_d.x;

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
	//float dr = 1 + k.at<float>(0, 0)*r2 + k.at<float>(1, 0)*r2*r2 + k.at<float>(2, 0)*r2*r2*r2 + k.at<float>(3, 0)*r2*r2*r2*r2 + k.at < float >(4, 0)*r2*r2*r2*r2*r2;
	re_pp_n.y *= dr;

	float re_pp_nn_y = re_pp_n.y*(h / 2) + h / 2;

	y = re_pp_nn_y;
	return y;
}


//不同的投影方法，先外参，计算了再内参
float Funcyy(const cv::Point3f &single_obj_points_every, cv::Mat A, const cv::Mat &R, const cv::Mat &t, const cv::Mat &k, int w, int h)
{
	//变为齐次坐标
	cv::Point3f homo_single_obj_points_every;
	homo_single_obj_points_every = cv::Point3f(single_obj_points_every.x, single_obj_points_every.y, 1);

	R.col(2) = 0;
	R.col(2) = 0 + t;

	cv::Mat p1 = R * cv::Mat(homo_single_obj_points_every);
	//cout << p1 << endl;
	p1 /= p1.at<float>(2);
	//cout << p1 << endl;


	float r2 = p1.at<float>(0)*p1.at<float>(0) + p1.at<float>(1)*p1.at<float>(1) + 1;
	float d_r = 1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2;
	//float d_r = (1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2 + k.at<float>(2)*r2*r2*r2 + k.at<float>(3)*r2*r2*r2*r2 + k.at<float>(4)*r2*r2*r2*r2*r2);

	p1.at<float>(0) = p1.at<float>(0)*d_r;
	p1.at<float>(1) = p1.at<float>(1)*d_r;

	cv::Mat p2 = A * p1;
	p2 = p2 / p2.at<float>(2);

	return p2.at<float>(1);

}

float Funcyyy(const cv::Point3f &single_obj_points_every, cv::Mat A, const cv::Mat &R, const cv::Mat &t, const cv::Mat &k, int w, int h)
{
	//变为齐次坐标
	cv::Mat homo_single_obj_points_every;
	homo_single_obj_points_every = (cv::Mat_<float>(4, 1) << single_obj_points_every.x, single_obj_points_every.y, single_obj_points_every.z, 1);
	//cout << homo_single_obj_points_every << endl;

	cv::Mat Rt = cv::Mat::zeros(3, 4, CV_32FC1);
	R.copyTo(Rt.colRange(0, 3));
	//Rt.colRange(0, 3) = R.clone();
	t.copyTo(Rt.colRange(3, 4));
	//Rt.colRange(3, 4) = t.clone();
	//cout << Rt << endl;

	cv::Mat p1 = Rt * cv::Mat(homo_single_obj_points_every);
	p1 /= p1.at<float>(2);
	//cout << p1 << endl;

	float r2 = p1.at<float>(0)*p1.at<float>(0) + p1.at<float>(1)*p1.at<float>(1); //+1;
	float d_r = 1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2;
	//float d_r = (1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2 + k.at<float>(2)*r2*r2*r2 + k.at<float>(3)*r2*r2*r2*r2 + k.at<float>(4)*r2*r2*r2*r2*r2);

	p1.at<float>(0) = p1.at<float>(0)*d_r;
	p1.at<float>(1) = p1.at<float>(1)*d_r;

	cv::Mat p2 = A * p1;
	p2 = p2 / p2.at<float>(2);

	//cout << p2 << endl;

	return p2.at<float>(1);

}

//投影方法按照张正友论文
float Func4y(const cv::Point3f &single_obj_points_every, cv::Mat A, const cv::Mat &R, const cv::Mat &t, const cv::Mat &k, int w, int h)
{
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

	float fx = A.at<float>(0, 0);
	float fy = A.at<float>(1, 1);
	float skew = A.at<float>(0, 1);
	float cx = A.at<float>(0, 2);
	float cy = A.at<float>(1, 2);

	//归一化
	re_pp = cv::Point2f(re_pp.x - cx, re_pp.y - cy);
	cv::Point2f re_n = cv::Point2f((re_pp.x - skew * re_pp.y) / fx, re_pp.y / fy);

	float r2 = re_n.x*re_n.x + re_n.y*re_n.y;

	float params_dis;
	switch (k.rows)
	{
	case 2:
		params_dis = 1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2;
		break;
	case 3:
		params_dis = 1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2 + k.at<float>(2)*r2*r2*r2;
		break;
	case 4:
		params_dis = 1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2 + k.at<float>(2)*r2*r2*r2 + k.at<float>(3)*r2*r2*r2*r2;
		break;
	case 5:
		params_dis = 1 + k.at<float>(0)*r2 + k.at<float>(1)*r2*r2 + k.at<float>(2)*r2*r2*r2 + k.at<float>(3)*r2*r2*r2*r2 + k.at<float>(4)*r2*r2*r2*r2*r2;
		break;
	default:
		cout << " k dimension is not the choosed range" << endl;
	}

	cv::Point2f re_d = cv::Point2f(re_n.x*params_dis, re_n.y*params_dis);

	re_d = cv::Point2f(re_d.x*fx + cx + skew * re_d.y, re_d.y*fy + cy);

	return re_d.y;

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

//cout << "params" << params << endl;
//cout << "params1:" << params1 << endl;
//cout << params.at<float>(0, 0) - params1.at<float>(0, 0) << endl;

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
	int num_params_singleImg = (A.rows*A.cols - 4) + R_set[0].rows*R_set[0].cols + t_set[0].rows*t_set[0].cols + k.rows*k.cols;;

	cv::Mat r(m,1,CV_32FC1);  //residual
	cv::Mat r_tmp(m, 1, CV_32FC1);

	//cv::Mat J(m, num_params, CV_32FC1) = cv::Mat::zeros(m, num_params, CV_32FC1);
	cv::Mat J = cv::Mat::zeros(m, num_params, CV_32FC1);

	//cv::Point3f single_obj_points_every;

	cv::Mat params(num_params, 1, CV_32FC1);
	params.at<float>(0, 0) = A.at<float>(0, 0);  //fx
	params.at<float>(1, 0) = A.at<float>(1, 1);  //fy
	params.at<float>(2, 0) = A.at<float>(0, 1);  //倾斜因子
	params.at<float>(3, 0) = A.at<float>(0, 2);  //u0
	params.at<float>(4, 0) = A.at<float>(1, 2);  //v0

	int col = 5;
	int num_R = 3 * 3;
	int num_t = 3;
	for (int i = 0;  i < obj_points.size(); i++)
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

		params.at<float>(14 + i * num_t + num_R * (obj_points.size() - 1), 0) = t_set[i].at<float>(0, 0);
		params.at<float>(15 + i * num_t + num_R * (obj_points.size() - 1), 0) = t_set[i].at<float>(1, 0);
		params.at<float>(16 + i * num_t + num_R * (obj_points.size() - 1), 0) = t_set[i].at<float>(2, 0);

	}

	params.at<float>(17 + num_R * (obj_points.size() - 1) + num_t* (obj_points.size() - 1), 0) = k.at<float>(0, 0);
	params.at<float>(18 + num_R * (obj_points.size() - 1) + num_t * (obj_points.size() - 1), 0) = k.at<float>(1, 0);

	cv::Mat params_singleImg(num_params_singleImg, 1, CV_32FC1);

	//cout << "params------------:" << params << endl;
	cv::Mat params_tmp = params.clone();


	float last_mse = 0;
	float u = 1, v = 2;

	cv::Mat I = cv::Mat::eye(num_params, num_params, CV_32FC1);

	cv::Point3f single_obj_points_every;
	for (int i = 0; i < MAX_ITER; i++)
	{
		float mse = 0;
		float mse_temp = 0;

		int n_index = 0;

		for (int j = 0; j < obj_points.size(); j++)
		{


			//把长序列参数 换成 短序列参数
			for (int p1 = 0; p1 < params_singleImg.rows; p1++)
			{
				if (p1 < 5)
					params_singleImg.at<float>(p1, 0) = params.at<float>(p1, 0);
				else if (p1 >= 5 && p1 < (5 + num_R))
					params_singleImg.at<float>(p1, 0) = params.at<float>(p1 + j * num_R, 0);
				else if (p1 >= 5 + num_R && p1 < 5 + num_R + num_t)
					params_singleImg.at<float>(p1, 0) = params.at<float>(p1 + (obj_points.size()-1)*num_R + j * num_t);
				else
					params_singleImg.at<float>(p1, 0) = params.at<float>(p1 + (obj_points.size()-1)*num_R + (obj_points.size()-1) * num_t);
			}
			//cout << "params" << j << endl << params_singleImg << endl;

			//参数放回各个矩阵中
			A.at<float>(0, 0) = params_singleImg.at<float>(0, 0);
			A.at<float>(1, 1) = params_singleImg.at<float>(1, 0);
			A.at<float>(0, 1) = params_singleImg.at<float>(2, 0);
			A.at<float>(0, 2) = params_singleImg.at<float>(3, 0);
			A.at<float>(1, 2) = params_singleImg.at<float>(4, 0);

			R_set[j].at<float>(0, 0) = params_singleImg.at<float>(5, 0);
			R_set[j].at<float>(0, 1) = params_singleImg.at<float>(6, 0);
			R_set[j].at<float>(0, 2) = params_singleImg.at<float>(7, 0);
			R_set[j].at<float>(1, 0) = params_singleImg.at<float>(8, 0);
			R_set[j].at<float>(1, 1) = params_singleImg.at<float>(9, 0);
			R_set[j].at<float>(1, 2) = params_singleImg.at<float>(10, 0);
			R_set[j].at<float>(2, 0) = params_singleImg.at<float>(11, 0);
			R_set[j].at<float>(2, 1) = params_singleImg.at<float>(12, 0);
			R_set[j].at<float>(2, 2) = params_singleImg.at<float>(13, 0);

			t_set[j].at<float>(0, 0) = params_singleImg.at<float>(14, 0);
			t_set[j].at<float>(1, 0) = params_singleImg.at<float>(15, 0);
			t_set[j].at<float>(2, 0) = params_singleImg.at<float>(16, 0);

			k.at<float>(0, 0) = params_singleImg.at<float>(17, 0);
			k.at<float>(1, 0) = params_singleImg.at<float>(18, 0);



			
			for (int kk = 0; kk < obj_points[j].size(); kk++)
			{
				single_obj_points_every = obj_points[j][kk];

				//计算residual
				r.at<float>( 2*(kk+j*obj_points[j].size()) ,0) = Funcx(single_obj_points_every, A, \
					R_set[j], t_set[j], k, w_h[j].first, w_h[j].second) - img_points[j][kk].x;

				mse += r.at<float>(2 * (kk + j * obj_points[j].size()), 0)*r.at<float>(2 * (kk + j * obj_points[j].size()), 0);

				r.at<float>( 2*(kk + j * obj_points[j].size())+1 , 0) = Funcy(single_obj_points_every, A, \
					R_set[j], t_set[j], k, w_h[j].first, w_h[j].second) - img_points[j][kk].y;

				mse += r.at<float>(2 * (kk + j * obj_points[j].size()) + 1, 0)*r.at<float>(2 * (kk + j * obj_points[j].size()) + 1, 0);

				//构造雅克比矩阵

				for (int p_J = 0; p_J < num_params_singleImg; p_J++)
				{
					if (p_J < 5)
						n_index = p_J;
					else if (p_J >= 5 && p_J < (5 + num_R))
						n_index = p_J + j * num_R;
					else if (p_J >= 5 + num_R && p_J < 5 + num_R + num_t)
						n_index = p_J + (obj_points.size() - 1)*num_R + j * num_t;
					else
						n_index = (obj_points.size() - 1)*num_R + (obj_points.size() - 1) * num_t;



					J.at<float>(2 * (kk + j * obj_points[j].size()), n_index) = Deriv(Funcx, single_obj_points_every,\
						A, R_set[j], t_set[j], k, w_h[j].first, w_h[j].second, p_J);
					J.at<float>(2 * (kk + j * obj_points[j].size()) + 1, n_index) = Deriv(Funcy, single_obj_points_every, \
						A, R_set[j], t_set[j], k, w_h[j].first, w_h[j].second, p_J);

				}


			}



		}

		//cout << "JJJJJJJJJJJJJJJJJJJJJJJJJJJJJJ" << endl<<J.row(1)<<endl<<J.row(2)<<endl;
		//cout << "rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr" << endl <<r<< endl;

		mse /= m;

		params_tmp = params.clone();
		cv::Mat hlm = (-1)*(J.t()*J + u * I).inv()*J.t()*r;
		params_tmp += hlm;


		cv::Mat params_singleImg_tmp(num_params_singleImg, 1, CV_32FC1);
		
		for (int j = 0; j < obj_points.size(); j++)
		{
			//把长序列参数 换成 短序列参数
			for (int p1 = 0; p1 < params_singleImg_tmp.rows; p1++)
			{
				if (p1 < 5)
					params_singleImg_tmp.at<float>(p1, 0) = params_tmp.at<float>(p1, 0);
				else if (p1 >= 5 && p1 < (5 + num_R))
					params_singleImg_tmp.at<float>(p1, 0) = params_tmp.at<float>(p1 + j * num_R, 0);
				else if (p1 >= 5 + num_R && p1 < 5 + num_R + num_t)
					params_singleImg_tmp.at<float>(p1, 0) = params_tmp.at<float>(p1 + (obj_points.size() - 1)*num_R + j * num_t);
				else
					params_singleImg_tmp.at<float>(p1, 0) = params_tmp.at<float>(p1 + (obj_points.size() - 1)*num_R + (obj_points.size() - 1) * num_t);
			}

			//参数放回各个矩阵中
			A.at<float>(0, 0) = params_singleImg_tmp.at<float>(0, 0);
			A.at<float>(1, 1) = params_singleImg_tmp.at<float>(1, 0);
			A.at<float>(0, 1) = params_singleImg_tmp.at<float>(2, 0);
			A.at<float>(0, 2) = params_singleImg_tmp.at<float>(3, 0);
			A.at<float>(1, 2) = params_singleImg_tmp.at<float>(4, 0);

			R_set[j].at<float>(0, 0) = params_singleImg_tmp.at<float>(5, 0);
			R_set[j].at<float>(0, 1) = params_singleImg_tmp.at<float>(6, 0);
			R_set[j].at<float>(0, 2) = params_singleImg_tmp.at<float>(7, 0);
			R_set[j].at<float>(1, 0) = params_singleImg_tmp.at<float>(8, 0);
			R_set[j].at<float>(1, 1) = params_singleImg_tmp.at<float>(9, 0);
			R_set[j].at<float>(1, 2) = params_singleImg_tmp.at<float>(10, 0);
			R_set[j].at<float>(2, 0) = params_singleImg_tmp.at<float>(11, 0);
			R_set[j].at<float>(2, 1) = params_singleImg_tmp.at<float>(12, 0);
			R_set[j].at<float>(2, 2) = params_singleImg_tmp.at<float>(13, 0);

			t_set[j].at<float>(0, 0) = params_singleImg_tmp.at<float>(14, 0);
			t_set[j].at<float>(1, 0) = params_singleImg_tmp.at<float>(15, 0);
			t_set[j].at<float>(2, 0) = params_singleImg_tmp.at<float>(16, 0);

			k.at<float>(0, 0) = params_singleImg_tmp.at<float>(17, 0);
			k.at<float>(1, 0) = params_singleImg_tmp.at<float>(18, 0);



		}




		for (int j = 0; j < obj_points.size(); j++)
		{
			for (int kk = 0; kk < obj_points[j].size(); kk++)
			{

				single_obj_points_every = obj_points[j][kk];
				//计算residual
				r_tmp.at<float>(2 * (kk + j * obj_points[j].size()), 0) = Funcx(single_obj_points_every, A, \
					R_set[j], t_set[j], k, w_h[j].first, w_h[j].second) - img_points[j][kk].x;

				mse_temp += r_tmp.at<float>(2 * (kk + j * obj_points[j].size()), 0)*r_tmp.at<float>(2 * (kk + j * obj_points[j].size()), 0);

				r_tmp.at<float>(2 * (kk + j * obj_points[j].size()) + 1, 0) = Funcy(single_obj_points_every, A, \
					R_set[j], t_set[j], k, w_h[j].first, w_h[j].second) - img_points[j][kk].y;

				mse_temp += r_tmp.at<float>(2 * (kk + j * obj_points[j].size()) + 1, 0)*r_tmp.at<float>(2 * (kk + j * obj_points[j].size()) + 1, 0);
			}
		}
		mse_temp /= m;




		cv::Mat q(1, 1, CV_64F);
		q = (mse - mse_temp) / (0.5*hlm.t()*(u*hlm - J.t()*r));

		float q_value = q.at<float>(0, 0);

		if (q_value > 0)
		{
			double s = 1.0 / 3.0;
			v = 2;
			mse = mse_temp;
			params = params_tmp;

			double temp = 1 - pow(2 * q_value - 1, 3);
			if (s > temp)
			{
				u = u * s;
			}
			else
			{
				u = u * temp;
			}
		}
		else
		{
			u = u * v;
			v = 2 * v;
			mse = mse_temp;           
			params = params_tmp;
		}

		if (fabs(mse - last_mse) < 1e-12)     //第一次的值为1e-8
		{
			cout << "reason: mse-last_mse" << endl;
			cout << "the final:" << params << endl;

			break;
		}

		if (mse < 1e-12)        //mse很小，跳出吧
		{
			cout << "reason: mse is small" << endl;
			break;
		}

		last_mse = mse;




		int ssss = 0;

	}

}
	



//const vector<vector<cv::Point3f>> &obj_points, \
const vector<vector<cv::Point2f>> &img_points, \
cv::Mat A, vector<cv::Mat> R_set, vector<cv::Mat> t_set, cv::Mat k, vector<pair<int, int>> w_h



//float Funcx(const cv::Point3f &single_obj_points_every, cv::Mat A, const cv::Mat &R, const cv::Mat &t, const cv::Mat &k, int w, int h)

float computeReprojectionErrors(const vector<vector<cv::Point3f>> &obj_points,
	const vector<vector<cv::Point2f>> &img_points, 
	cv::Mat A, vector<cv::Mat> R_set, vector<cv::Mat> t_set, cv::Mat k, vector<pair<int, int>> w_h)
{
	int num_Img = obj_points.size();
	int num_Point_per_Img;

	cv::Point3f single_obj_points_every;
	float funcx,
		funcy;

	float sub_x,
		sub_y;
	float sub_xy_sqrt;

	//float mean_Error[num_Img] = { 0 };  //num_Img不为常量
	cv::Mat mean_Error_per_Img  = cv::Mat::zeros(num_Img,1,CV_32FC1);
	float mean_Error_total = 0;

	for (int i = 0; i < num_Img; i++)
	{
		num_Point_per_Img = obj_points[i].size();
		for (int j = 0; j < num_Point_per_Img; j++)
		{
			single_obj_points_every = obj_points[i][j];
			funcx = Func4x(single_obj_points_every, A, R_set[i], t_set[i], k, w_h[i].first, w_h[i].second);
			funcy = Func4y(single_obj_points_every, A, R_set[i], t_set[i], k, w_h[i].first, w_h[i].second);
			sub_x = funcx - img_points[i][j].x;
			sub_y = funcy - img_points[i][j].y;
			sub_xy_sqrt = sqrt(sub_x*sub_x + sub_y * sub_y);
			mean_Error_per_Img.at<float>(i,0) += sub_xy_sqrt;
		}
		mean_Error_per_Img.at<float>(i,0) /= num_Point_per_Img;
		mean_Error_total += mean_Error_per_Img.at<float>(i, 0);
	}
	mean_Error_total /= num_Img;

	return mean_Error_total;
}