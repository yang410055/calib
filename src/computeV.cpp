
# include "computeV.h"

///////////////定义计算v向量函数，因为在V中，它要转为行向量，所以这儿直接计算的就是行向量
cv::Mat computeSmallv( cv::Mat H, int i, int j );

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


		V.rowRange(i * 2, i * 2 + 1) += computeSmallv( H_s_t, 1, 2 );    //不能用这儿的引索序号去-1，因为在函数中有常数序号，那儿就没有实现-1
		//std::cout << computeSmallv(H_s_t, 1, 2) << std::endl;
		//std::cout << V << std::endl;

		V.rowRange(i * 2 + 1, i * 2 + 2) += computeSmallv(H_s_t, 1, 1) - computeSmallv(H_s_t, 2, 2 );
		
		//std::cout << computeSmallv(H_s_t, 1, 1) << std::endl;
		//std::cout << computeSmallv(H_s_t, 2, 2) << std::endl;
		//std::cout << V << std::endl;
	}

	return 1;

}


cv::Mat computeSmallv( cv::Mat H, int i, int j )
{
	cv::Mat v = cv::Mat::zeros(1,6, CV_32FC1);
	//v.rowRange(0, 1) = (cv::Mat_<float>(1, 6) << 0, 1, 2, 3, 4, 5);

	//std::cout << H << endl;
	//std::cout << H.at<double>(i, 1) << endl;   ///由于H.type()返回值为6，对应的是CV_64FC1,为double类型
	//std::cout << H.row(0) << endl;
	//std::cout << H.type() << std::endl;

	//上面得到的H为CV_64FC1,下面取元素用了.at<float>，所以这儿先转化为float类型的Mat
	H.convertTo(H, CV_32FC1);
	//std::cout << H.type() << endl;
	//std::cout << H.at<float>(i, 1) << endl;

	//v.row(0)
	////这儿第一次用的v.rowRange(0,1),结果得到的v为0，没有赋值？ row(0)结果也一样,  手册上给出的解释row是临时header ，赋值很远，可以对其本身运算 比如这儿+=是有效的
	////  用v直接来作为赋值也可以
	v.row(0) += (cv::Mat_<float>(1, 6) << H.at<float>(i-1,1-1)*H.at<float>(j-1,1-1), H.at<float>(i-1,1-1)*H.at<float>(j-1,2-1)+H.at<float>(i-1,2-1)*H.at<float>(j-1,1-1),
		            H.at<float>(i-1,2-1)*H.at<float>(j-1,2-1), H.at<float>(i-1,3-1)*H.at<float>(j-1,1-1)+H.at<float>(i-1,1-1)*H.at<float>(j-1,3-1),\
		            H.at<float>(i-1,3-1)*H.at<float>(j-1,2-1)+H.at<float>(i-1,2-1)*H.at<float>(j-1,3-1), H.at<float>(i-1,3-1)*H.at<float>(j-1,3-1) );



	/*cv::Mat u = (cv::Mat_<float>(1, 6) << H.at<float>(i,1)*H.at<float>(j,1), H.at<float>(i, 1)*H.at<float>(j, 2) + H.at<float>(i, 2)*H.at<float>(j, 1), \
		H.at<float>(i, 2)*H.at<float>(j, 2), H.at<float>(i,3), 5, 6);*/
	//std::cout << H.at<float>(i - 1, 1 - 1)*H.at<float>(j - 1, 1 - 1) << endl;
	//std::cout << v << endl;
	return v;
 }




//计算含内参矩阵B
int computeB(cv::Mat V, cv::Mat &B)
{
	int num = V.rows;


	/////%%%%%%%%%%%%%%%%%%    最小二乘法，结果和猜想的一样为0
	////std::cout << V.type() << endl;
    // cv::Mat r_v_0 = cv::Mat::zeros(num,1,CV_32FC1);
	//r_v_0 = r_v_0 + pow(10,-10);
	//cv::Mat b = (V.t()*V).inv()*V.t()*r_v_0;  
	//std::cout << b << endl;

	cv::Mat b(6,1,CV_32FC1);
	cv::SVD::solveZ(V,b);
	//std::cout << b << endl;
	B = (cv::Mat_<float>(3, 3) << b.at<float>(1-1,0), b.at<float>(2-1,0), b.at<float>(4-1,0), b.at<float>(2-1,0),b.at<float>(3-1,0),\
		b.at<float>(5-1), b.at<float>(4-1), b.at<float>(5-1), b.at<float>(6-1) );

	//std::cout << B << endl;
	return 1;

}


////计算内参矩阵A
int computeA(cv::Mat B, cv::Mat &A)
{
	float v0 = (B.at<float>(1-1,2-1)*B.at<float>(1-1,3-1)-B.at<float>(1-1,1-1)*B.at<float>(2-1,3-1))/  \
		( B.at<float>(1-1,1-1)*B.at<float>(2-1,2-1)-B.at<float>(1-1,2-1)*B.at<float>(1-1,2-1) );
	float namuda = B.at<float>(3-1,3-1)-( B.at<float>(1-1,3-1)*B.at<float>(1-1,3-1) + \
		v0*( B.at<float>(1-1,2-1)*B.at<float>(1-1,3-1)-B.at<float>(1-1,1-1)*B.at<float>(2-1,3-1) ) )/B.at<float>(1-1,1-1) ;
	float fx = sqrt(namuda/B.at<float>(1-1,1-1));
	float fy = sqrt( namuda*B.at<float>(1-1,1-1)/( B.at<float>(1-1,1-1)*B.at<float>(2-1,2-1)-B.at<float>(1-1,2-1)*B.at<float>(1-1,2-1) ) );
	float gama = (-1)*B.at<float>(1 - 1, 2 - 1)*fx*fx*fy / namuda;
	float u0 = gama * v0 / fy - B.at<float>(1 - 1, 3 - 1)*fx*fx / namuda;   ////////////gama*v0/fx

	A = (cv::Mat_<float>(3,3)<<fx,gama,u0,0,fy,v0,0,0,1 );


	//std::cout << "B:" << B << endl;
	std::cout << "v0:"<<v0 << endl;
	std::cout << "namuda:" << namuda << endl;
	std::cout << "fx:" << fx << endl;
	std::cout << "fy:" << fy << endl;
	std::cout << "gama:" << gama << endl;
	std::cout << "u0:" << u0 << endl;

	return 1;
}


////%%%%%%%%%%%%%计算外参
int computeR_t( cv::Mat A, vector<cv::Mat>H_set, vector<cv::Mat>&R_set, vector<cv::Mat>&t_set)
{
	int num_H = H_set.size();

	cv::Mat H_single;
	cv::Mat h1,h2,h3;

	for (int i = 0; i < num_H; i++)
	{
		H_single = H_set[i];
		//std::cout << "H_single type:" << H_single.type() << endl;
		//H_single.convertTo(H_single, CV_32FC1);

		h1 = 0 + H_single.col(0);  
		h2 = 0 + H_single.col(1);
		h3 = 0 + H_single.col(2);
		//std::cout << "h1 type:" << h1.type() << endl;
		h1.convertTo(h1, CV_32FC1);
		h2.convertTo(h2, CV_32FC1);
		h3.convertTo(h3, CV_32FC1);
		//std::cout << h1 << endl;
		//std::cout << h2 << endl;
		//std::cout << h3 << endl;

		//std::cout << "A:" << A << endl;
		//std::cout << A.inv() << endl;


		//cv::Mat tt = A.inv()*h1;
		//std::cout <<"tt:"<<tt << endl;
        //A.convertTo(A, CV_64FC1);
		//std::cout << A.type() << endl;
		//////要保持Mat元素类型一致，不然矩阵计算时会出错，double和float都要转变为一样

		cv::Mat R_single = cv::Mat::zeros(3,3,CV_32FC1);
		cv::Mat t_single = cv::Mat::zeros(3,1,CV_32FC1);

		float lambda1 = 1 / norm(A.inv()*h1);
		float lambda2 = 1 / norm(A.inv()*h2);

		R_single.col(0) = 0 + lambda1*A.inv()*h1;
		R_single.col(1) = 0 + lambda1 * A.inv()*h2;
		R_single.col(2) = 0 + R_single.col(0).cross(R_single.col(1));

		t_single = 0 + lambda1 * A.inv()*h3;

		R_set.push_back(R_single);
		t_set.push_back(t_single);

		//std::cout << "R:" << R_single << endl;
		std::cout << "t:" << t_single << endl;

		

	}


	return 1;
}