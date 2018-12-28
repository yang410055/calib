
# include "computeV.h"

///////////////�������v������������Ϊ��V�У���ҪתΪ���������������ֱ�Ӽ���ľ���������
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


		V.rowRange(i * 2, i * 2 + 1) += computeSmallv( H_s_t, 1, 2 );    //������������������ȥ-1����Ϊ�ں������г�����ţ��Ƕ���û��ʵ��-1
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
	//std::cout << H.at<double>(i, 1) << endl;   ///����H.type()����ֵΪ6����Ӧ����CV_64FC1,Ϊdouble����
	//std::cout << H.row(0) << endl;
	//std::cout << H.type() << std::endl;

	//����õ���HΪCV_64FC1,����ȡԪ������.at<float>�����������ת��Ϊfloat���͵�Mat
	H.convertTo(H, CV_32FC1);
	//std::cout << H.type() << endl;
	//std::cout << H.at<float>(i, 1) << endl;

	//v.row(0)
	////�����һ���õ�v.rowRange(0,1),����õ���vΪ0��û�и�ֵ�� row(0)���Ҳһ��,  �ֲ��ϸ����Ľ���row����ʱheader ����ֵ��Զ�����Զ��䱾������ �������+=����Ч��
	////  ��vֱ������Ϊ��ֵҲ����
	v.row(0) += (cv::Mat_<float>(1, 6) << H.at<float>(i-1,1-1)*H.at<float>(j-1,1-1), H.at<float>(i-1,1-1)*H.at<float>(j-1,2-1)+H.at<float>(i-1,2-1)*H.at<float>(j-1,1-1),
		            H.at<float>(i-1,2-1)*H.at<float>(j-1,2-1), H.at<float>(i-1,3-1)*H.at<float>(j-1,1-1)+H.at<float>(i-1,1-1)*H.at<float>(j-1,3-1),\
		            H.at<float>(i-1,3-1)*H.at<float>(j-1,2-1)+H.at<float>(i-1,2-1)*H.at<float>(j-1,3-1), H.at<float>(i-1,3-1)*H.at<float>(j-1,3-1) );



	/*cv::Mat u = (cv::Mat_<float>(1, 6) << H.at<float>(i,1)*H.at<float>(j,1), H.at<float>(i, 1)*H.at<float>(j, 2) + H.at<float>(i, 2)*H.at<float>(j, 1), \
		H.at<float>(i, 2)*H.at<float>(j, 2), H.at<float>(i,3), 5, 6);*/
	//std::cout << H.at<float>(i - 1, 1 - 1)*H.at<float>(j - 1, 1 - 1) << endl;
	//std::cout << v << endl;
	return v;
 }




//���㺬�ڲξ���B
int computeB(cv::Mat V, cv::Mat B)
{
	int num = V.rows;


	/////%%%%%%%%%%%%%%%%%%    ��С���˷�������Ͳ����һ��Ϊ0
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

	std::cout << B << endl;
	return 1;

}


////�����ڲξ���A
int computeA(cv::Mat B, cv::Mat A)
{

}
