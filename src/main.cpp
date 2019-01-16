
#include <iostream>
#include <stdio.h>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "computeHomograph.h"
#include "computeV.h"
#include "compute2cameraRt.h"
#include "Refine.h"



cv::Mat img_temp;    
//vector<pair<int, int>> w_h;   //��ǰ����Ϊȫ�ֵģ�ÿ������������ܲ�һ���ֱ��ʣ����ǵ��������ҽǵ�ĳ����У���Ϊһ�������

//////�ҵ�ͼ�����еĽǵ㣬��ͼ�����еĽǵ�洢��img_points�У����еĳߴ�洢��obj_points��
void find_chess_corners(int board_width, int board_height, int num_img, float square_size,
	char* img_directory, char* img_filename, char* extension,
	vector< vector< cv::Point3f > > &obj_points,
	vector< vector< cv::Point2f > > &img_points,
	vector <pair<int, int>>&w_h ) {

	cv::Size board_size = cv::Size(board_width, board_height);
	for (int k = 1; k <= num_img; k++)
	{
		char img_file[100];
		sprintf(img_file, "%s%s%d%s", img_directory, img_filename, k, extension);
		
		img_temp = cv::imread(img_file, CV_LOAD_IMAGE_COLOR);  //��������Ϊȫ�ֵģ��������������л����õ�img_temp
		w_h.push_back(pair<int,int>(img_temp.cols, img_temp.rows  ));

		cv::Mat gray_temp;
		cv::cvtColor(img_temp, gray_temp, CV_BGR2GRAY);

		vector< cv::Point2f > corners;
		bool found;
		found = cv::findChessboardCorners(img_temp, board_size, corners,
			                              CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);  //��ȡ�Ľǵ�ŵ�corners��
		
		////corners�Ѿ�Ϊ�����أ���һ��refine
		if (found)
		{
			
			cv::cornerSubPix(gray_temp, corners, cv::Size(5,5), cv::Size(-1,-1),
				cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1) );
			cv::drawChessboardCorners(gray_temp, board_size, corners, found);
			//cv::imshow("display_corners", gray_temp);
			//cv::waitKey(0);

			img_points.push_back(corners);
		}

		vector< cv::Point3f > obj;
		for(int i = 0; i < board_height; i++)
			for (int j = 0; j < board_width; j++)
			{
				obj.push_back( cv::Point3f((float)j*square_size, (float)i*square_size,0 ) );  
				//�������ά������corners�еĽǵ��ƺ�x,y�෴�����ǽ�i��j�ߵ�������Ӱ�����������������Ӧ�û����к�������ж�
			}
		obj_points.push_back(obj);
	}
}



int main(int argc, char **argv)
{
	//cv::Mat img;
	//img = cv::imread("circle.png");
 // 
	////�ж��Ƿ����ͼ��
	//if (img.data == nullptr)
	//{
	//	cout << "the image reading is not correct" << endl;
	//	return 0;
	//}


	/********************************Բ�α궨 ***********************************************/
	/*  ͼ��Բ���ⷵ��false  findCircleGrid()   ��ʱ��֪��ԭ��   */
	//����궨�����
	//cv::Size patternsize(11,4);  //number of Centers
	//vector<cv::Point2f> centers;
	////���Բ��
	//bool patternfound = cv::findCirclesGrid(img, patternsize, centers, cv::CALIB_CB_ASYMMETRIC_GRID);

	//cv::drawChessboardCorners(img, patternsize, cv::Mat(centers), patternfound);
	//cv::waitKey(0);



	//cv::namedWindow("calib Img", 0);
	//cv::resizeWindow("calib Img",500,500);
	//cv::imshow("calib Img",img);
	//cv::waitKey(0);

	//�������̸�궨��ͼ��Ĳ���
	int board_width, board_height, num_img;
	float square_size;
	char* img_directory;
	char* img_filename;
	//char* out_file;
	char* extension;

	char* outfile;

	vector< vector< cv::Point3f > > obj_points;
	vector< vector< cv::Point2f > > img_points;

	board_width = 9;   // first valuation is 6
	board_height = 6;  // first valuation is 9
	num_img = 10;
	square_size = 20;
	img_directory = "calib_imgs/1/";
	img_filename = "left";
	extension = ".jpg";

	outfile = "result";

	//%%%%%%%%%%%%%%%%%%%�ڶ������
	vector< vector< cv::Point3f > > obj_points2;
	vector< vector< cv::Point2f > > img_points2;
	char *img_directory2 = "calib_imgs/2/";
	char *img_filename2 = "right";

	vector<pair<int, int>> w_h2;
	find_chess_corners(board_width, board_height, num_img, square_size,
		img_directory2, img_filename2, extension,
		obj_points2, img_points2, w_h2);

	vector<cv::Mat> H_set2;
	int num_img2 = num_img;
	computeH(obj_points2, img_points2, num_img2, H_set2);

	cv::Mat V2 = cv::Mat::zeros(H_set2.size(), 6, CV_32FC1);
	computeV(H_set2, V2);

	cv::Mat B2 = cv::Mat::zeros(3, 3, CV_32FC1);
	computeB(V2, B2);

	cv::Mat A2 = cv::Mat::zeros(3, 3, CV_32FC1);
	computeA(B2, A2);

	vector<cv::Mat> R_set2;
	vector<cv::Mat> t_set2;
	computeR_t(A2, H_set2, R_set2, t_set2);


	//%%%%%%%%%%%%%%%%%%%%%%%%%%


	vector<pair<int, int>> w_h;
	find_chess_corners( board_width, board_height, num_img, square_size,
		img_directory,  img_filename, extension,
		 obj_points, img_points, w_h );


	std::cout << "starting calibration" << endl;
	cv::Mat K;
	cv::Mat D;
	vector< cv::Mat > R_mat, t_vec;

	int flag = 0;
	flag |= CV_CALIB_FIX_K4;
	flag |= CV_CALIB_FIX_K5;


	cv::calibrateCamera(obj_points, img_points, img_temp.size(), K, D, R_mat, t_vec, flag);

	cv::FileStorage fs(outfile, cv::FileStorage::WRITE);
	fs << "K" << K;
	fs << "D" << D;

	std::cout << t_vec[0] << endl;
	std::cout << t_vec[1] << endl;

	std::cout << R_mat[0] << endl;
	std::cout << R_mat[1] << endl;

	std::cout << "calibration done" << std::endl;


	
	//%%%%%%%%%%%%%%����ͼ�����еĵ�Ӧ�Ծ���
	vector<cv::Mat> H_set;
	int num_img1 = num_img;

	computeH(obj_points, img_points, num_img, H_set);
	//for (int i = 0; i < 10; i++)
	//{
	//	cout << i << endl;
	//	cout << H_set[i] << endl;
	//}


	//%%%%%%%%%%%%%%%%%%��������ͼ�����е�V����
	cv::Mat V  = cv::Mat::zeros(H_set.size(), 6, CV_32FC1);

	computeV(H_set,V);

	//cout <<"V:"<< V << endl;
	
	////%%%%%%%%%%%%%%%%%%% ����b����

	cv::Mat B = cv::Mat::zeros(3,3, CV_32FC1);
	computeB(V, B);

	//std::cout << "B:" << B << endl;

	////%%%%%%%%%%%%%%    �����ڲξ���A
	cv::Mat A = cv::Mat::zeros(3,3, CV_32FC1);
	computeA(B,A);
	//std::cout << "A:"<< A << endl;

	//////%%%%%%%%%%%%  �������


	vector<cv::Mat> R_set;
	vector<cv::Mat> t_set;
	computeR_t( A, H_set, R_set, t_set);



	/////%%%%%%�������������תƽ�Ƴ�ֵ
	cv::Mat R_init;
	cv::Mat t_init;
	compute_2camera_R_t(R_set, t_set, R_set2, t_set2, R_init, t_init);



	//////������ͶӰ

	vector<cv::Point2f> reprojective_points;
	projective2D(A, R_set[0], t_set[0], obj_points[0], reprojective_points);

	cout << "reprojective points:" << reprojective_points << endl;
	cout << "--------------------------" << endl;



	cv::Mat k;
	computeInitDistortion(img_points, obj_points,
		A, R_set, t_set, w_h, k);

	cout << "intial k:"<<k << endl;

	cv::Mat k2;
	computeInitDistortion(img_points2, obj_points2,
		A, R_set2, t_set2, w_h2, k2);

	cout << "intial k2:" << k2 << endl;



	cv::Point3f single_obj_points_every;
	single_obj_points_every = (obj_points[0])[0];


	cout << "single point reproject x real:" << img_points[0][0].x << endl;
	cout << "single point reproject y real:" << img_points[0][0].y << endl;

	cout << "single point reproject x without Distortion parameter:" << reprojective_points[0].x << endl;
	cout << "single point reproject y without Distortion parameter:" << reprojective_points[0].y << endl;

	cout <<"single point reproject x:" << Funcx(single_obj_points_every, A, R_set[0], t_set[0], k, w_h[0].first, w_h[0].second) << endl;
	//cv::Mat t1 = t_set[0].clone();
	//t1.at<float>(0, 0) = 0.001 + t1.at<float>(0, 0);
	//cout << "t1-t:" << t1 - t_set[0] << endl;
	//cout << "single point reproject x:" << Funcx(single_obj_points_every, A, R_set[0], t1, k, w_h[0].first, w_h[0].second) << endl;
	cout << "single point reproject y:" << Funcy(single_obj_points_every, A, R_set[0], t_set[0], k, w_h[0].first, w_h[0].second) << endl;

	cout<<"Deriv:"<<
	Deriv(Funcx, single_obj_points_every, A, R_set[0], t_set[0], k, w_h[0].first, w_h[0].second, 15);  //14




	system("pause");
	cout << "this is end" << endl;

	

}