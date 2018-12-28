
#include <iostream>
#include <stdio.h>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "computeHomograph.h"
#include "computeV.h"



cv::Mat img_temp;    

//////找到图像序列的角点，把图像序列的角点存储到img_points中，排列的尺寸存储到obj_points中
void find_chess_corners(int board_width, int board_height, int num_img, float square_size,
	char* img_directory, char* img_filename, char* extension,
	vector< vector< cv::Point3f > > &obj_points,
	vector< vector< cv::Point2f > > &img_points) {

	cv::Size board_size = cv::Size(board_width, board_height);
	for (int k = 1; k <= num_img; k++)
	{
		char img_file[100];
		sprintf(img_file, "%s%s%d%s", img_directory, img_filename, k, extension);
		
		img_temp = cv::imread(img_file, CV_LOAD_IMAGE_COLOR);  //把它定义为全局的，后面再主函数中还会用到img_temp

		cv::Mat gray_temp;
		cv::cvtColor(img_temp, gray_temp, CV_BGR2GRAY);

		vector< cv::Point2f > corners;
		bool found;
		found = cv::findChessboardCorners(img_temp, board_size, corners,
			                              CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);  //提取的角点放到corners中
		
		////corners已经为亚像素，再一次refine
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
				//这儿的三维坐标点和corners中的角点似乎x,y相反，但是将i，j颠倒，并不影响结果的输出，函数中应该还加有横纵轴的判定
			}
		obj_points.push_back(obj);
	}
}



int main(int argc, char **argv)
{
	//cv::Mat img;
	//img = cv::imread("circle.png");
 // 
	////判断是否读到图像
	//if (img.data == nullptr)
	//{
	//	cout << "the image reading is not correct" << endl;
	//	return 0;
	//}


	/********************************圆形标定 ***********************************************/
	/*  图像圆点检测返回false  findCircleGrid()   暂时不知道原因   */
	//定义标定板参数
	//cv::Size patternsize(11,4);  //number of Centers
	//vector<cv::Point2f> centers;
	////检测圆点
	//bool patternfound = cv::findCirclesGrid(img, patternsize, centers, cv::CALIB_CB_ASYMMETRIC_GRID);

	//cv::drawChessboardCorners(img, patternsize, cv::Mat(centers), patternfound);
	//cv::waitKey(0);



	//cv::namedWindow("calib Img", 0);
	//cv::resizeWindow("calib Img",500,500);
	//cv::imshow("calib Img",img);
	//cv::waitKey(0);

	//定义棋盘格标定板图像的参数
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


	find_chess_corners( board_width, board_height, num_img, square_size,
		img_directory,  img_filename, extension,
		 obj_points, img_points );


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


	
	//%%%%%%%%%%%%%%计算图像序列的单应性矩阵
	vector<cv::Mat> H_set;
	int num_img1 = num_img;

	computeH(obj_points, img_points, num_img, H_set);
	for (int i = 0; i < 10; i++)
	{
		cout << i << endl;
		cout << H_set[i] << endl;
	}


	//%%%%%%%%%%%%%%%%%%计算整个图像序列的V矩阵
	cv::Mat V  = cv::Mat::zeros(H_set.size(), 6, CV_32FC1);

	computeV(H_set,V);


	


	system("pause");
	cout << "this is end" << endl;



}