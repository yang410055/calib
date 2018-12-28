#ifndef COMPUTEV
#define COMPUTEV


#include <iostream>
#include <stdio.h>
#include <math.h>
using namespace std;



#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>



//���������������е�V����
int computeV(vector<cv::Mat> &H_set, cv::Mat &V);

//���㺬�ڲξ���B
int computeB(cv::Mat V, cv::Mat B);

//�����ڲξ���A
int computeA(cv::Mat B, cv::Mat A);


#endif