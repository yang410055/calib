
# include "computeHomograph.h"

int computeH(vector< vector< cv::Point3f > > obj_points, vector< vector<cv::Point2f> >img_points, int num_img, vector<cv::Mat> &H_set)
{
	for (int k = 0; k < num_img; k++)
	{
		vector< cv::Point2f > single_frame_img_points;
		vector< cv::Point3f > single_frame_obj_points;
		///  vector< cv::Point2f > single_frame_obj_points2f;  ///未初始化，不能这样使用

		single_frame_img_points = img_points[k];
		single_frame_obj_points = obj_points[k];

		vector< cv::Point2f > single_frame_obj_points2f;

		for (int i = 0; i < single_frame_obj_points.size(); i++)
		{
			 single_frame_obj_points2f.push_back( cv::Point2f(single_frame_obj_points[i].x, single_frame_obj_points[i].y) );
		}

		cv::Mat H;
        H = cv::findHomography(single_frame_obj_points2f, single_frame_img_points);
		//H = cv::findHomography(single_frame_img_points, single_frame_obj_points2f);
		H_set.push_back(H);

	}
	
	return 0;

}
