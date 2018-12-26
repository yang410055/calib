
# include computeHomograph

int computeH(vector< vector< cv::Point3f > > &obj_points, vector< vector<cv::Point2f> >img_points, int num_img, vector<cv::Mat H> &H_set)
{
	for (int k = 0; k < num_img; k++)
	{
		vector< cv::Point2f > single_frame_img_points;
		vector< cv::Point3f > single_frame_obj_points;


        cv::findHomography();
	}
	


}
