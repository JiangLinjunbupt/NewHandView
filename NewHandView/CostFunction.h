#pragma once
#include "CloudPoint.h"
using namespace cv;

struct CostFunction
{
	float weight1 = 1, weight2 = 1;
	float costfunction;

	float ComputeSilhouetteDifference(cv::Mat rendered_Silhouette, cv::Mat groundtruth_Silhouette)
	{

		cv::Mat rendered_Silhouette_copy = cv::Mat::zeros(rendered_Silhouette.rows, rendered_Silhouette.cols, CV_8UC1);
		for (int i = 0; i < rendered_Silhouette.rows; i++)
		{
			for (int j = 0; j < rendered_Silhouette.cols; j++) {
				if (rendered_Silhouette.at<ushort>(i, j) != 0)
					rendered_Silhouette_copy.at<uchar>(i, j) = 255;
			}
		}

		cv::Mat groundtruth_Silhouette_copy = cv::Mat::zeros(groundtruth_Silhouette.rows, groundtruth_Silhouette.cols, CV_8UC1);
		for (int i = 0; i < groundtruth_Silhouette.rows; i++)
		{
			for (int j = 0; j < groundtruth_Silhouette.cols; j++) {
				if (groundtruth_Silhouette.at<ushort>(i, j) != 0)
					groundtruth_Silhouette_copy.at<uchar>(i, j) = 255;
			}
		}

		threshold(rendered_Silhouette_copy, rendered_Silhouette_copy, 10, 255.0, CV_THRESH_BINARY);          //threshold 的输入图像应该是单通道8位或者32位的，所以要先把16位的图像转换成8位图像
		threshold(groundtruth_Silhouette_copy, groundtruth_Silhouette_copy, 10, 255.0, CV_THRESH_BINARY);


		vector<vector<Point>>contours_rendered_Silhouette;
		vector<Vec4i>hierarchy_rendered_Silhouette;

		vector<vector<Point>>contours_groundtruth_Silhouette;
		vector<Vec4i>hierarchy_groundtruth_Silhouette;

		findContours(rendered_Silhouette_copy, contours_rendered_Silhouette, hierarchy_rendered_Silhouette, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		findContours(groundtruth_Silhouette_copy, contours_groundtruth_Silhouette, hierarchy_groundtruth_Silhouette, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);


		cv::Ptr <cv::HausdorffDistanceExtractor> hausdorff_ptr = cv::createHausdorffDistanceExtractor();
		float distance = hausdorff_ptr->computeDistance(contours_rendered_Silhouette[0], contours_groundtruth_Silhouette[0]);

		//cv::imshow("rendered_Silhouette", rendered_Silhouette_copy);
		//cv::imshow("groundtruth_Silhouette", groundtruth_Silhouette_copy);

		return distance;

	}
	float ComputeCostfunction(cv::Mat rendered_Silhouette, cv::Mat groundtruth_Silhouette)
	{
		this->costfunction = weight1*_cloudpoint.SumDistance / _cloudpoint.num_cloudpoint + weight2*ComputeSilhouetteDifference(rendered_Silhouette, groundtruth_Silhouette);
		return this->costfunction;
	}



};

static CostFunction _costfunction;