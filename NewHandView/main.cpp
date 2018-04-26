#include "Model.h"
#include <iostream>
#include "Display.h"
#include <iostream>
#include "Viewer.h"
#include "Projection.h"
#include <algorithm>
#include "SubdivisionSurfaces.h"
#include "CloudPoint.h"
#include "HandControl.h"


void LoadgroundtruthMat(char* filename);
float Compute_area(cv::Mat input);
float Compute_targetFunction(float area1, cv::Mat input);
cv::Mat generate_depthMap(Model* model, Projection *projection);
void GenerateGroundtruth(Model* model);

void main(int argc, char** argv) {
	
	Pose pose(0, 0, 0);
	model = new Model(".\\model\\HandBase.bvh");
	model->init();

	handcontrol.LoadParams("handcontrol.txt");
	handcontrol.ControlHand();
	/*GenerateGroundtruth(model);*/
	DS::groundtruthmat = generate_depthMap(model, projection);

	float a = ComputeSilhouetteDifference(DS::groundtruthmat, DS::groundtruthmat);
	

	SS::SubdivisionTheHand(model, 0);

	_cloudpoint.init(DS::groundtruthmat);
	_cloudpoint.DepthMatToCloudPoint(DS::groundtruthmat, 241.3, 160, 120);
	_cloudpoint.Compute_Cloud_to_Mesh_Distance();

	cout << _cloudpoint.SumDistance / _cloudpoint.num_cloudpoint << endl;
	cout << _cloudpoint.num_cloudpoint << endl;


	//用于opengl显示
	_data.init(SS::disVertices.size(), SS::disPatches.size());
	_data.SS_set(SS::disVertices, SS::disPatches);
	_data.set_skeleton(model);

	DS::init(argc, argv);
	DS::start();
}

float Compute_area(cv::Mat input)
{
	int HandPixelCount = 0;
	for (int i = 0; i < input.rows; i++) {
		for (int j = 0; j < input.cols; j++) {
			if (input.at<ushort>(i, j) != 0)
				HandPixelCount++;
		}
	}

	return ((float)HandPixelCount) / (input.rows*input.cols);
}

float Compute_targetFunction(float area1, cv::Mat input)
{
	float area2 = Compute_area(input);
	return abs(area1 - area2);
}

cv::Mat generate_depthMap(Model* model, Projection *projection)
{
	cv::Mat generated_mat = cv::Mat::zeros(240, 320, CV_16UC1);
	handcontrol.ControlHand();

	model->forward_kinematic();
	model->compute_mesh();

	projection->set_color_index(model);
	projection->compute_current_orientation(model);
	projection->project_3d_to_2d_(model, generated_mat);
	return generated_mat;
}

void LoadgroundtruthMat(char* filename)
{
	DS::groundtruthmat = cv::imread(filename, CV_LOAD_IMAGE_ANYDEPTH);  //这里采用CV_LOAD_IMAGE_UNCHANGED或者CV_LOAD_IMAGE_ANYDEPTH这个模式才可以真确的读入，不然读入都是不正确的，可能和存储的深度值是16位有关系。
	cv::Mat show_depth = cv::Mat::zeros(DS::groundtruthmat.rows, DS::groundtruthmat.cols, CV_8UC1);
	for (int i = 0; i < DS::groundtruthmat.rows; i++)
	{
		for (int j = 0; j < DS::groundtruthmat.cols; j++) {
			show_depth.at<uchar>(i, j) = DS::groundtruthmat.at<ushort>(i, j) % 255;
		}
	}
	cv::imshow("kkk", show_depth);
	cv::waitKey(100);
}

void GenerateGroundtruth(Model* model)
{
	//生成真实值begin ，并且保存真实值为txt文件和png图片文件
	handcontrol.RandomGenerateParams();
	handcontrol.ControlHand();
	handcontrol.SaveParams("groundtruth.txt");
	cv::Mat grounthmat = generate_depthMap( model, projection);
	cv::imwrite("groundtruth.png", grounthmat);
	//生成真实值end
}