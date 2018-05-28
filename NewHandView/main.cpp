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
cv::Mat generate_ROIdepthMap(Model* model, Projection *projection);
void GenerateGroundtruth(Model* model);

void main(int argc, char** argv) {
	
	Pose pose(0, 0, 0);
	model = new Model(".\\model\\HandBase.bvh");
	model->init();
	
	_handcontrol->LoadParams("handcontrol.txt");
	//_handcontrol->LoadGlovePose("Handinf3.txt");
	_handcontrol->ControlHand();
	/*GenerateGroundtruth(model);*/
	//_handcontrol->_costfunction.groundtruthmat = generate_depthMap(model, projection);
	//imwrite("handcontrolMat.png", _handcontrol->_costfunction.groundtruthmat);
	LoadgroundtruthMat("handcontrolMat.png");
	//LoadgroundtruthMat("groundtruth3.png");
	_handcontrol->_costfunction.ComputeGroundtruthRoIBinaryMat();


	_cloudpoint.init(_handcontrol->_costfunction.groundtruthmat, 
		_handcontrol->_costfunction.p_center.x, 
		_handcontrol->_costfunction.p_center.y, 
		_handcontrol->_costfunction.ROI_len_x, 
		_handcontrol->_costfunction.ROI_len_y);

	//_cloudpoint.DepthMatToCloudPoint(_handcontrol->_costfunction.groundtruthmat, 241.3, 160, 120);
	_cloudpoint.DepthMatToCloudPoint(_handcontrol->_costfunction.groundtruthmat, 381.8452,382.1713, 264.0945, 217.1487);


	projection->GroundTruthMatCenter_x = _handcontrol->_costfunction.p_center.x;
	projection->GroundTruthMatCenter_y = _handcontrol->_costfunction.p_center.y;
	projection->GroundTruthRoI_lenx = _handcontrol->_costfunction.ROI_len_x;
	projection->GroundTruthRoI_leny = _handcontrol->_costfunction.ROI_len_y;

	_handcontrol->SetPlam_Position(_cloudpoint.PointCloud_center_x, _cloudpoint.PointCloud_center_y, _cloudpoint.PointCloud_center_z); //使用点云的形心作为手摸的初始位置。

	_handcontrol->AddNoiseToPalmRotation();
	_handcontrol->ControlHand();

	SS::SubdivisionTheHand(model, 0);
	_cloudpoint.Compute_Cloud_to_Mesh_Distance();

	cv::Mat generated_mat = generate_ROIdepthMap(model, projection);
	_handcontrol->_costfunction.ComputeCostfunction(generated_mat);
	MixShowResult(_handcontrol->_costfunction.groundtruthROIMat, generated_mat);

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
	//cv::Mat generated_mat = cv::Mat::zeros(240, 320, CV_16UC1);
	cv::Mat generated_mat = cv::Mat::zeros(424, 512, CV_16UC1);
	_handcontrol->ControlHand();

	projection->set_color_index(model);
	projection->compute_current_orientation(model);
	projection->project_3d_to_2d_(model, generated_mat);
	return generated_mat;
}

cv::Mat generate_ROIdepthMap(Model* model, Projection *projection)
{
	//cv::Mat generated_mat = cv::Mat::zeros(240, 320, CV_16UC1);
	cv::Mat generated_mat = cv::Mat::zeros(_handcontrol->_costfunction.ROI_len_y, _handcontrol->_costfunction.ROI_len_x, CV_16UC1);
	_handcontrol->ControlHand();

	projection->set_color_index(model);
	projection->compute_current_orientation(model);
	projection->project_3d_to_2d_when_calc(model, generated_mat);
	return generated_mat;
}

void LoadgroundtruthMat(char* filename)
{
	_handcontrol->_costfunction.groundtruthmat = cv::imread(filename, CV_LOAD_IMAGE_ANYDEPTH);  //这里采用CV_LOAD_IMAGE_UNCHANGED或者CV_LOAD_IMAGE_ANYDEPTH这个模式才可以真确的读入，不然读入都是不正确的，可能和存储的深度值是16位有关系。
	cv::Mat show_depth = cv::Mat::zeros(_handcontrol->_costfunction.groundtruthmat.rows, _handcontrol->_costfunction.groundtruthmat.cols, CV_8UC1);
	for (int i = 0; i < _handcontrol->_costfunction.groundtruthmat.rows; i++)
	{
		for (int j = 0; j < _handcontrol->_costfunction.groundtruthmat.cols; j++) {
			show_depth.at<uchar>(i, j) = _handcontrol->_costfunction.groundtruthmat.at<ushort>(i, j) % 255;
		}
	}
	Moments moment = moments(show_depth, true);
	Point center(moment.m10 / moment.m00, moment.m01 / moment.m00);

	//cout << center.x << "   " << center.y << endl;
	circle(show_depth, center, 8, Scalar(255, 255, 255), CV_FILLED);
	cv::imshow("The groundtruthMat", show_depth);
	cv::waitKey(100);
}

void GenerateGroundtruth(Model* model)
{
	//生成真实值begin ，并且保存真实值为txt文件和png图片文件
	_handcontrol->RandomGenerateParams();
	_handcontrol->ControlHand();
	_handcontrol->SaveParams("groundtruth.txt");
	cv::Mat grounthmat = generate_depthMap( model, projection);
	cv::imwrite("groundtruth.png", grounthmat);
	//生成真实值end
}