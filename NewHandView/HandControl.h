#pragma once

#include "Model.h"
#include "Random.h"
#include "opencv2/objdetect/objdetect.hpp"  
#include "opencv2/features2d/features2d.hpp"  
#include "opencv2/highgui/highgui.hpp"  
#include "opencv2/calib3d/calib3d.hpp"  
#include "opencv2/imgproc/imgproc_c.h"
#include<levmar.h>
class RotateControl
{
private:
	float RotateX;
	float RotateY;
	float RotateZ;

public:
	RotateControl() { this->RotateX = 0; this->RotateY = 0; this->RotateZ = 0; }
	~RotateControl() {}

	float GetRotateX() { return this->RotateX; }
	float GetRotateY() { return this->RotateY; }
	float GetRotateZ() { return this->RotateZ; }
	void SetRotateX(float x) { this->RotateX = x; }
	void SetRotateY(float y) { this->RotateY = y; }
	void SetRotateZ(float z) { this->RotateZ = z; }
	void SetRotateXYZ(float x, float y, float z) { this->RotateX = x; this->RotateY = y; this->RotateZ = z; }
};

class TransControl
{
private:
	float transX;
public:
	TransControl() {this->transX = 0; }
	~TransControl() {}

	float GettransX() { return this->transX; }
	void SettransX(float x) { this->transX = x; }

};

class Finger {
private:
	float fingerscale;
	RotateControl *rotation;
	TransControl *trans;
public:
	Finger() { this->fingerscale = 1; this->rotation = new RotateControl[4]; this->trans = new TransControl[4]; }
	~Finger() { delete rotation; delete trans; }

	float Getfingerscale() { return this->fingerscale; }
	RotateControl* GetRotete() { return this->rotation; }
	TransControl* GetTrans() { return this->trans; }

	void Setfingerscale(float scale) { this->fingerscale = scale; }
	void SetRotate(RotateControl R,int index) { this->rotation[index] = R; }
	void SetRotate(float x, float y, float z, int index) { this->rotation[index].SetRotateXYZ(x, y, z); }
	void SetTrans(TransControl T,int index) { this->trans[index] = T; }
	void SetTrans(float t, int index) { this->trans[index].SettransX(t); }
};

class Palm {
private:
	float palmscale;
	RotateControl rotation;
public:
	Palm() { this->palmscale = 1;}
	~Palm() {}

	float Getplamscale() { return this->palmscale; }
	RotateControl GetRotate() { return this->rotation; }

	void Setpalmscale(float scale) { this->palmscale = scale; }
	void SetRotate(RotateControl R) { this->rotation = R; }
	void SetRotate(float x, float y, float z) { this->rotation.SetRotateXYZ(x, y, z); }
};

void func(double *p, double *x, int m, int n, void *data);


class HandControl
{
public:
	Finger *fingers;
	Palm palm;
	Pose palm_position;

	int paramsSize;
	float* paramsOfhand;

	bool ParamsChangeStop;

	struct CostFunction
	{
		float weight1 = 1,weight2 = 0.5;

		double steparry[7] = {0.001, 0.005,0.01,0.03,0.05,0.08,0.1 };
		float step = 0.01;


		float trans_step = 0.5;
		float position_h = 1;
		float scale_h = 0.1;
		float trans_h = 1;
		float T = 0;
		float costfunction = 10000;
		float gradient[24] = { 0 };
		int rendered_silhouetteSize;
		int groundtruth_silhouetteSize = 0;

		vector<float> renderToGroundtruthMinDistance;
		vector<float> groundtruthTorenderMinDistance;
		float distance_renderTogroundtruth;
		float distance_groundtruthTorender;
		/*cv::Mat groundtruthmat = cv::Mat::zeros(240, 320, CV_16UC1);
		cv::Mat groundtruthBinaryMat = cv::Mat::zeros(240, 320, CV_64F);*/

		cv::Mat groundtruthmat = cv::Mat::zeros(424, 512, CV_16UC1);
		cv::Mat groundtruthBinaryMat = cv::Mat::zeros(424, 512, CV_64F);

		vector<Point> groundtruth_silhouette;
		
		void ComputeGroundtruth_silhouette()
		{
			groundtruth_silhouette.clear();
			cv::Mat groundtruth_Silhouette_copy = cv::Mat::zeros(groundtruthmat.rows, groundtruthmat.cols, CV_8UC1);
			for (int i = 0; i < groundtruthmat.rows; i++)
			{
				for (int j = 0; j < groundtruthmat.cols; j++) {
					if (groundtruthmat.at<ushort>(i, j) != 0)
					{
						groundtruth_Silhouette_copy.at<uchar>(i, j) = 255;
						groundtruthBinaryMat.at<double>(i,j) = 1;
					}
				}
			}

			threshold(groundtruth_Silhouette_copy, groundtruth_Silhouette_copy, 10, 255.0, CV_THRESH_BINARY);

			vector<vector<Point>>contours_groundtruth_Silhouette;
			vector<Vec4i>hierarchy_groundtruth_Silhouette;

			findContours(groundtruth_Silhouette_copy, contours_groundtruth_Silhouette, hierarchy_groundtruth_Silhouette, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

			int index_groundthuth = -1;
			double area, maxArea(0);
			for (int i = 0; i < contours_groundtruth_Silhouette.size(); i++)
			{
				area = contourArea(Mat(contours_groundtruth_Silhouette[i]));
				if (area > maxArea) {
					maxArea = area;
					index_groundthuth = i;
				}
			}
			this->groundtruth_silhouetteSize = contours_groundtruth_Silhouette[index_groundthuth].size();
			for (int i = 0; i < contours_groundtruth_Silhouette[index_groundthuth].size(); i++)
			{
				groundtruth_silhouette.push_back(contours_groundtruth_Silhouette[index_groundthuth][i]);
			}

		}

		//float ComputeSilhouetteDifference(cv::Mat rendered_Silhouette,cv::Mat useless)
		//{

		//	cv::Mat rendered_Silhouette_copy = cv::Mat::zeros(rendered_Silhouette.rows, rendered_Silhouette.cols, CV_8UC1);
		//	for (int i = 0; i < rendered_Silhouette.rows; i++)
		//	{
		//		for (int j = 0; j < rendered_Silhouette.cols; j++) {
		//			if (rendered_Silhouette.at<ushort>(i, j) != 0)
		//				rendered_Silhouette_copy.at<uchar>(i, j) = 255;
		//		}
		//	}

		//	
		//	threshold(rendered_Silhouette_copy, rendered_Silhouette_copy, 10, 255.0, CV_THRESH_BINARY);          //threshold 的输入图像应该是单通道8位或者32位的，所以要先把16位的图像转换成8位图像
		//	//cv::imshow("rendered_Silhouette", rendered_Silhouette_copy);

		//	vector<vector<Point>>contours_rendered_Silhouette;
		//	vector<Vec4i>hierarchy_rendered_Silhouette;
		//	findContours(rendered_Silhouette_copy, contours_rendered_Silhouette, hierarchy_rendered_Silhouette, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
		//	

		//	int index_render = -1;
		//	double area, maxArea(0);
		//	for (int i = 0; i < contours_rendered_Silhouette.size(); i++)
		//	{
		//		area = contourArea(Mat(contours_rendered_Silhouette[i]));
		//		if(area > maxArea){
		//			maxArea = area;
		//			index_render = i;
		//		}
		//	}
		//	
		//	/*cv::Ptr <cv::HausdorffDistanceExtractor> hausdorff_ptr = cv::createHausdorffDistanceExtractor();
		//	float distance = hausdorff_ptr->computeDistance(contours_rendered_Silhouette[index_render], contours_groundtruth_Silhouette[index_groundthuth]);*/

		//	rendered_silhouetteSize = contours_rendered_Silhouette[index_render].size();
		//	renderToGroundtruthMinDistance.clear();
		//	groundtruthTorenderMinDistance.clear();

		//	Mat sourse = cv::Mat(this->groundtruth_silhouette).reshape(1);
		//	Mat target = cv::Mat(contours_rendered_Silhouette[index_render]).reshape(1);
		//	target.convertTo(target, CV_32F);       //这里必须转换成CV_32F或者CV_8U
		//	sourse.convertTo(sourse, CV_32F);
		//	flann::KDTreeIndexParams indexParams(2);
		//	flann::Index kdtree(sourse, indexParams);

		//	int k = 1;
		//	cv::Mat indices(target.rows, k, CV_32F);
		//	cv::Mat dists(target.rows, k, CV_32F);         //搜索到的最近邻的距离

		//	kdtree.knnSearch(target, indices, dists,k, cv::flann::SearchParams(32));

		//	this->distance_renderTogroundtruth = 0;

		//	for (int i = 0; i < dists.rows; i++)
		//	{
		//		this->distance_renderTogroundtruth = this->distance_renderTogroundtruth + dists.at<float>(i, 0);
		//		renderToGroundtruthMinDistance.push_back(sqrt(dists.at<float>(i, 0)));
		//	}
		//	this->distance_renderTogroundtruth = this->distance_renderTogroundtruth / dists.rows;



		//	flann::KDTreeIndexParams indexParams2(2);
		//	flann::Index kdtree2(target, indexParams2);

		//	int k2 = 1;
		//	cv::Mat indices2(sourse.rows, k2, CV_32F);
		//	cv::Mat dists2(sourse.rows, k2, CV_32F);         //搜索到的最近邻的距离

		//	kdtree2.knnSearch(sourse, indices2, dists2, k2, cv::flann::SearchParams(32));

		//	distance_groundtruthTorender = 0;

		//	for (int i = 0; i < dists2.rows; i++)
		//	{
		//		distance_groundtruthTorender = distance_groundtruthTorender + dists2.at<float>(i, 0);
		//		groundtruthTorenderMinDistance.push_back(sqrt(dists2.at<float>(i, 0)));
		//	}
		//	
		//	distance_groundtruthTorender = distance_groundtruthTorender / dists2.rows;
		//	/*cv::imshow("rendered_Silhouette", rendered_Silhouette_copy);
		//	cv::imshow("groundtruth_Silhouette", groundtruth_Silhouette_copy);*/

		//	return 1*distance_groundtruthTorender+1* this->distance_renderTogroundtruth;

		//}

		float ComputeMatDifference(cv::Mat rendered_Silhouette)
		{
			cv::Mat rendered_BinaryMat = cv::Mat::zeros(rendered_Silhouette.rows, rendered_Silhouette.cols, CV_64F);
			for (int i = 0; i < rendered_Silhouette.rows; i++)
			{
				for (int j = 0; j < rendered_Silhouette.cols; j++) {
					if (rendered_Silhouette.at<ushort>(i, j) != 0)
						rendered_BinaryMat.at<double>(i, j) = 1;
				}
			}


			cv::Mat differenceMat = groundtruthBinaryMat - rendered_BinaryMat;
			float difference = countNonZero(differenceMat);
			return difference;
		}
		//float ComputeCostfunction2(cv::Mat rendered_Silhouette, cv::Mat groundtruth_Silhouette)
		//{
		//	
		//	this->costfunction = weight1*_cloudpoint.SumDistance / _cloudpoint.num_cloudpoint + weight2*ComputeSilhouetteDifference(rendered_Silhouette, groundtruth_Silhouette);
		//	//cout << "cloud :" << _cloudpoint.SumDistance / _cloudpoint.num_cloudpoint << "   silhuuette : " << ComputeSilhouetteDifference(rendered_Silhouette, groundtruth_Silhouette) << endl;
		//	return this->costfunction;
		//}

		float ComputeCostfunction(cv::Mat rendered_Silhouette, cv::Mat groundtruth_Silhouette)
		{

			this->costfunction = weight1*_cloudpoint.SumDistance / _cloudpoint.num_cloudpoint + weight2*ComputeMatDifference(rendered_Silhouette);
			//cout << "cloud :" << _cloudpoint.SumDistance / _cloudpoint.num_cloudpoint << "   silhuuette : " << ComputeMatDifference(rendered_Silhouette) << endl;
			//cout << "costfunction is :" << this->costfunction << endl;
			return this->costfunction;
		}

	}_costfunction;
	
	HandControl() { 
		this->fingers = new Finger[5]; 
		this->palm_position.x = 0;
		this->palm_position.y = 0;
		this->paramsSize = 24;
		this->palm_position.z = -800; 
		this->paramsOfhand = new float[this->paramsSize];
		this->ParamsToMat();
		this->ParamsChangeStop = false;
	}
	~HandControl() { delete this->fingers; }
	void SetPlam_Position(float x, float y, float z)
	{
		Pose p;
		p.x = x; p.y = y; p.z = z;
		this->palm_position = p;
	}
	void SetPlam(Palm p) { this->palm = p; }
	void SetFingers(Finger* f) { this->fingers = f;}
	void SetParamOfHand(float *p) { this->paramsOfhand = p; }
	//吧参数传递给手摸的函数,并使得手摸进行相应变化
	void ControlHand() {
		model->set_global_position(palm_position);
		model->set_global_position_center(palm_position);

		model->set_joint_scale(palm.Getplamscale(), 0);
		model->set_joint_scale(palm.Getplamscale(), 21);  //这里让手腕和手掌同等缩小放大，防止看起来怪异
		model->set_one_rotation(Pose(palm.GetRotate().GetRotateX(), palm.GetRotate().GetRotateY(), palm.GetRotate().GetRotateZ()),0);

		for (int i = 0; i < 5; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				model->set_joint_scale(fingers[i].Getfingerscale(), i*4+j+1);
				model->set_one_rotation(Pose(fingers[i].GetRotete()[j].GetRotateX(), fingers[i].GetRotete()[j].GetRotateY(), fingers[i].GetRotete()[j].GetRotateZ()), i*4 + j + 1);
				model->set_one_trans(fingers[i].GetTrans()[j].GettransX(), i*4 + j + 1);
			}
		}

		model->forward_kinematic();
		model->compute_mesh();
	}

	//随机给定参数
	void RandomGenerateParams()
	{
		Pose upper, lower;
		Random random;
		//随机全局位置
		float random_palm_positonX, random_palm_positonY, random_palm_positonZ;
		random_palm_positonX = random.Next(-200, 0);
		random_palm_positonY = random.Next(-200, 0);
		random_palm_positonZ = random.Next(-800, -400);
		this->SetPlam_Position(random_palm_positonX, random_palm_positonY, random_palm_positonZ);

		//随机手掌参数
		Palm random_palm;
		float random_palmscale;
		random_palmscale = 1+ random.NextDouble();   //[0,2]
		random_palm.Setpalmscale(random_palmscale);
		RotateControl random_palmRotation;
		upper = model->get_upper_of_angle(0);
		lower = model->get_lower_of_angle(0);
		random_palmRotation.SetRotateX(random.Next(lower.x, upper.x));
		random_palmRotation.SetRotateY(random.Next(lower.y, upper.y));
		random_palmRotation.SetRotateZ(random.Next(lower.z, upper.z));
		random_palm.SetRotate(random_palmRotation);
		this->SetPlam(random_palm);

		//随机手指参数
		Finger *random_fingers = new Finger[5];
		for (int i = 0; i < 5; i++)
		{
			float random_fingerscale;
			random_fingerscale = 1+ 0.5*random.NextDouble();
			random_fingers[i].Setfingerscale(random_fingerscale);
			for (int j = 0; j < 4; j++)
			{
				RotateControl random_fingerRotation;
				upper = model->get_upper_of_angle(i * 4 + j + 1);
				lower = model->get_lower_of_angle(i * 4 + j + 1);
				random_fingerRotation.SetRotateX(random.Next(lower.x, upper.x));
				random_fingerRotation.SetRotateY(random.Next(lower.y, upper.y));
				random_fingerRotation.SetRotateZ(random.Next(lower.z, upper.z));

				TransControl random_fingerTrans;
				if (j == 3)
				{
					random_fingerTrans.SettransX(random.Next(0, 0));       //这里发现指尖点对mesh顶点是没有权重的，因此，手指的第三个关节长度是没办法变化的。
				}
				else
				{
					random_fingerTrans.SettransX(random.Next(10, 30));     //
				}

				random_fingers[i].SetRotate(random_fingerRotation, j);
				random_fingers[i].SetTrans(random_fingerTrans, j);
			}
		}
		this->SetFingers(random_fingers);


		this->ParamsToMat();

	}
	void RandomScaleAndTransParams()
	{
		Random random;
		//随机全局位置
		/*float random_palm_positonX, random_palm_positonY, random_palm_positonZ;
		random_palm_positonX = random.Next(-50, 50);
		random_palm_positonY = random.Next(-50, 50);
		random_palm_positonZ = random.Next(-550, -500);
		this->SetPlam_Position(random_palm_positonX, random_palm_positonY, random_palm_positonZ);
*/
		//随机手掌参数
		Palm random_palm;
		float random_palmscale;
		random_palmscale = 1 + random.NextDouble();   //[1,2]
		random_palm.Setpalmscale(random_palmscale);
		RotateControl random_palmRotation;
		random_palmRotation.SetRotateX(this->palm.GetRotate().GetRotateX());
		random_palmRotation.SetRotateY(this->palm.GetRotate().GetRotateY());
		random_palmRotation.SetRotateZ(this->palm.GetRotate().GetRotateZ());
		random_palm.SetRotate(random_palmRotation);
		this->SetPlam(random_palm);

		//随机手指参数
		Finger *random_fingers = new Finger[5];
		for (int i = 0; i < 5; i++)
		{
			float random_fingerscale;
			random_fingerscale = 1 + 0.5*random.NextDouble();
			random_fingers[i].Setfingerscale(random_fingerscale);
			for (int j = 0; j < 4; j++)
			{
				RotateControl random_fingerRotation;
				random_fingerRotation.SetRotateX(this->fingers[i].GetRotete()[j].GetRotateX());
				random_fingerRotation.SetRotateY(this->fingers[i].GetRotete()[j].GetRotateY());
				random_fingerRotation.SetRotateZ(this->fingers[i].GetRotete()[j].GetRotateZ());

				TransControl random_fingerTrans;
				if (j == 3)
				{
					random_fingerTrans.SettransX(random.Next(0, 0));       //这里发现指尖点对mesh顶点是没有权重的，因此，手指的第三个关节长度是没办法通过这个关节变化的，只能通过其他关节变化。
				}
				else
				{
					random_fingerTrans.SettransX(random.Next(10, 30));     //
				}

				random_fingers[i].SetRotate(random_fingerRotation, j);
				random_fingers[i].SetTrans(random_fingerTrans, j);
			}
			
		}
		this->SetFingers(random_fingers);


		this->ParamsToMat();
		
	}
	void RandomScaleAndTransParams_justlittle()
	{
		Random random;
		//随机全局位置
		/*float random_palm_positonX, random_palm_positonY, random_palm_positonZ;
		random_palm_positonX = random.Next(-50, 50);
		random_palm_positonY = random.Next(-50, 50);
		random_palm_positonZ = random.Next(-550, -500);
		this->SetPlam_Position(random_palm_positonX, random_palm_positonY, random_palm_positonZ);
		*/
		//随机手掌参数
		
		this->palm.Setpalmscale(this->palm.Getplamscale()+(-0.5 + random.NextDouble()));



		//随机手指参数
		
		for (int i = 0; i < 5; i++)
		{
			this->fingers[i].Setfingerscale(this->fingers[i].Getfingerscale() + (-0.5 + random.NextDouble()));

			for (int j = 0; j < 3; j++)
			{
				this->fingers[i].SetTrans(this->fingers[i].GetTrans()[j].GettransX() + random.Next(-5, 5), j);
			}
		}
		

		this->ParamsToMat();

	}
	void RandomScale()
	{
		Random random;
		//随机全局位置
		/*float random_palm_positonX, random_palm_positonY, random_palm_positonZ;
		random_palm_positonX = random.Next(-50, 50);
		random_palm_positonY = random.Next(-50, 50);
		random_palm_positonZ = random.Next(-550, -500);
		this->SetPlam_Position(random_palm_positonX, random_palm_positonY, random_palm_positonZ);*/
		
		//随机手掌参数
		this->palm.Setpalmscale(1 + random.NextDouble());

		////随机手指参数
		//for (int i = 0; i < 5; i++)
		//{
		//	
		//	this->fingers[i].Setfingerscale(0.5 + random.NextDouble());
		//}
		//


		this->ParamsToMat();

	}
	//根据梯度，或者牛顿高斯法改变参数，只改变大小，全局位置，和手指长度（sacle, position ,trans)

	void ParamsChangeUseLevmar()
	{
		int ret;
		const int m = 1;
		int n = 0*_cloudpoint.num_cloudpoint*3 + _costfunction.groundtruthmat.rows * _costfunction.groundtruthmat.cols;
		//int n = model->num_vertices_*3;
		//int n = _cloudpoint.num_cloudpoint * 3;
		double *x,*p;
		x = new double[n];
		p = new double[m];
		double opts[LM_OPTS_SZ], info[LM_INFO_SZ];

		opts[0] = LM_INIT_MU; opts[1] = 1E-15; opts[2] = 1E-15; opts[3] = 1E-20;
		opts[4] = 0.1;

		//for (int i = 0; i < _cloudpoint.num_cloudpoint*3; i++)
		//{
		//	x[i] = _cloudpoint.cloudpoint[i];
		//}

		for (int i = 0; i < _costfunction.groundtruthBinaryMat.rows; i++)
		{
			for (int j = 0; j < _costfunction.groundtruthBinaryMat.cols; j++)
			{
				x[0*_cloudpoint.num_cloudpoint * 3 + _costfunction.groundtruthBinaryMat.cols*i + j] = _costfunction.groundtruthBinaryMat.at<double>(i, j);
			}
		}

		this->RandomScale();
		this->ParamsToMat();
		this->ControlHand();


		cout << "this original params is ..... " << endl;
		//for (int i = 0; i < m; i++)
		//{
		//	p[i] = this->paramsOfhand[i];
		//	cout << this->paramsOfhand[i] << endl;
		//}
		p[0] = this->palm.Getplamscale();
		cout << p[0];
		cout << endl;
		cout << endl;


		ret = dlevmar_dif(func, p, x, m, n, 1000,opts, info, NULL, NULL,NULL);

		printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);

		cout << "the optimizing params is ....." << endl;
		//for (int i = 0; i < m; i++)
		//{
		//	cout << p[i] << endl;
		//}
		cout << p[0];
		cout << endl;
		cout << endl;
		
	}
	void ParamsChangeUseGradient()
	{


		//this->palm_position.x = this->palm_position.x - _costfunction.step * _costfunction.gradient[0];
		//this->palm_position.y = this->palm_position.y - _costfunction.step * _costfunction.gradient[1];
		//this->palm_position.z = this->palm_position.z - _costfunction.step * _costfunction.gradient[2];

		//this->palm.Setpalmscale(this->palm.Getplamscale() - _costfunction.step * _costfunction.gradient[3]);
		//
  //      
		//for (int i = 0; i < 5; i++)
		//{
		//	this->fingers[i].Setfingerscale(this->fingers[i].Getfingerscale() - _costfunction.step * _costfunction.gradient[3 + i * 4 + 1]);
		//	this->fingers[i].SetTrans(this->fingers[i].GetTrans()[0].GettransX() - _costfunction.trans_step * _costfunction.gradient[3 + i * 4 + 2], 0);
		//	this->fingers[i].SetTrans(this->fingers[i].GetTrans()[1].GettransX() - _costfunction.trans_step * _costfunction.gradient[3 + i * 4 + 3], 1);
		//	this->fingers[i].SetTrans(this->fingers[i].GetTrans()[2].GettransX() - _costfunction.trans_step * _costfunction.gradient[3 + i * 4 + 4], 2);
		//}


		//this->LimitedPalmScale();
		//this->LimitedfingerScale();
		//this->LimitedfingerLength();






		//先把参数存起来
		this->ParamsToMat();


		float costfunction, mincostfunction = FLT_MAX;
		int minindex;

		for (int i = 0; i < 7; i++)
		{
			this->ParamsChange(_costfunction.steparry[i]);


			costfunction = this->Function_Out();

			if (costfunction < mincostfunction)
			{
				mincostfunction = costfunction;
				minindex = i;
			}

			this->MatToParams();
		}

		//cout << "the step is :" << _costfunction.steparry[minindex] << endl;
		//cout << "the  gradient is :  " << endl;
		//for (int i = 0; i < 24; i++)
		//{
		//	if ((i == 3) || (i == 4) || (i == 8) || (i == 12) || (i == 16) || (i == 20))
		//	{
		//		cout << _costfunction.steparry[minindex] *_costfunction.gradient[i] << endl;
		//	}
		//	else
		//	{
		//		cout << _costfunction.steparry[minindex] * 50 * _costfunction.gradient[i] << endl;
		//	}
		//}
		//cout << "gradient end~~~~" << endl;

		this->ParamsChange(_costfunction.steparry[minindex]);

		this->ParamsChangeStop = true;
		float JudgeGradient[24];
		for (int i = 0; i < 24; i++)
		{
			if ((i == 3) || (i == 4) || (i == 8) || (i == 12) || (i == 16) || (i == 20))
			{
				JudgeGradient[i] = _costfunction.steparry[minindex] * _costfunction.gradient[i];
			}
			else
			{
				JudgeGradient[i] = _costfunction.steparry[minindex] * 50 * _costfunction.gradient[i];
			}

			if (abs(JudgeGradient[i]) > 0.35)
			{
				this->ParamsChangeStop = false;
			}
		}


	}


	//this function only adjust the palmScale
	void ParamsChangeUsingGN()
	{
		Mat outputs(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);

		for (int i = 0; i < _costfunction.groundtruthBinaryMat.rows; i++)
		{
			for (int j = 0; j < _costfunction.groundtruthBinaryMat.cols; j++)
			{
				outputs.at<float>(i* _costfunction.groundtruthBinaryMat.cols + j, 0) = _costfunction.groundtruthBinaryMat.at<double>(i, j);
			}
		}

		
		float Params = 1.5;

		for (int itr = 0; itr < 100; itr++)
		{
			Mat residual(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);
			Mat Fun_outputs(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);
			Mat Fun_outputs_delta(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);
			Mat Jacobian(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);


			//先算Fun_outputs
			this->palm.Setpalmscale(Params);
			this->ControlHand();
			cv::Mat generated_mat_outputs = cv::Mat::zeros(240, 320, CV_16UC1);
			projection->compute_current_orientation(model);
			projection->project_3d_to_2d_(model, generated_mat_outputs);
			cv::Mat generated_mat_outputs_Binary = cv::Mat::zeros(240, 320, CV_64F);
			for (int i = 0; i < generated_mat_outputs.rows; i++)
			{
				for (int j = 0; j < generated_mat_outputs.cols; j++)
				{
					if (generated_mat_outputs.at<ushort>(i, j) != 0)
					{
						generated_mat_outputs_Binary.at<double>(i, j) = 1;
					}
				}
			}
			
			for (int i = 0; i <generated_mat_outputs_Binary.rows; i++)
			{
				for (int j = 0; j < generated_mat_outputs_Binary.cols; j++)
				{
					Fun_outputs.at<float>(i* generated_mat_outputs_Binary.cols + j, 0) = generated_mat_outputs_Binary.at<double>(i, j);
				}
			}
			//cout << countNonZero(Fun_outputs) << endl;

			//然后计算residual
			for (int i = 0; i < _costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows; i++)
			{
				residual.at<float>(i, 0) = Fun_outputs.at<float>(i, 0) - outputs.at<float>(i, 0);
				
			}
			
			//cout << countNonZero(residual) << endl;
			//然后算 Params + delta之后的Fun_outputs
			this->palm.Setpalmscale(Params + 0.01);
			this->ControlHand();
			cv::Mat generated_mat_outputs_delta = cv::Mat::zeros(240, 320, CV_16UC1);
			projection->compute_current_orientation(model);
			projection->project_3d_to_2d_(model, generated_mat_outputs_delta);
			cv::Mat generated_mat_outputs_delta_Binary = cv::Mat::zeros(240, 320, CV_64F);
			for (int i = 0; i < generated_mat_outputs_delta.rows; i++)
			{
				for (int j = 0; j < generated_mat_outputs_delta.cols; j++)
				{
					if (generated_mat_outputs_delta.at<ushort>(i, j) != 0)
					{
						generated_mat_outputs_delta_Binary.at<double>(i, j) = 1;
					}
				}
			}
			for (int i = 0; i <generated_mat_outputs_delta_Binary.rows; i++)
			{
				for (int j = 0; j < generated_mat_outputs_delta_Binary.cols; j++)
				{
					Fun_outputs_delta.at<float>(i* generated_mat_outputs_Binary.cols + j, 0) = generated_mat_outputs_delta_Binary.at<double>(i, j);
				}
			}
			//cout << countNonZero(Fun_outputs_delta) << endl;

			//然后计算Jacobian
			for (int i = 0; i < _costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows; i++)
			{
				Jacobian.at<float>(i, 0) = (Fun_outputs_delta.at<float>(i, 0) - Fun_outputs.at<float>(i, 0)) / 0.01;
			}

			//这一步是成功的关键！！！！
			//由于是采用前向差分计算的jacobian行列式，所以这里必须更新residual =（f(x+delta) + f(x) - 2*I)，
			//若继续使用residual = (f(x) - I)会造成Jacobian.t()*residual是全零的情况，具体原因我还没分析出来~

			//tomorrow work:  如果有多个自变量，比如24个params那么，要分别算每一个的J和每一个的residual,不能共用一个residual,然后组合成一个列向量,
			//或者其他的组合，明天在进行相关的尝试
			for (int i = 0; i < _costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows; i++)
			{
				residual.at<float>(i, 0) = residual.at<float>(i, 0) + Fun_outputs_delta.at<float>(i,0) - outputs.at<float>(i, 0);

			}

			//cout << countNonZero(Jacobian) << endl;
			//然后计算调整的delta

			//cout << Jacobian.t()*Jacobian << endl;
			//cout << ((Jacobian.t()*Jacobian)).inv() << endl;
			//cout << Jacobian.t() << endl;
			cout << "the " << itr << "th result is: " << endl;
			cout << Jacobian.t()*residual << endl;
			Mat delta = ((Jacobian.t()*Jacobian)).inv() * Jacobian.t()*residual;
			
			//Mat delta = Jacobian.t()*residual;

			cout << "delta is :"<<delta << endl;
			//然后根据delta改变Params
			Params = Params - delta.at<float>(0,0);

			cout << "params is:  "<<Params << endl;
			cout << endl << endl;

			if (abs(delta.at<float>(0, 0)) < 0.001)
			{
				break;
			}

		}

	}


	void ALLParamsChangeUsingGN()
	{
		Mat outputs(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);

		for (int i = 0; i < _costfunction.groundtruthBinaryMat.rows; i++)
		{
			for (int j = 0; j < _costfunction.groundtruthBinaryMat.cols; j++)
			{
				outputs.at<float>(i* _costfunction.groundtruthBinaryMat.cols + j, 0) = _costfunction.groundtruthBinaryMat.at<double>(i, j);
			}
		}

		this->RandomScaleAndTransParams_justlittle();

		cout << "the original params is :  " << endl;
		for (int i = 0; i < this->paramsSize; i++)
		{
			cout << this->paramsOfhand[i] << endl;
		}
		cout << endl;
		//this->RandomScale();

		for (int itr = 0; itr < 100; itr++)
		{
			//整体的雅各比行列式
			Mat Jacobian(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, this->paramsSize, CV_32F);
			//整体的梯度
			Mat Gradient(this->paramsSize,1, CV_32F);
			//某一个变量的雅各比行列式
			Mat OneDimension_Jacobian(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);
			//某一个参数改变对应的输出，用于差分求导数
			Mat Fun_outputs(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);
			Mat Fun_outputs_delta(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);
			//residual
			Mat residual(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);

			Mat OneDimensionDelta = Mat::zeros(this->paramsSize,1, CV_32F);


			this->Function_Outputs(Fun_outputs);


			for (int paramsInt = 0; paramsInt <this->paramsSize; paramsInt++)
			{
				if ((paramsInt == 0) || (paramsInt == 4) || (paramsInt == 8) || (paramsInt == 12) || (paramsInt == 16) || (paramsInt == 20)) //如果参数代表scale,那么使用差分计算偏导的时候，使用delta = 0.01
				{
					this->paramsOfhand[paramsInt] = this->paramsOfhand[paramsInt] + 0.01;
				}
				else  //如果代表长度，那么使用差分法计算偏导的时候，delta选择0.1
				{
					this->paramsOfhand[paramsInt] = this->paramsOfhand[paramsInt] + 0.5;
				}


				this->Function_Outputs(Fun_outputs_delta);


				//下面是把参数还原，因为上面为了使用差分法计算偏导，加了一个delta
				if ((paramsInt == 0) || (paramsInt == 4) || (paramsInt == 8) || (paramsInt == 12) || (paramsInt == 16) || (paramsInt == 20)) //如果参数代表scale,那么使用差分计算偏导的时候，使用delta = 0.01
				{
					this->paramsOfhand[paramsInt] = this->paramsOfhand[paramsInt] - 0.01;
				}
				else  //如果代表长度，那么使用差分法计算偏导的时候，delta选择0.1
				{
					this->paramsOfhand[paramsInt] = this->paramsOfhand[paramsInt] - 0.5;
				}

				this->ComputeResidual(outputs, Fun_outputs, Fun_outputs_delta, residual);
				this->ComputeOneDimensionJacobian(paramsInt, Fun_outputs, Fun_outputs_delta, OneDimension_Jacobian);


				Mat D = (2*(OneDimension_Jacobian.t()*OneDimension_Jacobian)).inv() * OneDimension_Jacobian.t()*residual;
				//cout << D.at<float>(0, 0) << endl;
				OneDimensionDelta.at<float>(paramsInt, 0) = D.at<float>(0, 0);
				//cout << OneDimensionDelta.at<float>(paramsInt, 0) << endl;
				for (int i = 0; i < _costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows; i++)
				{
					Jacobian.at<float>(i, paramsInt) = OneDimension_Jacobian.at<float>(i, 0);
				}

				Mat OneDimensionGradient = OneDimension_Jacobian.t()*residual;
				//cout << OneDimensionGradient << endl;
				Gradient.at<float>(paramsInt, 0) = OneDimensionGradient.at<float>(0, 0);

			}


			Mat delta = (2*(Jacobian.t()*Jacobian)).inv() * Gradient;


			cout << "the " << itr << " th iteration :   the Params Changed is :  " << endl;
			for (int i = 0; i < this->paramsSize; i++)
			{
				//this->paramsOfhand[i] = this->paramsOfhand[i] - delta.at<float>(i, 0);
				this->paramsOfhand[i] = this->paramsOfhand[i] - OneDimensionDelta.at<float>(i,0);
				cout << this->paramsOfhand[i] << endl;
			}

			this->MatToParams();
			this->LimitedPalmScale();
			this->LimitedfingerScale();
			this->LimitedfingerLength();
			this->ParamsToMat();

			cout << endl;
			
		}
	}

	void Function_Outputs(Mat &func_outputs)
	{
		this->MatToParams();
		this->ControlHand();

		//cv::Mat generated_mat_outputs = cv::Mat::zeros(240, 320, CV_16UC1);

		cv::Mat generated_mat_outputs = cv::Mat::zeros(424, 512, CV_16UC1);
		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_(model, generated_mat_outputs);
		//cv::Mat generated_mat_outputs_Binary = cv::Mat::zeros(240, 320, CV_64F);
		cv::Mat generated_mat_outputs_Binary = cv::Mat::zeros(424, 512, CV_64F);
		for (int i = 0; i < generated_mat_outputs.rows; i++)
		{
			for (int j = 0; j < generated_mat_outputs.cols; j++)
			{
				if (generated_mat_outputs.at<ushort>(i, j) != 0)
				{
					generated_mat_outputs_Binary.at<double>(i, j) = 1;
				}
			}
		}
		for (int i = 0; i <generated_mat_outputs_Binary.rows; i++)
		{
			for (int j = 0; j < generated_mat_outputs_Binary.cols; j++)
			{
				func_outputs.at<float>(i* generated_mat_outputs_Binary.cols + j, 0) = generated_mat_outputs_Binary.at<double>(i, j);
			}
		}

	}

	void ComputeResidual(const Mat &outputs,const Mat &func_outputs,const Mat &func_outputs_delta,Mat &residual )
	{
		for (int i = 0; i < _costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows; i++)
		{
			residual.at<float>(i, 0) = func_outputs.at<float>(i,0) + func_outputs_delta.at<float>(i, 0) - 2*outputs.at<float>(i, 0);
		}
	}

	void ComputeOneDimensionJacobian(int index, const Mat &func_outputs, const Mat &func_outputs_delta, Mat &OneDimJacobian)
	{
		if ((index == 0) || (index == 4) || (index == 8) || (index == 12) || (index == 16) || (index == 20)) //如果参数代表scale,那么计算差分计算偏导的时候，使用delta = 0.01
		{
			for (int i = 0; i < _costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows; i++)
			{
				OneDimJacobian.at<float>(i, 0) = (func_outputs_delta.at<float>(i, 0) - func_outputs.at<float>(i, 0)) / 0.01;
			}
		}
		else
		{
			for (int i = 0; i < _costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows; i++)
			{
				OneDimJacobian.at<float>(i, 0) = (func_outputs_delta.at<float>(i, 0) - func_outputs.at<float>(i, 0)) / 0.5;
			}
		}
	}
	//void ParamsChangeUseGaussNewTon()
	//{
	//	//这里注意 创建Mat的时候是CV_32F， 则.at<float>这里用float，如果是CV_64F ，则必须用double，不然会出错
	//	Mat r_cloudPoint(_cloudpoint.num_cloudpoint, 1, CV_32F); // residual matrix  
	//	Mat r_silhouette(_costfunction.groundtruth_silhouetteSize,1,CV_32F);
	//	Mat r_3(1, 1, CV_32F);
	//	this->ComputeResidual(r_cloudPoint, r_silhouette,r_3);


	//	Mat Jf_cloudpoint(_cloudpoint.num_cloudpoint, this->paramsSize, CV_32F); // Jacobian of Func()  
	//	Mat Jf_silhouette(_costfunction.groundtruth_silhouetteSize, this->paramsSize, CV_32F); // Jacobian of Func()  
	//	Mat Jf_3(1, this->paramsSize, CV_32F);// Jacobian of Func()  
	//	Mat E = cv::Mat::eye(this->paramsSize, this->paramsSize, CV_32F);

	//	for (int i = 0; i < Jf_cloudpoint.cols; i++)
	//	{
	//		Mat r_cloud_1(_cloudpoint.num_cloudpoint, 1, CV_32F);
	//		Mat r_cloud_2(_cloudpoint.num_cloudpoint, 1, CV_32F);
	//		this->ComputeCloudPointJacobian(r_cloud_1, r_cloud_2, i);

	//		for (int j = 0; j < Jf_cloudpoint.rows; j++)
	//		{
	//			//cout << r_cloud_2.at<float>(j, 0) << endl;
	//			Jf_cloudpoint.at<float>(j, i) = r_cloud_2.at<float>(j, 0) / _cloudpoint.num_cloudpoint;
	//		}

	//	}

	//	for (int i = 0; i < Jf_silhouette.cols; i++)
	//	{
	//		Mat r_silh_1(_costfunction.groundtruth_silhouetteSize, 1, CV_32F);
	//		Mat r_silh_2(_costfunction.groundtruth_silhouetteSize, 1, CV_32F);
	//		this->ComputeSilhouetteJacobian(r_silh_1, r_silh_2, i);

	//		for (int j = 0; j < Jf_silhouette.rows; j++)
	//		{
	//			Jf_silhouette.at<float>(j, i) = r_silh_2.at<float>(j, 0) /_costfunction.groundtruth_silhouetteSize;
	//		}

	//	}

	//	for (int i = 0; i < Jf_3.cols; i++)
	//	{
	//		Jf_3.at<float>(0, i) = this->ComputeJ_3(i);
	//	}

	//	//Mat delta = ((Jf_cloudpoint.t()*Jf_cloudpoint) + (Jf_silhouette.t()*Jf_silhouette) + (Jf_3.t()*Jf_3)).inv() * ((Jf_cloudpoint.t()*r_cloudPoint) + (Jf_silhouette.t()*r_silhouette) + (Jf_3.t()*r_3));
	//	Mat delta = ((Jf_cloudpoint.t()*Jf_cloudpoint) + (Jf_silhouette.t()*Jf_silhouette)).inv() * ((Jf_cloudpoint.t()*r_cloudPoint) + (Jf_silhouette.t()*r_silhouette));
	//	//Mat delta = (Jf_cloudpoint.t()*Jf_cloudpoint).inv()*(Jf_cloudpoint.t()*r_cloudPoint);
	//	//Mat delta = ((Jf_3.t()*Jf_3)).inv() * ((Jf_3.t()*r_3));
	//	this->paramsOfhand = this->paramsOfhand + delta;
	//	this->MatToParams();

	//	this->LimitedPalmScale();
	//	this->LimitedfingerScale();
	//	this->LimitedfingerLength();
	//	cout << endl;
	//	cout << "PARAMS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
	//	cout << "the paras Mat is :  " << paramsOfhand << endl;
	//	cout << "end PARAMS ." << endl;
	//	cout << endl;
	//	this->ControlHand();
	//}


	//存储和读出参数
	void SaveParams(char* filename){
		ofstream f;
		f.open(filename, ios::out);
		f << this->palm_position.x << " " << this->palm_position.y <<" "<<this->palm_position.z<< endl;

		f << this->palm.Getplamscale() << " " << this->palm.GetRotate().GetRotateX() << " " << this->palm.GetRotate().GetRotateY() << " " << this->palm.GetRotate().GetRotateZ() << endl;
		for (int i = 0; i < 5; i++) {
			f << this->fingers[i].Getfingerscale() << endl;
			for (int j = 0; j < 4; j++)
			{
				f << this->fingers[i].GetRotete()[j].GetRotateX() << " " << this->fingers[i].GetRotete()[j].GetRotateY() << " " << this->fingers[i].GetRotete()[j].GetRotateZ() << " "<< this->fingers[i].GetTrans()[j].GettransX() <<endl;
			}
		}
		f.close();
	}
	void LoadParams(char* filename){
		fstream f;
		f.open(filename, ios::in);
		f >> this->palm_position.x>> this->palm_position.y>> this->palm_position.z;

		float read_palmscale, read_RotateX, read_RotateY, read_RotateZ;
		RotateControl read_rotate;
		f >> read_palmscale >> read_RotateX >> read_RotateY >> read_RotateZ;

		read_rotate.SetRotateX(read_RotateX);
		read_rotate.SetRotateY(read_RotateY);
		read_rotate.SetRotateZ(read_RotateZ);
		this->palm.Setpalmscale(read_palmscale);
		this->palm.SetRotate(read_rotate);

		Finger *read_fingers = new Finger[5];
		for (int i = 0; i < 5; i++)
		{
			float read_fingerscale;
			f >> read_fingerscale;
			read_fingers[i].Setfingerscale(read_fingerscale);
			for (int j = 0; j < 4; j++) {
				RotateControl read_fingerRotation;
				TransControl read_fingerTrans;
				float read_fingerRotateX, read_fingerRotateY, read_fingerRotateZ, read_fingertrans;
				f >> read_fingerRotateX >> read_fingerRotateY >> read_fingerRotateZ>> read_fingertrans;
				read_fingerRotation.SetRotateX(read_fingerRotateX);
				read_fingerRotation.SetRotateY(read_fingerRotateY);
				read_fingerRotation.SetRotateZ(read_fingerRotateZ);
				read_fingerTrans.SettransX(read_fingertrans);

				read_fingers[i].SetRotate(read_fingerRotation, j);
				read_fingers[i].SetTrans(read_fingerTrans, j);

			}
		}
		this->SetFingers(read_fingers);
		this->ParamsToMat();
		f.close();
	}

	//计算梯度
	float ComputePalmPositionXGradient()
	{
		float gradient1, gradient2;
		float save;
		save = palm_position.x;
		this->palm_position.x = this->palm_position.x + _costfunction.position_h;
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_1 = cv::Mat::zeros(240, 320, CV_16UC1);

		cv::Mat generated_mat_1 = cv::Mat::zeros(424, 512, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_(model, generated_mat_1);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient1 = _costfunction.ComputeCostfunction(generated_mat_1, _costfunction.groundtruthmat);


		this->palm_position.x = this->palm_position.x - 2 * _costfunction.position_h;
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_2 = cv::Mat::zeros(240, 320, CV_16UC1);

		cv::Mat generated_mat_2 = cv::Mat::zeros(424, 512, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_(model, generated_mat_2);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient2 = _costfunction.ComputeCostfunction(generated_mat_2, _costfunction.groundtruthmat);
		float gradient_positionX = (gradient1 - gradient2) / 2.0*_costfunction.position_h;

		this->palm_position.x = save;
		this->ControlHand();
		return gradient_positionX;
	}
	float ComputePalmPositionYGradient()
	{
		float gradient1, gradient2;
		float save;
		save = this->palm_position.y;
		this->palm_position.y = this->palm_position.y + _costfunction.position_h;
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);

		//cv::Mat generated_mat_1 = cv::Mat::zeros(240, 320, CV_16UC1);
		cv::Mat generated_mat_1 = cv::Mat::zeros(424, 512, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_(model, generated_mat_1);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient1 = _costfunction.ComputeCostfunction(generated_mat_1, _costfunction.groundtruthmat);


		this->palm_position.y = this->palm_position.y - 2 * _costfunction.position_h;
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_2 = cv::Mat::zeros(240, 320, CV_16UC1);

		cv::Mat generated_mat_2 = cv::Mat::zeros(424, 512, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_(model, generated_mat_2);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient2 = _costfunction.ComputeCostfunction(generated_mat_2, _costfunction.groundtruthmat);
		float gradient_positionY = (gradient1 - gradient2) / 2.0*_costfunction.position_h;


		this->palm_position.y = save;
		this->ControlHand();
		return gradient_positionY;
	}
	float ComputePalmPositionZGradient()
	{
		float gradient1, gradient2;
		float save;
		save = this->palm_position.z;
		this->palm_position.z = this->palm_position.z + _costfunction.position_h;
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_1 = cv::Mat::zeros(240, 320, CV_16UC1);
		cv::Mat generated_mat_1 = cv::Mat::zeros(424, 512, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_(model, generated_mat_1);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient1 = _costfunction.ComputeCostfunction(generated_mat_1, _costfunction.groundtruthmat);


		this->palm_position.z = this->palm_position.z - 2 * _costfunction.position_h;
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_2 = cv::Mat::zeros(240, 320, CV_16UC1);
		cv::Mat generated_mat_2 = cv::Mat::zeros(424, 512, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_(model, generated_mat_2);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient2 = _costfunction.ComputeCostfunction(generated_mat_2, _costfunction.groundtruthmat);
		float gradient_positionZ = (gradient1 - gradient2) / 2.0*_costfunction.position_h;

		this->palm_position.z = save;
		this->ControlHand();
		return gradient_positionZ;
	}
	float ComputePalmScaleGradient()
	{
		float gradient1, gradient2;
		float save;
		save = this->palm.Getplamscale();
		this->palm.Setpalmscale(save + _costfunction.scale_h);
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_1 = cv::Mat::zeros(240, 320, CV_16UC1);;
		cv::Mat generated_mat_1 = cv::Mat::zeros(424, 512, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_(model, generated_mat_1);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient1 = _costfunction.ComputeCostfunction(generated_mat_1, _costfunction.groundtruthmat);


		this->palm.Setpalmscale(save - _costfunction.scale_h);
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_2 = cv::Mat::zeros(240, 320, CV_16UC1);;
		cv::Mat generated_mat_2 = cv::Mat::zeros(424, 512, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_(model, generated_mat_2);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient2 = _costfunction.ComputeCostfunction(generated_mat_2, _costfunction.groundtruthmat);
		float gradient_palmscale = (gradient1 - gradient2) / 2.0*_costfunction.scale_h;

		this->palm.Setpalmscale(save);
		this->ControlHand();
		return gradient_palmscale;
	}
	float ComputeFingerScaleGradient(int index)
	{
		float gradient1, gradient2;
		float save;
		save = this->fingers[index].Getfingerscale();
		this->fingers[index].Setfingerscale(save + _costfunction.scale_h);
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_1 = cv::Mat::zeros(240, 320, CV_16UC1);;
		cv::Mat generated_mat_1 = cv::Mat::zeros(424, 512, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_(model, generated_mat_1);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient1 = _costfunction.ComputeCostfunction(generated_mat_1, _costfunction.groundtruthmat);


		this->fingers[index].Setfingerscale(save - _costfunction.scale_h);
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_2 = cv::Mat::zeros(240, 320, CV_16UC1);;
		cv::Mat generated_mat_2 = cv::Mat::zeros(424, 512, CV_16UC1);;

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_(model, generated_mat_2);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient2 = _costfunction.ComputeCostfunction(generated_mat_2, _costfunction.groundtruthmat);
		float gradient_fingerscale = (gradient1 - gradient2) / 2.0*_costfunction.scale_h;

		this->fingers[index].Setfingerscale(save);
		this->ControlHand();
		return gradient_fingerscale;
	}
	float ComputeFingerTransGradient(int index1, int index2)
	{
		float gradient1, gradient2;
		float save;
		save = this->fingers[index1].GetTrans()[index2].GettransX();

		this->fingers[index1].SetTrans(save + _costfunction.trans_h, index2);
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_1 = cv::Mat::zeros(240, 320, CV_16UC1);;
		cv::Mat generated_mat_1 = cv::Mat::zeros(424, 512, CV_16UC1);;

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_(model, generated_mat_1);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient1 = _costfunction.ComputeCostfunction(generated_mat_1, _costfunction.groundtruthmat);


		this->fingers[index1].SetTrans(save - _costfunction.trans_h, index2);
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_2 = cv::Mat::zeros(240, 320, CV_16UC1);;
		cv::Mat generated_mat_2 = cv::Mat::zeros(424, 512, CV_16UC1);;

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_(model, generated_mat_2);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient2 = _costfunction.ComputeCostfunction(generated_mat_2, _costfunction.groundtruthmat);
		float gradient_fingertrans = (gradient1 - gradient2) / 2.0*_costfunction.trans_h;

		this->fingers[index1].SetTrans(save, index2);
		this->ControlHand();
		return gradient_fingertrans;
	}
	void ComputeGradient()
	{
		_costfunction.gradient[0] = this->ComputePalmPositionXGradient();
		_costfunction.gradient[1] = this->ComputePalmPositionYGradient();
		_costfunction.gradient[2] = this->ComputePalmPositionZGradient();

		/*_costfunction.gradient[3] = this->ComputePalmScaleGradient();

		for (int i = 0; i < 5; i++)
		{
			_costfunction.gradient[3 + i * 4 + 1] = this->ComputeFingerScaleGradient(i);
			_costfunction.gradient[3 + i * 4 + 2] = this->ComputeFingerTransGradient(i, 0);
			_costfunction.gradient[3 + i * 4 + 3] = this->ComputeFingerTransGradient(i, 1);
			_costfunction.gradient[3 + i * 4 + 4] = this->ComputeFingerTransGradient(i, 2);
		}*/

	}


	//计算点云的残差和轮廓的残差
	//void ComputeResidual(Mat &r_cloud,Mat &r_sil,Mat &r_3)
	//{
	//	this->ControlHand();
	//	SS::SubdivisionTheHand(model, 0);
	//	cv::Mat generated_mat = cv::Mat::zeros(240, 320, CV_16UC1);;
	//	projection->compute_current_orientation(model);
	//	projection->project_3d_to_2d_(model, generated_mat);

	//	_costfunction.ComputeSilhouetteDifference(generated_mat, _costfunction.groundtruthmat);
	//	_cloudpoint.Compute_Cloud_to_Mesh_Distance();

	//	r_3.at<float>(0, 0) = _costfunction.distance_renderTogroundtruth;
	//	for (int i = 0; i < r_sil.rows; i++)
	//	{
	//		r_sil.at<float>(i, 0) = 0.5*_costfunction.groundtruthTorenderMinDistance[i]/_costfunction.groundtruth_silhouetteSize;
	//	}
	//	for (int i = 0; i < r_cloud.rows; i++)
	//	{
	//		r_cloud.at<float>(i, 0) = 0.5*_cloudpoint.cloudpointTomesh_minDistance[i]/_cloudpoint.num_cloudpoint;
	//	}

	//}
	//void ComputeCloudPointJacobian(Mat &m1,Mat &m2,int index)
	//{
	//	this->ParamsToMat();
	//	float save = this->paramsOfhand.at<float>(index, 0);
	//	if ((index == 0) || (index == 4) || (index == 8) || (index == 12) || (index == 16) || (index == 20) || (index == 24))
	//	{
	//		this->paramsOfhand.at<float>(index, 0) = save + 0.1;
	//		this->MatToParams();
	//		this->ControlHand();
	//		SS::SubdivisionTheHand(model, 0);
	//		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
	//		for (int i = 0; i < m1.rows; i++)
	//		{
	//			m1.at<float>(i,0) = _cloudpoint.cloudpointTomesh_minDistance[i];
	//		}

	//		this->paramsOfhand.at<float>(index, 0) = save - 0.1;
	//		this->MatToParams();
	//		this->ControlHand();
	//		SS::SubdivisionTheHand(model, 0);
	//		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
	//		for (int i = 0; i < m2.rows; i++)
	//		{
	//			m2.at<float>(i, 0) = (m1.at<float>(i,0) -_cloudpoint.cloudpointTomesh_minDistance[i])/0.2;
	//		}
	//		this->paramsOfhand.at<float>(index, 0) = save;
	//		
	//	}
	//	else
	//	{
	//		this->paramsOfhand.at<float>(index, 0) = save + 1;
	//		this->MatToParams();
	//		this->ControlHand();
	//		SS::SubdivisionTheHand(model, 0);
	//		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
	//		for (int i = 0; i < m1.rows; i++)
	//		{
	//			m1.at<float>(i, 0) = _cloudpoint.cloudpointTomesh_minDistance[i];
	//		}

	//		this->paramsOfhand.at<float>(index, 0) = save - 1;
	//		this->MatToParams();
	//		this->ControlHand();
	//		SS::SubdivisionTheHand(model, 0);
	//		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
	//		for (int i = 0; i < m2.rows; i++)
	//		{
	//			m2.at<float>(i, 0) = (m1.at<float>(i, 0) - _cloudpoint.cloudpointTomesh_minDistance[i]) / 2.0;
	//		}
	//		this->paramsOfhand.at<float>(index, 0) = save;
	//		
	//	}
	//	this->MatToParams();
	//}
	//void ComputeSilhouetteJacobian(Mat &m1,Mat &m2,int index)
	//{
	//	this->ParamsToMat();
	//	float save = this->paramsOfhand.at<float>(index, 0);
	//	if ((index == 0) || (index == 4) || (index == 8) || (index == 12) || (index == 16) || (index == 20))
	//	{
	//		this->paramsOfhand.at<float>(index, 0) = save + 0.1;
	//		this->MatToParams();
	//		this->ControlHand();
	//		cv::Mat generated_mat1 = cv::Mat::zeros(240, 320, CV_16UC1);;
	//		projection->compute_current_orientation(model);
	//		projection->project_3d_to_2d_(model, generated_mat1);
	//		_costfunction.ComputeSilhouetteDifference(generated_mat1, _costfunction.groundtruthmat);

	//		for (int i = 0; i < m1.rows; i++)
	//		{
	//			m1.at<float>(i, 0) = _costfunction.groundtruthTorenderMinDistance[i];
	//		}

	//		this->paramsOfhand.at<float>(index, 0) = save - 0.1;
	//		this->MatToParams();
	//		this->ControlHand();
	//		cv::Mat generated_mat2 = cv::Mat::zeros(240, 320, CV_16UC1);;
	//		projection->compute_current_orientation(model);
	//		projection->project_3d_to_2d_(model, generated_mat2);
	//		_costfunction.ComputeSilhouetteDifference(generated_mat2, _costfunction.groundtruthmat);

	//		for (int i = 0; i < m2.rows; i++)
	//		{
	//			m2.at<float>(i, 0) = (m1.at<float>(i, 0) - _costfunction.groundtruthTorenderMinDistance[i]) / 0.2;
	//		}
	//		this->paramsOfhand.at<float>(index, 0) = save;
	//	}
	//	else
	//	{
	//		this->paramsOfhand.at<float>(index, 0) = save + 1;
	//		this->MatToParams();
	//		this->ControlHand();
	//		cv::Mat generated_mat1 = cv::Mat::zeros(240, 320, CV_16UC1);;
	//		projection->compute_current_orientation(model);
	//		projection->project_3d_to_2d_(model, generated_mat1);
	//		_costfunction.ComputeSilhouetteDifference(generated_mat1, _costfunction.groundtruthmat);

	//		for (int i = 0; i < m1.rows; i++)
	//		{
	//			m1.at<float>(i, 0) = _costfunction.groundtruthTorenderMinDistance[i];
	//		}

	//		this->paramsOfhand.at<float>(index, 0) = save - 1;
	//		this->MatToParams();
	//		this->ControlHand();
	//		cv::Mat generated_mat2 = cv::Mat::zeros(240, 320, CV_16UC1);;
	//		projection->compute_current_orientation(model);
	//		projection->project_3d_to_2d_(model, generated_mat2);
	//		_costfunction.ComputeSilhouetteDifference(generated_mat2, _costfunction.groundtruthmat);

	//		for (int i = 0; i < m2.rows; i++)
	//		{
	//			m2.at<float>(i, 0) = (m1.at<float>(i, 0) - _costfunction.groundtruthTorenderMinDistance[i]) / 2.0;
	//		}
	//		this->paramsOfhand.at<float>(index, 0) = save;
	//		
	//	}
	//	this->MatToParams();
	//}
	//float ComputeJ_3(int index)
	//{
	//	this->ParamsToMat();
	//	float gradient1, gradient2,gradient = 0;
	//	float save = this->paramsOfhand.at<float>(index, 0);
	//	if ((index == 0) || (index == 4) || (index == 8) || (index == 12) || (index == 16) || (index == 20))
	//	{
	//		this->paramsOfhand.at<float>(index, 0) = save + 0.1;
	//		this->MatToParams();
	//		this->ControlHand();
	//		cv::Mat generated_mat1 = cv::Mat::zeros(240, 320, CV_16UC1);;
	//		projection->compute_current_orientation(model);
	//		projection->project_3d_to_2d_(model, generated_mat1);
	//		_costfunction.ComputeSilhouetteDifference(generated_mat1, _costfunction.groundtruthmat);
	//		gradient1 = _costfunction.distance_renderTogroundtruth;


	//		this->paramsOfhand.at<float>(index, 0) = save - 0.1;
	//		this->MatToParams();
	//		this->ControlHand();
	//		cv::Mat generated_mat2 = cv::Mat::zeros(240, 320, CV_16UC1);;
	//		projection->compute_current_orientation(model);
	//		projection->project_3d_to_2d_(model, generated_mat2);
	//		_costfunction.ComputeSilhouetteDifference(generated_mat2, _costfunction.groundtruthmat);
	//		gradient2 = _costfunction.distance_renderTogroundtruth;
	//		gradient = (gradient1 - gradient2) / 0.2;

	//		this->paramsOfhand.at<float>(index, 0) = save;
	//	}
	//	else
	//	{
	//		this->paramsOfhand.at<float>(index, 0) = save + 1;
	//		this->MatToParams();
	//		this->ControlHand();
	//		cv::Mat generated_mat1 = cv::Mat::zeros(240, 320, CV_16UC1);;
	//		projection->compute_current_orientation(model);
	//		projection->project_3d_to_2d_(model, generated_mat1);
	//		_costfunction.ComputeSilhouetteDifference(generated_mat1, _costfunction.groundtruthmat);
	//		gradient1 = _costfunction.distance_renderTogroundtruth;


	//		this->paramsOfhand.at<float>(index, 0) = save - 1;
	//		this->MatToParams();
	//		this->ControlHand();
	//		cv::Mat generated_mat2 = cv::Mat::zeros(240, 320, CV_16UC1);;
	//		projection->compute_current_orientation(model);
	//		projection->project_3d_to_2d_(model, generated_mat2);
	//		_costfunction.ComputeSilhouetteDifference(generated_mat2, _costfunction.groundtruthmat);
	//		gradient2 = _costfunction.distance_renderTogroundtruth;
	//		gradient = (gradient1 - gradient2) / 2;

	//		this->paramsOfhand.at<float>(index, 0) = save;
	//	}
	//	this->MatToParams();
	//	return gradient;
	//}


	//设置（手指，手掌）大小，位置，长度的边界，如果调整超过边界，则进行约束
	void LimitedPalmScale()
	{
		if (this->palm.Getplamscale() > 2.5)
		{
			this->palm.Setpalmscale(2.5);
		}
		else
		{
			if (this->palm.Getplamscale() < 0.6)
			{
				this->palm.Setpalmscale(0.6);
			}
		}

	}
	void LimitedfingerScale()
	{
		for (int i = 0; i < 5; i++)
		{
			if (this->fingers[i].Getfingerscale() > 1.5)
			{
				this->fingers[i].Setfingerscale(1.5);
			}
			else
			{
				if (this->fingers[i].Getfingerscale() < 0.5)
				{
					this->fingers[i].Setfingerscale(0.5);
				}
			}
		}
	}
	void LimitedfingerLength()
	{
		for (int i = 0; i < 5; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				if (this->fingers[i].GetTrans()[j].GettransX() > 30)
				{
					this->fingers[i].SetTrans(30, j);
				}
				else
				{
					if (this->fingers[i].GetTrans()[j].GettransX() < -15)
					{
						this->fingers[i].SetTrans(-15, j);
					}
				}
			}
		}
	}


	//改变参数存储形式
	void ParamsToMat()
	{
		this->paramsOfhand[0] = this->palm.Getplamscale();
		this->paramsOfhand[1] = this->palm_position.x;
		this->paramsOfhand[2] = this->palm_position.y;
		this->paramsOfhand[3] = this->palm_position.z;

		for (int i = 0; i < 5; i++)
		{
			this->paramsOfhand[3 + i * 4 + 1] = this->fingers[i].Getfingerscale();
			this->paramsOfhand[3 + i * 4 + 2] = this->fingers[i].GetTrans()[0].GettransX();
			this->paramsOfhand[3 + i * 4 + 3] = this->fingers[i].GetTrans()[1].GettransX();
			this->paramsOfhand[3 + i * 4 + 4] = this->fingers[i].GetTrans()[2].GettransX();
		}
		
	}
	void MatToParams()
	{
		this->palm.Setpalmscale(this->paramsOfhand[0]);
		this->palm_position.x = this->paramsOfhand[1];
		this->palm_position.y = this->paramsOfhand[2];
		this->palm_position.z = this->paramsOfhand[3];

		for (int i = 0; i < 5; i++)
		{
			this->fingers[i].Setfingerscale(this->paramsOfhand[3 + i * 4 + 1]);
			this->fingers[i].SetTrans(this->paramsOfhand[3 + i * 4 + 2] , 0);
			this->fingers[i].SetTrans(this->paramsOfhand[3 + i * 4 + 3] , 1);
			this->fingers[i].SetTrans(this->paramsOfhand[3 + i * 4 + 4] , 2);
		}

	}

	void ParamsChange(float step)
	{
		this->palm_position.x = this->palm_position.x - step * _costfunction.gradient[0];
		this->palm_position.y = this->palm_position.y - step * _costfunction.gradient[1];
		this->palm_position.z = this->palm_position.z - step * _costfunction.gradient[2];

		this->palm.Setpalmscale(this->palm.Getplamscale() - step * _costfunction.gradient[3]);


		for (int i = 0; i < 5; i++)
		{
			this->fingers[i].Setfingerscale(this->fingers[i].Getfingerscale() - step * _costfunction.gradient[3 + i * 4 + 1]);
			this->fingers[i].SetTrans(this->fingers[i].GetTrans()[0].GettransX() - step *50* _costfunction.gradient[3 + i * 4 + 2], 0);
			this->fingers[i].SetTrans(this->fingers[i].GetTrans()[1].GettransX() - step *50* _costfunction.gradient[3 + i * 4 + 3], 1);
			this->fingers[i].SetTrans(this->fingers[i].GetTrans()[2].GettransX() - step *50* _costfunction.gradient[3 + i * 4 + 4], 2);
		}


		this->LimitedPalmScale();
		this->LimitedfingerScale();
		this->LimitedfingerLength();
	}

	float Function_Out()
	{
		this->ControlHand();

		SS::SubdivisionTheHand(model, 0);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();

		//cv::Mat generated_mat = cv::Mat::zeros(240, 320, CV_16UC1);
		cv::Mat generated_mat = cv::Mat::zeros(424, 512, CV_16UC1);
		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_(model, generated_mat);
		
		float function_out;
		function_out = _costfunction.ComputeCostfunction(generated_mat, _costfunction.groundtruthmat);
		return function_out;
	}
};


static HandControl *_handcontrol = new HandControl();


void func(double *p, double *x, int m, int n, void *data)
{
	//_handcontrol->SetParamOfHand(p);
	//_handcontrol->MatToParams();
	//_handcontrol->ControlHand();

	_handcontrol->palm.Setpalmscale(p[0]);
	_handcontrol->ControlHand();

	SS::SubdivisionTheHand(model, 2);
	cv::Mat generated_mat = cv::Mat::zeros(240, 320, CV_16UC1);;
	projection->compute_current_orientation(model);
	projection->project_3d_to_2d_(model, generated_mat);
	_cloudpoint.Compute_Cloud_to_Mesh_Distance();


	cv::Mat generated_BinaryMat = cv::Mat::zeros(240, 320, CV_64F);
	for (int i = 0; i < generated_mat.rows; i++)
	{
		for (int j = 0; j < generated_mat.cols; j++)
		{
			if (generated_mat.at<ushort>(i, j) != 0)
			{
				generated_BinaryMat.at<double>(i, j) = 1;
			}
		}
	}

	//for (int i = 0; i < _cloudpoint.num_cloudpoint; i++)
	//{
	//	x[i * 3 + 0] = _cloudpoint.cloudpointTomesh_inscribePoint[i*3 + 0];
	//	x[i * 3 + 1] = _cloudpoint.cloudpointTomesh_inscribePoint[i*3 + 1];
	//	x[i * 3 + 2] = _cloudpoint.cloudpointTomesh_inscribePoint[i*3 + 2];
	//}

	cout << "!!!!!!!!!!!" << endl;
	for (int i = 0; i <generated_BinaryMat.rows; i++)
	{
		for (int j = 0; j < generated_BinaryMat.cols; j++)
		{
			x[0*_cloudpoint.num_cloudpoint * 3 + generated_BinaryMat.cols*i + j] = generated_BinaryMat.at<double>(i, j);
		}
	}


	_handcontrol->_costfunction.ComputeCostfunction(generated_mat, _handcontrol->_costfunction.groundtruthmat);
	
}