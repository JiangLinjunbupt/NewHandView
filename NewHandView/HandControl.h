#pragma once

#include "Model.h"
#include "Random.h"
#include "opencv2/objdetect/objdetect.hpp"  
#include "opencv2/features2d/features2d.hpp"  
#include "opencv2/highgui/highgui.hpp"  
#include "opencv2/calib3d/calib3d.hpp"  
#include "opencv2/imgproc/imgproc_c.h"
//#include<levmar.h>
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

	Pose palm_rotation;


	int paramsSize;
	float* paramsOfhand;

	bool ParamsChangeStop;

	struct CostFunction
	{
		float weight1 = 1,weight2 = 0.5;
		Point p_center;
		int  ROI_len_x = 140;
		int  ROI_len_y = 140;
		double steparry[7] = {0.001, 0.005,0.01,0.03,0.05,0.08,0.1 };
		float step = 0.01;


		float trans_step = 0.5;
		float position_h = 1;
		float scale_h = 0.1;
		float trans_h = 1;
		float T = 0;
		float costfunction = 10000;
		//float gradient[24] = { 0 };
		float gradient[27] = { 0 };

		cv::Mat groundtruthmat = cv::Mat::zeros(424, 512, CV_16UC1);
		cv::Mat groundtruthROIMat = cv::Mat::zeros(ROI_len_x, ROI_len_y, CV_16UC1);
		cv::Mat groundtruthROIBinaryMat = cv::Mat::zeros(ROI_len_x, ROI_len_y, CV_64F);

		
		void ComputeGroundtruthRoIBinaryMat()
		{
			Moments moment = moments(groundtruthmat, true);
			p_center.x = moment.m10 / moment.m00;
			p_center.y = moment.m01 / moment.m00;


			for (int i = 0; i < ROI_len_x; i++)
			{
				for (int j = 0; j < ROI_len_y; j++) {
					if (i + p_center.x - ROI_len_x / 2 >= 0 && i + p_center.x - ROI_len_x / 2 <= groundtruthmat.cols-1 && j + p_center.y - ROI_len_y / 2 >=0 && j + p_center.y - ROI_len_y / 2  <= groundtruthmat.rows-1)
					{
						if (groundtruthmat.at<ushort>(j + p_center.y - ROI_len_y / 2, i + p_center.x - ROI_len_x / 2) != 0)
						{
							//groundtruthBinaryMat.at<double>(i,j) = 1.0;
							groundtruthROIBinaryMat.at<double>(j, i) = 1.0;
							groundtruthROIMat.at<ushort>(j, i) = groundtruthmat.at<ushort>(j + p_center.y - ROI_len_y / 2, i + p_center.x - ROI_len_x / 2);
						}
					}
				}
			}
		}
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

			cv::Mat differenceMat = groundtruthROIBinaryMat - rendered_BinaryMat;
			float difference = countNonZero(differenceMat);
			return difference;
		}
		float ComputeCostfunction(cv::Mat rendered_Silhouette)
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
		this->palm_position.z = -800; 
		this->palm_rotation.x = 0;
		this->palm_rotation.y = 0;
		this->palm_rotation.z = 0;

		//this->paramsSize = 24;
		this->paramsSize = 27;
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
		
		this->palm.SetRotate(this->palm_rotation.x, this->palm_rotation.y, this->palm_rotation.z);    //在使用Kinect真实深度时候，需要注释掉

		model->set_global_position(palm_position);
		model->set_global_position_center(palm_position);

		model->set_joint_scale(palm.Getplamscale(), 0);
		model->set_joint_scale(palm.Getplamscale(), 21);  //这里让手腕和手掌同等缩小放大，防止看起来怪异
		model->set_one_rotation(Pose(palm.GetRotate().GetRotateX(), palm.GetRotate().GetRotateY(), palm.GetRotate().GetRotateZ()),0);  //在使用Kinect真实深度时候，需要注释掉
		//model->set_hand_rotation(Pose(palm.GetRotate().GetRotateX(), palm.GetRotate().GetRotateY(), palm.GetRotate().GetRotateZ()));     //在使用模拟数据时候，需要注释掉

		for (int i = 0; i < 5; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				model->set_joint_scale(fingers[i].Getfingerscale(), i*4+j+1);
				model->set_one_rotation(Pose(fingers[i].GetRotete()[j].GetRotateX(), fingers[i].GetRotete()[j].GetRotateY(), fingers[i].GetRotete()[j].GetRotateZ()), i*4 + j + 1);
				model->set_one_trans(fingers[i].GetTrans()[j].GettransX(), i*4 + j + 1);
			}
		}

		//this->LoadGlovePose("Handinf.txt");
		//model->set_one_rotation(palm_rotation, 0);
		//model->set_one_rotation(Pose(0,0,90), 0);
		//model->set_hand_rotation(Pose(0, 0, 90));

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
	void AddNoiseToPalmRotation()
	{
		Random random;
		float x = this->palm.GetRotate().GetRotateX() + 20 * random.NextDouble();
		float y = this->palm.GetRotate().GetRotateY() + 40 ;
		float z = this->palm.GetRotate().GetRotateZ() + 20 * random.NextDouble();

		this->palm.SetRotate(x, y, z);

		this->palm_rotation.x = this->palm.GetRotate().GetRotateX();
		this->palm_rotation.y = this->palm.GetRotate().GetRotateY();
		this->palm_rotation.z = this->palm.GetRotate().GetRotateZ();

	}
	//根据梯度，或者牛顿高斯法改变参数，只改变大小，全局位置，和手指长度（sacle, position ,trans)


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
		float JudgeGradient[27];
		for (int i = 0; i < 27; i++)
		{
			if ((i == 3) || (i == 4) || (i == 8) || (i == 12) || (i == 16) || (i == 20))
			{
				JudgeGradient[i] = _costfunction.steparry[minindex] * _costfunction.gradient[i];
			}
			else
			{
				JudgeGradient[i] = _costfunction.steparry[minindex] * 50 * _costfunction.gradient[i];
			}
			if (abs(JudgeGradient[i]) > 0.001)
			{
				this->ParamsChangeStop = false;
			}
		}
	}


	////this function only adjust the palmScale
	//void ParamsChangeUsingGN()
	//{
	// //这里注意 创建Mat的时候是CV_32F， 则.at<float>这里用float，如果是CV_64F ，则必须用double，不然会出错
	//	Mat outputs(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);
	//	for (int i = 0; i < _costfunction.groundtruthBinaryMat.rows; i++)
	//	{
	//		for (int j = 0; j < _costfunction.groundtruthBinaryMat.cols; j++)
	//		{
	//			outputs.at<float>(i* _costfunction.groundtruthBinaryMat.cols + j, 0) = _costfunction.groundtruthBinaryMat.at<double>(i, j);
	//		}
	//	}
	//	
	//	float Params = 1.5;
	//	for (int itr = 0; itr < 100; itr++)
	//	{
	//		Mat residual(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);
	//		Mat Fun_outputs(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);
	//		Mat Fun_outputs_delta(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);
	//		Mat Jacobian(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);
	//		//先算Fun_outputs
	//		this->palm.Setpalmscale(Params);
	//		this->ControlHand();
	//		cv::Mat generated_mat_outputs = cv::Mat::zeros(240, 320, CV_16UC1);
	//		projection->compute_current_orientation(model);
	//		projection->project_3d_to_2d_(model, generated_mat_outputs);
	//		cv::Mat generated_mat_outputs_Binary = cv::Mat::zeros(240, 320, CV_64F);
	//		for (int i = 0; i < generated_mat_outputs.rows; i++)
	//		{
	//			for (int j = 0; j < generated_mat_outputs.cols; j++)
	//			{
	//				if (generated_mat_outputs.at<ushort>(i, j) != 0)
	//				{
	//					generated_mat_outputs_Binary.at<double>(i, j) = 1;
	//				}
	//			}
	//		}
	//		
	//		for (int i = 0; i <generated_mat_outputs_Binary.rows; i++)
	//		{
	//			for (int j = 0; j < generated_mat_outputs_Binary.cols; j++)
	//			{
	//				Fun_outputs.at<float>(i* generated_mat_outputs_Binary.cols + j, 0) = generated_mat_outputs_Binary.at<double>(i, j);
	//			}
	//		}
	//		//cout << countNonZero(Fun_outputs) << endl;
	//		//然后计算residual
	//		for (int i = 0; i < _costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows; i++)
	//		{
	//			residual.at<float>(i, 0) = Fun_outputs.at<float>(i, 0) - outputs.at<float>(i, 0);
	//			
	//		}
	//		
	//		//cout << countNonZero(residual) << endl;
	//		//然后算 Params + delta之后的Fun_outputs
	//		this->palm.Setpalmscale(Params + 0.01);
	//		this->ControlHand();
	//		cv::Mat generated_mat_outputs_delta = cv::Mat::zeros(240, 320, CV_16UC1);
	//		projection->compute_current_orientation(model);
	//		projection->project_3d_to_2d_(model, generated_mat_outputs_delta);
	//		cv::Mat generated_mat_outputs_delta_Binary = cv::Mat::zeros(240, 320, CV_64F);
	//		for (int i = 0; i < generated_mat_outputs_delta.rows; i++)
	//		{
	//			for (int j = 0; j < generated_mat_outputs_delta.cols; j++)
	//			{
	//				if (generated_mat_outputs_delta.at<ushort>(i, j) != 0)
	//				{
	//					generated_mat_outputs_delta_Binary.at<double>(i, j) = 1;
	//				}
	//			}
	//		}
	//		for (int i = 0; i <generated_mat_outputs_delta_Binary.rows; i++)
	//		{
	//			for (int j = 0; j < generated_mat_outputs_delta_Binary.cols; j++)
	//			{
	//				Fun_outputs_delta.at<float>(i* generated_mat_outputs_Binary.cols + j, 0) = generated_mat_outputs_delta_Binary.at<double>(i, j);
	//			}
	//		}
	//		//cout << countNonZero(Fun_outputs_delta) << endl;
	//		//然后计算Jacobian
	//		for (int i = 0; i < _costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows; i++)
	//		{
	//			Jacobian.at<float>(i, 0) = (Fun_outputs_delta.at<float>(i, 0) - Fun_outputs.at<float>(i, 0)) / 0.01;
	//		}
	//		//这一步是成功的关键！！！！
	//		//由于是采用前向差分计算的jacobian行列式，所以这里必须更新residual =（f(x+delta) + f(x) - 2*I)，
	//		//若继续使用residual = (f(x) - I)会造成Jacobian.t()*residual是全零的情况，具体原因我还没分析出来~
	//		//tomorrow work:  如果有多个自变量，比如24个params那么，要分别算每一个的J和每一个的residual,不能共用一个residual,然后组合成一个列向量,
	//		//或者其他的组合，明天在进行相关的尝试
	//		for (int i = 0; i < _costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows; i++)
	//		{
	//			residual.at<float>(i, 0) = residual.at<float>(i, 0) + Fun_outputs_delta.at<float>(i,0) - outputs.at<float>(i, 0);
	//		}
	//		//cout << countNonZero(Jacobian) << endl;
	//		//然后计算调整的delta
	//		//cout << Jacobian.t()*Jacobian << endl;
	//		//cout << ((Jacobian.t()*Jacobian)).inv() << endl;
	//		//cout << Jacobian.t() << endl;
	//		cout << "the " << itr << "th result is: " << endl;
	//		cout << Jacobian.t()*residual << endl;
	//		Mat delta = ((Jacobian.t()*Jacobian)).inv() * Jacobian.t()*residual;
	//		
	//		//Mat delta = Jacobian.t()*residual;
	//		cout << "delta is :"<<delta << endl;
	//		//然后根据delta改变Params
	//		Params = Params - delta.at<float>(0,0);
	//		cout << "params is:  "<<Params << endl;
	//		cout << endl << endl;
	//		if (abs(delta.at<float>(0, 0)) < 0.001)
	//		{
	//			break;
	//		}
	//	}
	//}
	//void ALLParamsChangeUsingGN()
	//{
	//	Mat outputs(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);
	//	for (int i = 0; i < _costfunction.groundtruthBinaryMat.rows; i++)
	//	{
	//		for (int j = 0; j < _costfunction.groundtruthBinaryMat.cols; j++)
	//		{
	//			outputs.at<float>(i* _costfunction.groundtruthBinaryMat.cols + j, 0) = _costfunction.groundtruthBinaryMat.at<double>(i, j);
	//		}
	//	}
	//	this->RandomScaleAndTransParams_justlittle();
	//	cout << "the original params is :  " << endl;
	//	for (int i = 0; i < this->paramsSize; i++)
	//	{
	//		cout << this->paramsOfhand[i] << endl;
	//	}
	//	cout << endl;
	//	//this->RandomScale();
	//	for (int itr = 0; itr < 100; itr++)
	//	{
	//		//整体的雅各比行列式
	//		Mat Jacobian(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, this->paramsSize, CV_32F);
	//		//整体的梯度
	//		Mat Gradient(this->paramsSize,1, CV_32F);
	//		//某一个变量的雅各比行列式
	//		Mat OneDimension_Jacobian(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);
	//		//某一个参数改变对应的输出，用于差分求导数
	//		Mat Fun_outputs(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);
	//		Mat Fun_outputs_delta(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);
	//		//residual
	//		Mat residual(_costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows, 1, CV_32F);
	//		Mat OneDimensionDelta = Mat::zeros(this->paramsSize,1, CV_32F);
	//		this->Function_Outputs(Fun_outputs);
	//		for (int paramsInt = 0; paramsInt <this->paramsSize; paramsInt++)
	//		{
	//			if ((paramsInt == 0) || (paramsInt == 4) || (paramsInt == 8) || (paramsInt == 12) || (paramsInt == 16) || (paramsInt == 20)) //如果参数代表scale,那么使用差分计算偏导的时候，使用delta = 0.01
	//			{
	//				this->paramsOfhand[paramsInt] = this->paramsOfhand[paramsInt] + 0.01;
	//			}
	//			else  //如果代表长度，那么使用差分法计算偏导的时候，delta选择0.1
	//			{
	//				this->paramsOfhand[paramsInt] = this->paramsOfhand[paramsInt] + 0.5;
	//			}
	//			this->Function_Outputs(Fun_outputs_delta);
	//			//下面是把参数还原，因为上面为了使用差分法计算偏导，加了一个delta
	//			if ((paramsInt == 0) || (paramsInt == 4) || (paramsInt == 8) || (paramsInt == 12) || (paramsInt == 16) || (paramsInt == 20)) //如果参数代表scale,那么使用差分计算偏导的时候，使用delta = 0.01
	//			{
	//				this->paramsOfhand[paramsInt] = this->paramsOfhand[paramsInt] - 0.01;
	//			}
	//			else  //如果代表长度，那么使用差分法计算偏导的时候，delta选择0.1
	//			{
	//				this->paramsOfhand[paramsInt] = this->paramsOfhand[paramsInt] - 0.5;
	//			}
	//			this->ComputeResidual(outputs, Fun_outputs, Fun_outputs_delta, residual);
	//			this->ComputeOneDimensionJacobian(paramsInt, Fun_outputs, Fun_outputs_delta, OneDimension_Jacobian);
	//			Mat D = (2*(OneDimension_Jacobian.t()*OneDimension_Jacobian)).inv() * OneDimension_Jacobian.t()*residual;
	//			//cout << D.at<float>(0, 0) << endl;
	//			OneDimensionDelta.at<float>(paramsInt, 0) = D.at<float>(0, 0);
	//			//cout << OneDimensionDelta.at<float>(paramsInt, 0) << endl;
	//			for (int i = 0; i < _costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows; i++)
	//			{
	//				Jacobian.at<float>(i, paramsInt) = OneDimension_Jacobian.at<float>(i, 0);
	//			}
	//			Mat OneDimensionGradient = OneDimension_Jacobian.t()*residual;
	//			//cout << OneDimensionGradient << endl;
	//			Gradient.at<float>(paramsInt, 0) = OneDimensionGradient.at<float>(0, 0);
	//		}
	//		Mat delta = (2*(Jacobian.t()*Jacobian)).inv() * Gradient;
	//		cout << "the " << itr << " th iteration :   the Params Changed is :  " << endl;
	//		for (int i = 0; i < this->paramsSize; i++)
	//		{
	//			//this->paramsOfhand[i] = this->paramsOfhand[i] - delta.at<float>(i, 0);
	//			this->paramsOfhand[i] = this->paramsOfhand[i] - OneDimensionDelta.at<float>(i,0);
	//			cout << this->paramsOfhand[i] << endl;
	//		}
	//		this->MatToParams();
	//		this->LimitedPalmScale();
	//		this->LimitedfingerScale();
	//		this->LimitedfingerLength();
	//		this->ParamsToMat();
	//		cout << endl;
	//		
	//	}
	//}

	//void Function_Outputs(Mat &func_outputs)
	//{
	//	this->MatToParams();
	//	this->ControlHand();
	//	//cv::Mat generated_mat_outputs = cv::Mat::zeros(240, 320, CV_16UC1);
	//	cv::Mat generated_mat_outputs = cv::Mat::zeros(424, 512, CV_16UC1);
	//	projection->compute_current_orientation(model);
	//	projection->project_3d_to_2d_(model, generated_mat_outputs);
	//	//cv::Mat generated_mat_outputs_Binary = cv::Mat::zeros(240, 320, CV_64F);
	//	cv::Mat generated_mat_outputs_Binary = cv::Mat::zeros(424, 512, CV_64F);
	//	for (int i = 0; i < generated_mat_outputs.rows; i++)
	//	{
	//		for (int j = 0; j < generated_mat_outputs.cols; j++)
	//		{
	//			if (generated_mat_outputs.at<ushort>(i, j) != 0)
	//			{
	//				generated_mat_outputs_Binary.at<double>(i, j) = 1;
	//			}
	//		}
	//	}
	//	for (int i = 0; i <generated_mat_outputs_Binary.rows; i++)
	//	{
	//		for (int j = 0; j < generated_mat_outputs_Binary.cols; j++)
	//		{
	//			func_outputs.at<float>(i* generated_mat_outputs_Binary.cols + j, 0) = generated_mat_outputs_Binary.at<double>(i, j);
	//		}
	//	}
	//}

	//void ComputeResidual(const Mat &outputs,const Mat &func_outputs,const Mat &func_outputs_delta,Mat &residual )
	//{
	//	for (int i = 0; i < _costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows; i++)
	//	{
	//		residual.at<float>(i, 0) = func_outputs.at<float>(i,0) + func_outputs_delta.at<float>(i, 0) - 2*outputs.at<float>(i, 0);
	//	}
	//}
	//void ComputeOneDimensionJacobian(int index, const Mat &func_outputs, const Mat &func_outputs_delta, Mat &OneDimJacobian)
	//{
	//	if ((index == 0) || (index == 4) || (index == 8) || (index == 12) || (index == 16) || (index == 20)) //如果参数代表scale,那么计算差分计算偏导的时候，使用delta = 0.01
	//	{
	//		for (int i = 0; i < _costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows; i++)
	//		{
	//			OneDimJacobian.at<float>(i, 0) = (func_outputs_delta.at<float>(i, 0) - func_outputs.at<float>(i, 0)) / 0.01;
	//		}
	//	}
	//	else
	//	{
	//		for (int i = 0; i < _costfunction.groundtruthBinaryMat.cols*_costfunction.groundtruthBinaryMat.rows; i++)
	//		{
	//			OneDimJacobian.at<float>(i, 0) = (func_outputs_delta.at<float>(i, 0) - func_outputs.at<float>(i, 0)) / 0.5;
	//		}
	//	}
	//}
	

	//计算梯度
	float ComputePalmPositionXGradient()
	{
		float gradient1, gradient2;
		float save;
		save = palm_position.x;
		this->palm_position.x = save + _costfunction.position_h;
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_1 = cv::Mat::zeros(240, 320, CV_16UC1);

		cv::Mat generated_mat_1 = cv::Mat::zeros(_costfunction.ROI_len_y, _costfunction.ROI_len_x, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_when_calc(model, generated_mat_1);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient1 = _costfunction.ComputeCostfunction(generated_mat_1);


		this->palm_position.x = save - _costfunction.position_h;
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_2 = cv::Mat::zeros(240, 320, CV_16UC1);

		cv::Mat generated_mat_2 = cv::Mat::zeros(_costfunction.ROI_len_y, _costfunction.ROI_len_x, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_when_calc(model, generated_mat_2);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient2 = _costfunction.ComputeCostfunction(generated_mat_2);
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
		cv::Mat generated_mat_1 = cv::Mat::zeros(_costfunction.ROI_len_y, _costfunction.ROI_len_x, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_when_calc(model, generated_mat_1);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient1 = _costfunction.ComputeCostfunction(generated_mat_1);


		this->palm_position.y = this->palm_position.y - 2 * _costfunction.position_h;
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_2 = cv::Mat::zeros(240, 320, CV_16UC1);

		cv::Mat generated_mat_2 = cv::Mat::zeros(_costfunction.ROI_len_y, _costfunction.ROI_len_x, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_when_calc(model, generated_mat_2);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient2 = _costfunction.ComputeCostfunction(generated_mat_2);
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
		cv::Mat generated_mat_1 = cv::Mat::zeros(_costfunction.ROI_len_y, _costfunction.ROI_len_x, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_when_calc(model, generated_mat_1);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient1 = _costfunction.ComputeCostfunction(generated_mat_1);


		this->palm_position.z = this->palm_position.z - 2 * _costfunction.position_h;
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_2 = cv::Mat::zeros(240, 320, CV_16UC1);
		cv::Mat generated_mat_2 = cv::Mat::zeros(_costfunction.ROI_len_y, _costfunction.ROI_len_x, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_when_calc(model, generated_mat_2);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient2 = _costfunction.ComputeCostfunction(generated_mat_2);
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
		cv::Mat generated_mat_1 = cv::Mat::zeros(_costfunction.ROI_len_y, _costfunction.ROI_len_x, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_when_calc(model, generated_mat_1);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient1 = _costfunction.ComputeCostfunction(generated_mat_1);


		this->palm.Setpalmscale(save - _costfunction.scale_h);
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_2 = cv::Mat::zeros(240, 320, CV_16UC1);;
		cv::Mat generated_mat_2 = cv::Mat::zeros(_costfunction.ROI_len_y, _costfunction.ROI_len_x, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_when_calc(model, generated_mat_2);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient2 = _costfunction.ComputeCostfunction(generated_mat_2);
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
		cv::Mat generated_mat_1 = cv::Mat::zeros(_costfunction.ROI_len_y, _costfunction.ROI_len_x, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_when_calc(model, generated_mat_1);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient1 = _costfunction.ComputeCostfunction(generated_mat_1);


		this->fingers[index].Setfingerscale(save - _costfunction.scale_h);
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_2 = cv::Mat::zeros(240, 320, CV_16UC1);;
		cv::Mat generated_mat_2 = cv::Mat::zeros(_costfunction.ROI_len_y, _costfunction.ROI_len_x, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_when_calc(model, generated_mat_2);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient2 = _costfunction.ComputeCostfunction(generated_mat_2);
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
		cv::Mat generated_mat_1 = cv::Mat::zeros(_costfunction.ROI_len_y, _costfunction.ROI_len_x, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_when_calc(model, generated_mat_1);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient1 = _costfunction.ComputeCostfunction(generated_mat_1);


		this->fingers[index1].SetTrans(save - _costfunction.trans_h, index2);
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_2 = cv::Mat::zeros(240, 320, CV_16UC1);;
		cv::Mat generated_mat_2 = cv::Mat::zeros(_costfunction.ROI_len_y, _costfunction.ROI_len_x, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_when_calc(model, generated_mat_2);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient2 = _costfunction.ComputeCostfunction(generated_mat_2);
		float gradient_fingertrans = (gradient1 - gradient2) / 2.0*_costfunction.trans_h;

		this->fingers[index1].SetTrans(save, index2);
		this->ControlHand();
		return gradient_fingertrans;
	}

	float ComputePalmRotationXGradient()
	{
		float gradient1, gradient2;
		float save;
		save = palm_rotation.x;
		this->palm_rotation.x = this->palm_rotation.x + _costfunction.position_h;
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_1 = cv::Mat::zeros(240, 320, CV_16UC1);

		cv::Mat generated_mat_1 = cv::Mat::zeros(_costfunction.ROI_len_y, _costfunction.ROI_len_x, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_when_calc(model, generated_mat_1);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient1 = _costfunction.ComputeCostfunction(generated_mat_1);


		this->palm_rotation.x = this->palm_rotation.x - 2 * _costfunction.position_h;
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_2 = cv::Mat::zeros(240, 320, CV_16UC1);

		cv::Mat generated_mat_2 = cv::Mat::zeros(_costfunction.ROI_len_y, _costfunction.ROI_len_x, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_when_calc(model, generated_mat_2);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient2 = _costfunction.ComputeCostfunction(generated_mat_2);
		float gradient_positionX = (gradient1 - gradient2) / 2.0*_costfunction.position_h;

		this->palm_rotation.x = save;
		this->ControlHand();
		return gradient_positionX;
	}
	float ComputePalmRotationYGradient()
	{
		float gradient1, gradient2;
		float save;
		save = palm_rotation.y;
		this->palm_rotation.y = this->palm_rotation.y + _costfunction.position_h;
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_1 = cv::Mat::zeros(240, 320, CV_16UC1);

		cv::Mat generated_mat_1 = cv::Mat::zeros(_costfunction.ROI_len_y, _costfunction.ROI_len_x, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_when_calc(model, generated_mat_1);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient1 = _costfunction.ComputeCostfunction(generated_mat_1);


		this->palm_rotation.y = this->palm_rotation.y - 2 * _costfunction.position_h;
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_2 = cv::Mat::zeros(240, 320, CV_16UC1);

		cv::Mat generated_mat_2 = cv::Mat::zeros(_costfunction.ROI_len_y, _costfunction.ROI_len_x, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_when_calc(model, generated_mat_2);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient2 = _costfunction.ComputeCostfunction(generated_mat_2);
		float gradient_positionX = (gradient1 - gradient2) / 2.0*_costfunction.position_h;

		this->palm_rotation.y = save;
		this->ControlHand();
		return gradient_positionX;
	}
	float ComputePalmRotationZGradient()
	{
		float gradient1, gradient2;
		float save;
		save = palm_rotation.z;
		this->palm_rotation.z = this->palm_rotation.z + _costfunction.position_h;
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_1 = cv::Mat::zeros(240, 320, CV_16UC1);

		cv::Mat generated_mat_1 = cv::Mat::zeros(_costfunction.ROI_len_y, _costfunction.ROI_len_x, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_when_calc(model, generated_mat_1);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient1 = _costfunction.ComputeCostfunction(generated_mat_1);


		this->palm_rotation.z = this->palm_rotation.z - 2 * _costfunction.position_h;
		this->ControlHand();
		SS::SubdivisionTheHand(model, 0);
		//cv::Mat generated_mat_2 = cv::Mat::zeros(240, 320, CV_16UC1);

		cv::Mat generated_mat_2 = cv::Mat::zeros(_costfunction.ROI_len_y, _costfunction.ROI_len_x, CV_16UC1);

		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_when_calc(model, generated_mat_2);
		_cloudpoint.Compute_Cloud_to_Mesh_Distance();
		gradient2 = _costfunction.ComputeCostfunction(generated_mat_2);
		float gradient_positionX = (gradient1 - gradient2) / 2.0*_costfunction.position_h;

		this->palm_rotation.z = save;
		this->ControlHand();
		return gradient_positionX;
	}
	void  ComputeGradient()
	{
		_costfunction.gradient[0] = this->ComputePalmPositionXGradient();
		_costfunction.gradient[1] = this->ComputePalmPositionYGradient();
		_costfunction.gradient[2] = this->ComputePalmPositionZGradient();

		//_costfunction.gradient[3] = this->ComputePalmScaleGradient();

		/*for (int i = 0; i < 5; i++)
		{
			_costfunction.gradient[3 + i * 4 + 1] = this->ComputeFingerScaleGradient(i);
			_costfunction.gradient[3 + i * 4 + 2] = this->ComputeFingerTransGradient(i, 0);
			_costfunction.gradient[3 + i * 4 + 3] = this->ComputeFingerTransGradient(i, 1);
			_costfunction.gradient[3 + i * 4 + 4] = this->ComputeFingerTransGradient(i, 2);
		}*/

		_costfunction.gradient[24] = this->ComputePalmRotationXGradient();
		_costfunction.gradient[25] = this->ComputePalmRotationYGradient();
		_costfunction.gradient[26] = this->ComputePalmRotationZGradient();


	}


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

		this->paramsOfhand[24] = this->palm_rotation.x;
		this->paramsOfhand[25] = this->palm_rotation.y;
		this->paramsOfhand[26] = this->palm_rotation.z;

		
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

		this->palm_rotation.x = this->paramsOfhand[24];
		this->palm_rotation.y = this->paramsOfhand[25];
		this->palm_rotation.z = this->paramsOfhand[26];

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


		this->palm_rotation.x = this->palm_rotation.x - step*20*_costfunction.gradient[24];
		this->palm_rotation.y = this->palm_rotation.y - step*20*_costfunction.gradient[25];
		this->palm_rotation.z = this->palm_rotation.z - step*20*_costfunction.gradient[26];


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
		cv::Mat generated_mat = cv::Mat::zeros(_costfunction.ROI_len_y, _costfunction.ROI_len_x, CV_16UC1);
		projection->compute_current_orientation(model);
		projection->project_3d_to_2d_when_calc(model, generated_mat);
		
		float function_out;
		function_out = _costfunction.ComputeCostfunction(generated_mat);
		return function_out;
	}

	void LoadGlovePose(char *filename)
	{
		fstream f;
		f.open(filename, ios::in);

		for (int i = 0; i < 5; i++)
		{
			if (i != 4)
			{
				float Mcp_x, Mcp_z, Pip;
				f >> Mcp_x >> Mcp_z >> Pip;
				Pose p1(0, Mcp_x, Mcp_z), p2(0, Pip, 0), p3(0, Pip*0.66, 0);

				//this->fingers[3 - i].SetRotate(0, Mcp_x, Mcp_z, 0);
				this->fingers[3 - i].SetRotate(0, Mcp_x, 0, 0);
				this->fingers[3 - i].SetRotate(0, Pip, 0, 1);
				this->fingers[3 - i].SetRotate(0, Pip*0.66, 0, 2);
				/*model->set_one_rotation(p1, 13 - i * 4);
				model->set_one_rotation(p2, 14 - i * 4);
				model->set_one_rotation(p3, 15 - i * 4);*/
			}
			else
			{
				float Mcp_x, Mcp_z, Pip;
				f >> Mcp_x >> Mcp_z >> Pip;
				Pose p1(0, Mcp_x, 0), p2(0, Pip, 0);
				this->fingers[i].SetRotate(0, Mcp_x, Mcp_z, 0);
				this->fingers[i].SetRotate(0, Pip, 0, 2);
				//model->set_one_rotation(p1, 17);
				////model->set_one_rotation(p2, 18);
				//model->set_one_rotation(p2, 19);
			}
		}

		float global_x, global_y, global_z;
		f >> global_x >> global_y >> global_z;
		Pose p_global(global_x, global_y, global_z);
		RotateControl read_rotate;
		read_rotate.SetRotateX(global_x);
		read_rotate.SetRotateY(global_y);
		read_rotate.SetRotateZ(global_z);
		this->palm.SetRotate(read_rotate);
		//model->set_hand_rotation(p_global);
		f.close();
	}

	//存储和读出参数
	void SaveParams(char* filename) {
		ofstream f;
		f.open(filename, ios::out);
		f << this->palm_position.x << " " << this->palm_position.y << " " << this->palm_position.z << endl;

		f << this->palm.Getplamscale() << " " << this->palm.GetRotate().GetRotateX() << " " << this->palm.GetRotate().GetRotateY() << " " << this->palm.GetRotate().GetRotateZ() << endl;
		for (int i = 0; i < 5; i++) {
			f << this->fingers[i].Getfingerscale() << endl;
			for (int j = 0; j < 4; j++)
			{
				f << this->fingers[i].GetRotete()[j].GetRotateX() << " " << this->fingers[i].GetRotete()[j].GetRotateY() << " " << this->fingers[i].GetRotete()[j].GetRotateZ() << " " << this->fingers[i].GetTrans()[j].GettransX() << endl;
			}
		}
		f.close();
	}
	void LoadParams(char* filename) {
		fstream f;
		f.open(filename, ios::in);
		f >> this->palm_position.x >> this->palm_position.y >> this->palm_position.z;

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
				f >> read_fingerRotateX >> read_fingerRotateY >> read_fingerRotateZ >> read_fingertrans;
				read_fingerRotation.SetRotateX(read_fingerRotateX);
				read_fingerRotation.SetRotateY(read_fingerRotateY);
				read_fingerRotation.SetRotateZ(read_fingerRotateZ);
				read_fingerTrans.SettransX(read_fingertrans);

				read_fingers[i].SetRotate(read_fingerRotation, j);
				read_fingers[i].SetTrans(read_fingerTrans, j);

			}
		}
		this->SetFingers(read_fingers);

		this->palm_rotation.x = this->palm.GetRotate().GetRotateX();
		this->palm_rotation.y = this->palm.GetRotate().GetRotateY();
		this->palm_rotation.z = this->palm.GetRotate().GetRotateZ();

		this->ParamsToMat();
		f.close();
	}
};


static HandControl *_handcontrol = new HandControl();


//void func(double *p, double *x, int m, int n, void *data)
//{
//	//_handcontrol->SetParamOfHand(p);
//	//_handcontrol->MatToParams();
//	//_handcontrol->ControlHand();
//
//	_handcontrol->palm.Setpalmscale(p[0]);
//	_handcontrol->ControlHand();
//
//	SS::SubdivisionTheHand(model, 2);
//	cv::Mat generated_mat = cv::Mat::zeros(240, 320, CV_16UC1);;
//	projection->compute_current_orientation(model);
//	projection->project_3d_to_2d_(model, generated_mat);
//	_cloudpoint.Compute_Cloud_to_Mesh_Distance();
//
//
//	cv::Mat generated_BinaryMat = cv::Mat::zeros(240, 320, CV_64F);
//	for (int i = 0; i < generated_mat.rows; i++)
//	{
//		for (int j = 0; j < generated_mat.cols; j++)
//		{
//			if (generated_mat.at<ushort>(i, j) != 0)
//			{
//				generated_BinaryMat.at<double>(i, j) = 1;
//			}
//		}
//	}
//
//	//for (int i = 0; i < _cloudpoint.num_cloudpoint; i++)
//	//{
//	//	x[i * 3 + 0] = _cloudpoint.cloudpointTomesh_inscribePoint[i*3 + 0];
//	//	x[i * 3 + 1] = _cloudpoint.cloudpointTomesh_inscribePoint[i*3 + 1];
//	//	x[i * 3 + 2] = _cloudpoint.cloudpointTomesh_inscribePoint[i*3 + 2];
//	//}
//
//	cout << "!!!!!!!!!!!" << endl;
//	for (int i = 0; i <generated_BinaryMat.rows; i++)
//	{
//		for (int j = 0; j < generated_BinaryMat.cols; j++)
//		{
//			x[0*_cloudpoint.num_cloudpoint * 3 + generated_BinaryMat.cols*i + j] = generated_BinaryMat.at<double>(i, j);
//		}
//	}
//
//
//	_handcontrol->_costfunction.ComputeCostfunction(generated_mat);
//	
//}