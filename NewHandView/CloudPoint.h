#pragma once
#include<opencv2/opencv.hpp>    
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include "SubdivisionSurfaces.h"

struct CloudPoint {
	//记录思路，如果我们能知道每个点云是由深度图中代表哪个手部部分的像素点得到的，那么我们可以知道这个点云大致对应的手指部分，在后续计算与手摸之间的距离的时候，可以加以利用
	float *cloudpoint;
	float *cloudpointTomesh_minDistance;
	float *cloudpointTomesh_inscribePoint;
	int num_cloudpoint;
	float SumDistance;
	vector<cv::Point3f> cloudpoint_vector;
	vector<cv::Point3f> visibleHandvertices;


	float center_x, center_y, center_z;

	CloudPoint() :cloudpoint(nullptr), cloudpointTomesh_minDistance(nullptr), cloudpointTomesh_inscribePoint(nullptr) {};
	~CloudPoint() {
		delete[] cloudpoint;
		delete[] cloudpointTomesh_minDistance;
		delete[] cloudpointTomesh_inscribePoint;
	}

	void init(cv::Mat depthmat)
	{
		int Num_NotZeroPixel = 0;
		for (int i = 0; i < depthmat.rows; i++)
		{
			for (int j = 0; j < depthmat.cols; j++)
			{
				if (depthmat.at<ushort>(i, j) != 0)
				{
					Num_NotZeroPixel++;
				}
			}
		}

		this->num_cloudpoint = Num_NotZeroPixel;
		this->cloudpoint = new float[num_cloudpoint * 3];
		this->cloudpointTomesh_minDistance = new float[num_cloudpoint];
		this->cloudpointTomesh_inscribePoint = new float[num_cloudpoint * 3];

		center_x = 0;
		center_y = 0;
		center_z = 0;
	}

	void DepthMatToCloudPoint(cv::Mat depthmat, double focalx, double focaly,float centerx, float centery)
	{
		int k = 0;
		this->center_x = 0;
		this->center_y = 0;
		this->center_z = 0;

		for (int i = 0; i < depthmat.rows; i++)
		{
			for (int j = 0; j < depthmat.cols; j++)
			{
				if (depthmat.at<ushort>(i, j) != 0)
				{
					cv::Point3f p;
					/*p.x = (j - centerx) * (-depthmat.at<ushort>(i, j)) / focal;
					p.y = -(i - centery) * (-depthmat.at<ushort>(i, j)) / focal;
					p.z = -depthmat.at<ushort>(i, j);*/


					p.x = (j - centerx) * (-depthmat.at<ushort>(i, j)) / focalx;
					p.y = (i - centery) * (-depthmat.at<ushort>(i, j)) / focaly;
					p.z = -depthmat.at<ushort>(i, j);

					this->center_x += p.x;
					this->center_y += p.y;
					this->center_z += p.z;

					this->cloudpoint_vector.push_back(p);
					this->cloudpoint[k] = p.x;
					this->cloudpoint[k + 1] = p.y;
					this->cloudpoint[k + 2] = p.z;

					//cout << "the " << k << " point is : x  " << this->cloudpoint[k] << "  y: " << this->cloudpoint[k + 1] << " z: " << this->cloudpoint[k + 2] << endl;
					k = k + 3;
				}
			}
		}

		this->center_x = 3.0*this->center_x / (float)k;
		this->center_y = 3.0*this->center_y / (float)k;
		this->center_z = 3.0*this->center_z / (float)k;

	}

	void Compute_Cloud_to_Mesh_Distance2()
	{
		this->SumDistance = 0;
		for (int i = 0; i < num_cloudpoint; i++)
		{
			float MinDistance = 100000;
			float inscribeX = 0;
			float inscribeY = 0;
			float inscribeZ = 0;

			float Px = cloudpoint[i * 3 + 0];
			float Py = cloudpoint[i * 3 + 1];
			float Pz = cloudpoint[i * 3 + 2];

			for (int j = 0; j < SS::visiblePatches.size(); j++)
			{
				//开始计算距离
				float Ax = SS::disVertices[SS::visiblePatches[j].v_idx(0)].position.x;
				float Ay = SS::disVertices[SS::visiblePatches[j].v_idx(0)].position.y;
				float Az = SS::disVertices[SS::visiblePatches[j].v_idx(0)].position.z;

				float Bx = SS::disVertices[SS::visiblePatches[j].v_idx(1)].position.x;
				float By = SS::disVertices[SS::visiblePatches[j].v_idx(1)].position.y;
				float Bz = SS::disVertices[SS::visiblePatches[j].v_idx(1)].position.z;

				float Cx = SS::disVertices[SS::visiblePatches[j].v_idx(2)].position.x;;
				float Cy = SS::disVertices[SS::visiblePatches[j].v_idx(2)].position.y;;
				float Cz = SS::disVertices[SS::visiblePatches[j].v_idx(2)].position.z;;


				float Distance_AP = sqrt(pow((Ax - Px), 2) + pow((Ay - Py), 2) + pow((Az - Pz), 2));
				float Distance_BP = sqrt(pow((Bx - Px), 2) + pow((By - Py), 2) + pow((Bz - Pz), 2));
				float Distance_CP = sqrt(pow((Cx - Px), 2) + pow((Cy - Py), 2) + pow((Cz - Pz), 2));


				//这里目前采用该点云到三角网格三个顶点的距离来衡量，但是这部分是可以优化的，由于目前还没想到更好的表示点云和模型之间距离差异的其他方法，就先采用这个方法。
				float Distance_sum = (Distance_AP + Distance_BP + Distance_CP) / 3.0f;

				if (Distance_sum < MinDistance)
				{
					MinDistance = Distance_sum;
					inscribeX = (Ax + Bx) / 4.0f + Cx / 2.0f;          //这里随便选了三角形内部的一个点
					inscribeY = (Ay + By) / 4.0f + Cy / 2.0f;
					inscribeZ = (Az + Bz) / 4.0f + Cz / 2.0f;
				}
			}


			this->SumDistance += MinDistance;
			cloudpointTomesh_minDistance[i] = MinDistance;
			cloudpointTomesh_inscribePoint[i * 3] = inscribeX;
			cloudpointTomesh_inscribePoint[i * 3 + 1] = inscribeY;
			cloudpointTomesh_inscribePoint[i * 3 + 2] = inscribeZ;

		}
	}

	void Compute_Cloud_to_Mesh_Distance()
	{
		this->SumDistance = 0;
		cv::Mat target = cv::Mat(cloudpoint_vector).reshape(1);
		target.convertTo(target, CV_32F);

		visibleHandvertices.clear();
		vector<float> weight;

		//int handweightCount = 0;
		for (int j = 0; j < SS::visiblePatches.size(); j++)
		{
			cv::Point3f p;
			

			float Ax = SS::disVertices[SS::visiblePatches[j].v_idx(0)].position.x;
			float Ay = SS::disVertices[SS::visiblePatches[j].v_idx(0)].position.y;
			float Az = SS::disVertices[SS::visiblePatches[j].v_idx(0)].position.z;

			float Bx = SS::disVertices[SS::visiblePatches[j].v_idx(1)].position.x;
			float By = SS::disVertices[SS::visiblePatches[j].v_idx(1)].position.y;
			float Bz = SS::disVertices[SS::visiblePatches[j].v_idx(1)].position.z;

			float Cx = SS::disVertices[SS::visiblePatches[j].v_idx(2)].position.x;;
			float Cy = SS::disVertices[SS::visiblePatches[j].v_idx(2)].position.y;;
			float Cz = SS::disVertices[SS::visiblePatches[j].v_idx(2)].position.z;;

			//cout << "1:  " << SS::disVertices[SS::visiblePatches[j].v_idx(0)].weight << "   2:  " << SS::disVertices[SS::visiblePatches[j].v_idx(2)].weight << "   3:  " << SS::disVertices[SS::visiblePatches[j].v_idx(2)].weight << endl;
			

			float max_weight = SS::disVertices[SS::visiblePatches[j].v_idx(0)].weight;

			//if (SS::disVertices[SS::visiblePatches[j].v_idx(0)].weight > 2 || SS::disVertices[SS::visiblePatches[j].v_idx(1)].weight > 2 || SS::disVertices[SS::visiblePatches[j].v_idx(2)].weight > 2)
			//{
			//	handweightCount++;
			//}

			if (SS::disVertices[SS::visiblePatches[j].v_idx(1)].weight >max_weight)
			{
				max_weight = SS::disVertices[SS::visiblePatches[j].v_idx(1)].weight;
			}

			if (SS::disVertices[SS::visiblePatches[j].v_idx(2)].weight > max_weight)
			{
				max_weight = SS::disVertices[SS::visiblePatches[j].v_idx(2)].weight;
			}

			//if (max_weight > 2.0)
			//{
			//	handweightCount++;
			//}
			p.x = (Ax + Bx) / 4.0f + Cx / 2.0f;
			p.y = (Ay + By) / 4.0f + Cy / 2.0f;
			p.z = (Az + Bz) / 4.0f + Cz / 2.0f;
			visibleHandvertices.push_back(p);
			weight.push_back(max_weight);
		}


		//cout << "the size is : "<<weight.size()<<"    handweightCount is：" << handweightCount << endl;

		cv::Mat sourse = cv::Mat(visibleHandvertices).reshape(1);
		sourse.convertTo(sourse, CV_32F);


		flann::KDTreeIndexParams indexParams(4);
		flann::Index kdtree(sourse, indexParams);

		int k = 1;
		cv::Mat indices(target.rows, k, CV_32F);
		cv::Mat dists(target.rows, k, CV_32F);         //搜索到的最近邻的距离

		kdtree.knnSearch(target, indices, dists, k, cv::flann::SearchParams(32));



		for (int i = 0; i < dists.rows; i++)
		{
			this->SumDistance = this->SumDistance + dists.at<float>(i, 0)*weight[indices.at<int>(i, 0)];
			cloudpointTomesh_minDistance[i] = sqrt(dists.at<float>(i, 0));
			cloudpointTomesh_inscribePoint[i * 3] = visibleHandvertices[indices.at<int>(i, 0)].x;
			cloudpointTomesh_inscribePoint[i * 3 + 1] = visibleHandvertices[indices.at<int>(i, 0)].y;
			cloudpointTomesh_inscribePoint[i * 3 + 2] = visibleHandvertices[indices.at<int>(i, 0)].z;
		}
		

	}


};

static CloudPoint _cloudpoint;

