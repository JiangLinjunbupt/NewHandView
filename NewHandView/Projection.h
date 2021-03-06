#pragma once
#include<opencv2\opencv.hpp>
#include<opencv2\core\core.hpp>
#include<opencv2\highgui\highgui.hpp>
#include<opencv2\imgproc\imgproc.hpp>
#include "model.h"
#include <vector>
#include <fstream>

using namespace std;
#include <direct.h>
//struct ProjectParam {
//	double focal;
//	double centerx;
//	double centery;
//	ProjectParam() :focal(241.3), centerx(160), centery(120) {}
//};

struct ProjectParam {
	double focalx;
	double focaly;
	double centerx;
	double centery;
	ProjectParam() :focalx(381.8452), focaly(382.1713),centerx(264.0945), centery(217.1487) {}
};

struct Pixel {
	int x;
	int y;
};


int map_label[NUM_JOINT] = { 1,2,3,4,0,5,6,7,0,8,9,10,0,11,12,13,0,14,15,16,0,17 };


class Projection {
public:


	Projection(int height, int width, int num_joint, int num_pose,int RoI_lenx,int RoI_leny) {
		depth_ = cv::Mat::zeros(height, width, CV_16UC1);
		part_ = cv::Mat::zeros(height, width, CV_8UC1);
		orient_ = cv::Mat::zeros(height, width, CV_32FC1);
		joint_ = cv::Mat::zeros(num_joint, 3, CV_32FC1);
		pose_ = cv::Mat::zeros(num_pose, 1, CV_32FC1);
		joint2d_ = Eigen::MatrixXd::Zero(num_joint, 3);
		RoI_depth_ = cv::Mat::zeros(RoI_leny, RoI_lenx, CV_16UC1);
		RoI_part_ = cv::Mat::zeros(RoI_leny, RoI_lenx, CV_8UC1);
		GroundTruthMatCenter_x = 0;
		GroundTruthMatCenter_y = 0;
		GroundTruthRoI_lenx = 0;
		GroundTruthRoI_leny = 0;
	}

	~Projection() {}

	void set_color_index(Model* model) {
		coloridx_ = Eigen::MatrixXi::Zero(model->weight_.rows(), 1);
		double maxV = 0;
		int index = 0;
		for (int i = 0; i < model->weight_.rows(); i++) {
			maxV = 0.0;
			for (int j = 0; j < model->weight_.cols(); j++) {
				if (maxV <= model->weight_(i, j)) {
					maxV = model->weight_(i, j);
					index = j;
				}
			}
			coloridx_(i, 0) = (index);
			//cout<<index<<" ";
		}

	}
	void projection0(double x3d, double y3d, double z3d,
		double& x2d, double & y2d, double& depth) {
		depth = -z3d;
		x2d = x3d * param_.focalx / z3d + param_.centerx;
		y2d = y3d* param_.focaly / z3d + param_.centery;
	}
	void get_joint_position(Model* model) {
		int num_joint = model->get_number_of_joint();
		double* position = nullptr;
		joint_.setTo(0.0);
		for (int i = 0; i < num_joint; i++) {
			position = model->get_joint_position(i);
			joint_.at<float>(i, 0) = position[0];
			joint_.at<float>(i, 1) = position[1];
			joint_.at<float>(i, 2) = position[2];
		}
	}
	void get_joint_pose(Model* model) {
		int num_joint = model->get_number_of_joint();
		Pose pose(0, 0, 0);
		pose_.setTo(0.0);
		bool* dof = nullptr;
		int cnt = -1;
		for (int i = 0; i < num_joint; i++) {
			pose = model->get_pose_of_joint(i);
			dof = model->get_dof(i);
			if (dof[0] == true) {
				cnt++;
				pose_.at<float>(cnt, 0) = pose.x;
			}
			if (dof[1] == true) {
				cnt++;
				pose_.at<float>(cnt, 0) = pose.y;
			}
			if (dof[2] == true) {
				cnt++;
				pose_.at<float>(cnt, 0) = pose.z;
			}
		}


		pose = model->get_global_position();
		cnt++;
		pose_.at<float>(cnt, 0) = pose.x;
		cnt++;
		pose_.at<float>(cnt, 0) = pose.y;
		cnt++;
		pose_.at<float>(cnt, 0) = pose.z;

	}
	void compute_current_orientation(Model* model) {
		int num_joint = model->get_number_of_joint();
		double* position = nullptr;
		double x2d = 0, y2d = 0, depth = 0;
		for (int i = 0; i < num_joint; i++) {
			position = model->get_joint_position(i);
			projection0(position[0], position[1], position[2],
				x2d, y2d, depth);
			joint2d_(i, 0) = x2d;
			joint2d_(i, 1) = y2d;
		}
		//cout<<joint2d_;
		int idxchild[NUM_JOINT] = { 5,2,3,4,0,6,7,8,0,10,11,12,0,
			14,15,16,0,18,19,20,0,22,0 };
		for (int i = 0; i < num_joint; i++) {
			if (idxchild[i] != 0) {
				x2d = joint2d_(idxchild[i], 0) - joint2d_(i, 0);
				y2d = joint2d_(idxchild[i], 1) - joint2d_(i, 1);
				joint2d_(i, 2) = atan2(y2d, x2d)*180.0 / 3.1415926;
				//cout<<x2d<<" "<<y2d<<" "<<joint2d_(i,2)<<endl;
			}
			else {
				joint2d_(i, 2) = joint2d_(i - 1, 2);
				//cout<<i<<endl;
			}

		}

	}

	void projection(double x3d, double y3d, double z3d,
		double& x2d, double & y2d, double& depth) {
		depth = -z3d;
		x2d = x3d * param_.focalx / z3d + param_.centerx;
		y2d = y3d* param_.focaly / z3d + param_.centery;

		if (x2d < 0) x2d = 0;
		if (x2d > depth_.cols - 1) x2d = depth_.cols - 1;
		if (y2d <0) y2d = 0;
		if (y2d > depth_.rows - 1) y2d = depth_.rows - 1;
	}

	void projection_when_calc(double x3d, double y3d, double z3d,
		double& x2d, double & y2d, double& depth) {
		depth = -z3d;
		x2d = x3d * param_.focalx / z3d + param_.centerx;
		y2d = y3d* param_.focaly / z3d + param_.centery;

		if (x2d < 0) x2d = 0;
		if (x2d > depth_.cols - 1) x2d = depth_.cols - 1;
		if (y2d <0) y2d = 0;
		if (y2d > depth_.rows - 1) y2d = depth_.rows - 1;
	}

	void select_points_from_one_mesh(Model* model, int idx) {
		int f0 = model->faces_(idx, 0);
		int f1 = model->faces_(idx, 1);
		int f2 = model->faces_(idx, 2);
		Pixel p0, p1, p2;
		p0.x = vertice2d_(f0, 0); p0.y = vertice2d_(f0, 1);
		p1.x = vertice2d_(f1, 0); p1.y = vertice2d_(f1, 1);
		p2.x = vertice2d_(f2, 0); p2.y = vertice2d_(f2, 1);

		int xmin = min(min(p0.x, p1.x), p2.x);
		int xmax = max(max(p0.x, p1.x), p2.x);
		int ymin = min(min(p0.y, p1.y), p2.y);
		int ymax = max(max(p0.y, p1.y), p2.y);
		Pixel pixel;
		for (int i = ymin; i <= ymax; i++) {
			for (int j = xmin; j <= xmax; j++) {
				int a = (p1.x - p0.x)*(i - p0.y) - (p1.y - p0.y)*(j - p0.x);      //用叉乘计算面积，从而判断该点是否在三角形内部
				int b = (p2.x - p1.x)*(i - p1.y) - (p2.y - p1.y)*(j - p1.x);
				int c = (p0.x - p2.x)*(i - p2.y) - (p0.y - p2.y)*(j - p2.x);
				pixel.x = j; pixel.y = i;
				if (a >= 0 && b >= 0 && c >= 0) { pixels_[idx].push_back(pixel); }
				if (a <= 0 && b <= 0 && c <= 0) { pixels_[idx].push_back(pixel); }
			}
		}
	}


	void choose_point(Model* model) {
		int num_faces = model->faces_.rows();
		pixels_.resize(num_faces);
		for (int i = 0; i < num_faces; i++) {
			pixels_[i].clear();
			//printf("%d\n",i);
			select_points_from_one_mesh(model, i);
		}
	}


	template<typename T>
	void remove_bound_pixels(cv::Mat& img, int pixels) {

		for (int i = 0; i < img.rows; ++i) {
			for (int j = 0; j < img.cols; ++j) {
				if (i <= pixels || j <= pixels || i >= img.rows - pixels || j >= img.cols - pixels) {
					//if(img.type() == CV_8UC1){
					img.at<T>(i, j) = 0;
					//}else{
					//   img.at<ushort>(i,j) = 0;
					//}

				}

			}
		}


	}

	void remove_arm(cv::Mat& depth, const cv::Mat& part) {
		for (int i = 0; i < depth.rows; ++i) {
			for (int j = 0; j < depth.cols; ++j)
			{
				if (part.at<uchar>(i, j) == 17)
				{
					depth.at<ushort>(i, j) = 0;
				}
			}

		}


	}

	void project_3d_to_2d_(Model* model, cv::Mat outputmat) {
		Eigen::MatrixXd vertice = model->vertices_update_;
		int num_vertice = model->vertices_update_.rows();
		vertice2d_ = Eigen::MatrixXd::Zero(num_vertice, 3);
		double x2d, y2d, depth;

		depth_.setTo(0);
		part_.setTo(0);

		for (int i = 0; i < model->vertices_update_.rows(); i++) {
			projection(model->vertices_update_(i, 0),
				model->vertices_update_(i, 1), model->vertices_update_(i, 2),
				x2d, y2d, depth);
			vertice2d_(i, 0) = x2d;
			vertice2d_(i, 1) = y2d;
			vertice2d_(i, 2) = depth;
		}
		//cout<<vertice2d_<<endl;
		choose_point(model);
		double x0 = 0, y0 = 0, x1 = 0, y1 = 0, x2 = 0, y2 = 0;
		double alpha0 = 0, alpha1 = 0;
		double depth0 = 0, depth1 = 0, depth2 = 0;
		double d0 = 0, d1 = 0, d2 = 0;

		for (int i = 0; i < pixels_.size(); i++) {
			int f0 = model->faces_(i, 0);
			int f1 = model->faces_(i, 1);
			int f2 = model->faces_(i, 2);
			x0 = vertice2d_(f1, 0) - vertice2d_(f0, 0);
			y0 = vertice2d_(f1, 1) - vertice2d_(f0, 1);
			x1 = vertice2d_(f2, 0) - vertice2d_(f0, 0);
			y1 = vertice2d_(f2, 1) - vertice2d_(f0, 1);
			depth0 = vertice2d_(f0, 2);
			depth1 = vertice2d_(f1, 2);
			depth2 = vertice2d_(f2, 2);
			for (int j = 0; j < pixels_[i].size(); j++) {          //这个循环针对choose_point()这个函数提取的，每个三角面内的，像素点进行的处理
				x2 = pixels_[i][j].x - vertice2d_(f0, 0);
				y2 = pixels_[i][j].y - vertice2d_(f0, 1);
				d0 = sqrt((x2*x2) + y2*y2);
				double fx1 = pixels_[i][j].x - vertice2d_(f1, 0);
				double fy1 = pixels_[i][j].y - vertice2d_(f1, 1);
				d1 = sqrt((fx1*fx1) + fy1*fy1);
				double fx2 = pixels_[i][j].x - vertice2d_(f2, 0);
				double fy2 = pixels_[i][j].y - vertice2d_(f2, 1);
				d2 = sqrt((fx2*fx2) + fy2*fy2);
				int f = 0;
				if (d0 <= d1&&d0 <= d2) {
					f = f0;
				}
				if (d1 <= d0&&d1 <= d2) {
					f = f1;
				}
				if (d2 <= d0&&d2 <= d1) {
					f = f2;           //也就是说f是距离该点最近的顶点
				}

				if (x1*y0 - x0*y1 != 0) {
					alpha1 = (x2*y0 - y2*x0) / (x1*y0 - x0*y1);
					alpha0 = (x2*y1 - x1*y2) / (x0*y1 - x1*y0);
				}
				else {
					alpha1 = 0;
					if (y0 != 0) {
						alpha0 = (y2) / (y0);
					}
					else { alpha0 = (x2) / (x0); }
				}


				if (pixels_[i][j].x >= 0 && pixels_[i][j].x< depth_.cols
					&& pixels_[i][j].y >= 0 && pixels_[i][j].y < depth_.rows) {
					depth = depth0 + alpha0*(depth1 - depth0) + alpha1*(depth2 - depth0);
					ushort v = depth_.at<ushort>(pixels_[i][j].y, pixels_[i][j].x);
					if (v != 0) {
						ushort k = (ushort)depth;
						depth_.at<ushort>(pixels_[i][j].y, pixels_[i][j].x) = min(v, (ushort)depth);
						if (v>(ushort) depth) {
							part_.at<uchar>(pixels_[i][j].y, pixels_[i][j].x) = (uchar)(map_label[coloridx_(f, 0)]);
						}
					}
					else {
						ushort k = (ushort)depth;
						depth_.at<ushort>(pixels_[i][j].y, pixels_[i][j].x) = (ushort)depth;
						part_.at<uchar>(pixels_[i][j].y, pixels_[i][j].x) = (uchar)(map_label[coloridx_(f, 0)]);
					}
				}
			}
		}

		remove_arm(depth_, part_);
		cv::Mat save_depth;
		cv::medianBlur(depth_, save_depth, 5);


		// if depth value is larger than 65536, it is error.
		double minPixelValue, maxPixelValue;
		cv::minMaxIdx(save_depth, &minPixelValue, &maxPixelValue);
		if (maxPixelValue >= 5000) {
			return;
		}

		//cv::Mat show_depth = cv::Mat::zeros(240, 320, CV_8UC1);
		cv::Mat show_depth = cv::Mat::zeros(424, 512, CV_8UC1);
		for (int i = 0; i < show_depth.rows; i++)
		{
			for (int j = 0; j < show_depth.cols; j++) {
				show_depth.at<uchar>(i, j) = save_depth.at<ushort>(i, j) % 255;
			}
		}
		cv::imshow("depth", show_depth);

		save_depth.copyTo(outputmat);
	}

	void project_3d_to_2d_when_calc(Model* model, cv::Mat outputmat) {
		Eigen::MatrixXd vertice = model->vertices_update_;
		int num_vertice = model->vertices_update_.rows();
		vertice2d_ = Eigen::MatrixXd::Zero(num_vertice, 3);
		double x2d, y2d, depth;

		RoI_depth_.setTo(0);
		RoI_part_.setTo(0);

		for (int i = 0; i < model->vertices_update_.rows(); i++) {
			projection(model->vertices_update_(i, 0),
				model->vertices_update_(i, 1), model->vertices_update_(i, 2),
				x2d, y2d, depth);
			vertice2d_(i, 0) = x2d;
			vertice2d_(i, 1) = y2d;
			vertice2d_(i, 2) = depth;
		}
		//cout<<vertice2d_<<endl;
		choose_point(model);
		double x0 = 0, y0 = 0, x1 = 0, y1 = 0, x2 = 0, y2 = 0;
		double alpha0 = 0, alpha1 = 0;
		double depth0 = 0, depth1 = 0, depth2 = 0;
		double d0 = 0, d1 = 0, d2 = 0;

		for (int i = 0; i < pixels_.size(); i++) {
			int f0 = model->faces_(i, 0);
			int f1 = model->faces_(i, 1);
			int f2 = model->faces_(i, 2);
			x0 = vertice2d_(f1, 0) - vertice2d_(f0, 0);
			y0 = vertice2d_(f1, 1) - vertice2d_(f0, 1);
			x1 = vertice2d_(f2, 0) - vertice2d_(f0, 0);
			y1 = vertice2d_(f2, 1) - vertice2d_(f0, 1);
			depth0 = vertice2d_(f0, 2);
			depth1 = vertice2d_(f1, 2);
			depth2 = vertice2d_(f2, 2);
			for (int j = 0; j < pixels_[i].size(); j++) {          //这个循环针对choose_point()这个函数提取的，每个三角面内的，像素点进行的处理
				x2 = pixels_[i][j].x - vertice2d_(f0, 0);
				y2 = pixels_[i][j].y - vertice2d_(f0, 1);
				d0 = sqrt((x2*x2) + y2*y2);
				double fx1 = pixels_[i][j].x - vertice2d_(f1, 0);
				double fy1 = pixels_[i][j].y - vertice2d_(f1, 1);
				d1 = sqrt((fx1*fx1) + fy1*fy1);
				double fx2 = pixels_[i][j].x - vertice2d_(f2, 0);
				double fy2 = pixels_[i][j].y - vertice2d_(f2, 1);
				d2 = sqrt((fx2*fx2) + fy2*fy2);
				int f = 0;
				if (d0 <= d1&&d0 <= d2) {
					f = f0;
				}
				if (d1 <= d0&&d1 <= d2) {
					f = f1;
				}
				if (d2 <= d0&&d2 <= d1) {
					f = f2;           //也就是说f是距离该点最近的顶点
				}

				if (x1*y0 - x0*y1 != 0) {
					alpha1 = (x2*y0 - y2*x0) / (x1*y0 - x0*y1);
					alpha0 = (x2*y1 - x1*y2) / (x0*y1 - x1*y0);
				}
				else {
					alpha1 = 0;
					if (y0 != 0) {
						alpha0 = (y2) / (y0);
					}
					else { alpha0 = (x2) / (x0); }
				}

				


				if (pixels_[i][j].x >= 0 && pixels_[i][j].x< depth_.cols
					&& pixels_[i][j].y >= 0 && pixels_[i][j].y < depth_.rows) {
					if (pixels_[i][j].x >= GroundTruthMatCenter_x - GroundTruthRoI_lenx / 2 && pixels_[i][j].x < GroundTruthMatCenter_x + GroundTruthRoI_lenx / 2
						&& pixels_[i][j].y >= GroundTruthMatCenter_y - GroundTruthRoI_leny / 2 && pixels_[i][j].y < GroundTruthMatCenter_y + GroundTruthRoI_leny / 2)
					{
						depth = depth0 + alpha0*(depth1 - depth0) + alpha1*(depth2 - depth0);
						ushort v = RoI_depth_.at<ushort>(pixels_[i][j].y - GroundTruthMatCenter_y + GroundTruthRoI_leny/2, pixels_[i][j].x- GroundTruthMatCenter_x + GroundTruthRoI_lenx / 2);
						if (v != 0) {
							ushort k = (ushort)depth;
							RoI_depth_.at<ushort>(pixels_[i][j].y - GroundTruthMatCenter_y + GroundTruthRoI_leny / 2, pixels_[i][j].x - GroundTruthMatCenter_x + GroundTruthRoI_lenx / 2) = min(v, (ushort)depth);
							if (v > (ushort) depth) {
								RoI_part_.at<uchar>(pixels_[i][j].y - GroundTruthMatCenter_y + GroundTruthRoI_leny / 2, pixels_[i][j].x - GroundTruthMatCenter_x + GroundTruthRoI_lenx / 2) = (uchar)(map_label[coloridx_(f, 0)]);
							}
						}
						else {
							ushort k = (ushort)depth;
							RoI_depth_.at<ushort>(pixels_[i][j].y - GroundTruthMatCenter_y + GroundTruthRoI_leny / 2, pixels_[i][j].x - GroundTruthMatCenter_x + GroundTruthRoI_lenx / 2) = (ushort)depth;
							RoI_part_.at<uchar>(pixels_[i][j].y - GroundTruthMatCenter_y + GroundTruthRoI_leny / 2, pixels_[i][j].x - GroundTruthMatCenter_x + GroundTruthRoI_lenx / 2) = (uchar)(map_label[coloridx_(f, 0)]);
						}
					}
				}
			}
		}

		remove_arm(RoI_depth_, RoI_part_);
		cv::Mat save_depth;
		cv::medianBlur(RoI_depth_, save_depth, 5);


		// if depth value is larger than 65536, it is error.
		double minPixelValue, maxPixelValue;
		cv::minMaxIdx(save_depth, &minPixelValue, &maxPixelValue);
		if (maxPixelValue >= 5000) {
			return;
		}

		//cv::Mat show_depth = cv::Mat::zeros(240, 320, CV_8UC1);
		cv::Mat show_depth = cv::Mat::zeros(GroundTruthRoI_leny, GroundTruthRoI_lenx, CV_8UC1);
		for (int i = 0; i < show_depth.rows; i++)
		{
			for (int j = 0; j < show_depth.cols; j++) {
				show_depth.at<uchar>(i, j) = save_depth.at<ushort>(i, j) % 255;
			}
		}
		cv::imshow("depth", show_depth);

		save_depth.copyTo(outputmat);
	}


	void SetGroundTruthMatInf(int centerx, int centery, int lenx, int leny)
	{
		this->GroundTruthMatCenter_x = centerx;
		this->GroundTruthMatCenter_y = centery;
		this->GroundTruthRoI_lenx = lenx;
		this->GroundTruthRoI_leny = leny;
	}
private:
	cv::Mat depth_;
	cv::Mat orient_;
	cv::Mat part_;
	cv::Mat joint_;
	cv::Mat pose_;
	cv::Mat RoI_depth_;
	cv::Mat RoI_part_;
	ProjectParam param_;
	vector<vector<Pixel> > pixels_;
	Eigen::MatrixXd vertice2d_;
	Eigen::MatrixXi coloridx_;
	Eigen::MatrixXd joint2d_;

	int GroundTruthMatCenter_x;
	int GroundTruthMatCenter_y;
	int GroundTruthRoI_lenx;
	int GroundTruthRoI_leny;
};

//static Projection *projection = new Projection(240, 320, 23, 28);

static Projection *projection = new Projection(424, 512, 23, 28,140,140);