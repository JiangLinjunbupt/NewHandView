#pragma once
#include <iostream>
#include <opencv2\opencv.hpp>
using namespace std;
//using namespace cv;

#define PI 3.1415927

#define TrianglePatchSize 3
struct PatchType {
	cv::Vec3i v_idx;
	cv::Point3f normal;
	PatchType(int va, int vb, int vc, float nx = 0, float ny = 0, float nz = 0) {
		v_idx = cv::Vec3i(va, vb, vc);
		normal = cv::Point3f(nx, ny, nz);
	}
	PatchType() {}
};

struct PointType {
	cv::Point3f position;
	cv::Vec3f color;//b,g,r
	PointType(float x, float y, float z, float r, float g, float b) :position(x, y, z), color(b, g, r) {
	}
	PointType() {}
};

class Transformation
{
private:
	cv::Mat R; //3*3
	cv::Mat T; //3*1
public:
	Transformation() {
		R = cv::Mat::eye(3, 3, CV_64FC1);
		T = cv::Mat::zeros(3, 1, CV_64FC1);
	};
	Transformation(const cv::Mat& rotation, const cv::Mat& translation) {
		rotation.convertTo(R, CV_64FC1);
		translation.convertTo(T, CV_64FC1);
	};
	cv::Vec3d transform(const cv::Vec3d& vec)const {
		cv::Mat v = (cv::Mat_<double>(3, 1) <<
			vec[0], vec[1], vec[2]
			);
		v = R*v + T;
		cv::Vec3d new_vec(
			v.at<double>(0, 0),
			v.at<double>(1, 0),
			v.at<double>(2, 0)
		);
		return new_vec;
	}
	cv::Point3f transform(const cv::Point3f& pt)const {
		cv::Vec3d vec(pt.x, pt.y, pt.z);
		cv::Vec3d rs = transform(vec);
		return cv::Point3f((float)rs[0], (float)rs[1], (float)rs[2]);
	}
	Transformation inv() {
		cv::Mat iR = R.inv();
		cv::Mat iT = -iR*T;
		return Transformation(iR, iT);
	}
	void print() {
		cout << "Rotation:" << endl;
		for (int i = 0; i < R.rows; i++)
		{
			for (int j = 0; j < R.cols; j++)
				cout << R.at<double>(i, j) << " ";
			cout << endl;

		}
		cout << "Translation:" << endl;
		for (int i = 0; i < T.rows; i++)
		{
			for (int j = 0; j < T.cols; j++)
				cout << T.at<double>(i, j) << " ";
			cout << endl;

		}

	}
	~Transformation() {};
};