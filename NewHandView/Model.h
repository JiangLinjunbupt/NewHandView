#pragma once
#include <Eigen/Dense>
#include "BVH.h"
#include "HandStruct.h"
// the hand model is designed according to the mathematic formulation
// from Microsoft Research paper

// Learning an Efficent Model of Hand Shape Variation from Depth Images
// Sameh Khamis, Jamie Shotton, Andrew Fitzgibbon
// CVPR 2015



#define NUM_JOINT (23)
class Model {
public:
	Model();
	Model(char* file);
	~Model();
	void set_dof();
	void set_upper_lower_bound();
	void forward_kinematic();
	void compute_local_coordinate();
	void compute_init_global_position();
	void compute_parent_child_transform();
	void compute_local_inverse();

	void set_one_rotation(Pose pose, int index);
	void set_one_trans(float tansZ, int index);
	void set_rotation(Pose* pose);
	void set_global_position(Pose global_position);
	void set_global_position_center(Pose global_position);
	void set_joint_scale(float scale, int index);

	void transform_matrix(Pose pose, Eigen::MatrixXd& rot);
	void compute_global();
	void compute_mesh();

	void init();
	void load_faces(char* file);
	void load_vertices(char* file);
	void load_weight(char* file);


	Pose get_global_position() { return global_position_; }
	Pose get_upper_of_angle(int idx) { return bvh_.GetJoint(idx)->upper; }
	Pose get_lower_of_angle(int idx) { return bvh_.GetJoint(idx)->lower; }

	int  get_number_of_joint() { return bvh_.GetNumJoint(); };
	int  get_parent_of_joint(int idx) { return bvh_.GetJoint(idx)->parent->index; }
	bool* get_dof(int idx) { return bvh_.GetJoint(idx)->dof; }
	Pose get_pose_of_joint(int idx) { return bvh_.GetJoint(idx)->pose; }
	double* get_joint_position(int idx) { return bvh_.GetJoint(idx)->global_position; }


public:

	Eigen::MatrixXi faces_;
	Eigen::MatrixXd vertices_;
	Eigen::MatrixXd weight_;
	Eigen::MatrixXd vertices_update_;


private:
	BVH bvh_;
	Pose global_position_;
	Pose global_position_center_;

	int num_faces_;
	int num_vertices_;
	int num_weight_;
};

static Model* model = nullptr;