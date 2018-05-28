#pragma once
#include <vector>
#include <opencv2\opencv.hpp>
#include "Common.h"
#include "GeometryDataStructure.h"
using namespace std;
using namespace cv;

class SubSurfaces {
public:
	void initModel() {
		modeling(ori_vertices, ori_patches);
	}
	void initModel(const vector<PointType>& vertices, const vector<PatchType>& patches) {
		ori_vertices.assign(vertices.begin(), vertices.end());
		ori_patches.assign(patches.begin(), patches.end());
		initModel();
	}
	void subdivide() {
		vector<PointType> vertices;
		const int vp_num = vPool.size();
		const int ep_num = fPool.size()*TrianglePatchSize / 2;
		vertices.resize(vp_num + ep_num * 2);
		int actualSize = 0;
		ePool.generateNewEdgePoints(vPool, vertices, actualSize);
#ifdef DEBUG
		if (actualSize != ep_num)
		{
			cout << "Error: actual number of edge points is not consistent with precomputeted one" << endl;
			return;
		}
#endif // DEBUG
		vector<set<int>> vertexNeighbors;
		ePool.getVertexNeighbors(vertexNeighbors);

		vPool.generateNewVertexPoints(vertexNeighbors, vertices, actualSize);
#ifdef DEBUG
		if (actualSize != vertices.size())
		{
			cout << "Error: actual number of new points is not consistent with precomputeted one" << endl;
			return;
		}
#endif // DEBUG
		vertices.resize(actualSize);
		vector<PatchType> patches;
		fPool.generateNewPatches(vPool, ePool, patches);
		modeling(vertices, patches);
	}

	void generateDisplayModel(vector<PointType>& vertices, vector<PatchType>& patches, vector<PatchType>& visible_patches,set<int>& visible_vertices) {
		vPool.getVertices(vertices);
		fPool.getPatches(patches);
		visible_patches.clear();
		visible_vertices.clear();
		for (vector<PatchType>::iterator it = patches.begin(); it != patches.end(); it++)
		{
			cv::Vec3f va = vertices[it->v_idx[1]].position - vertices[it->v_idx[0]].position;
			cv::Vec3f vb = vertices[it->v_idx[2]].position - vertices[it->v_idx[1]].position;
			it->normal = cv::normalize(va.cross(vb));

			if (-it->normal.z < 0)
			{
				PatchType visible;
				visible.v_idx = it->v_idx;
				visible.normal = it->normal;
				visible_patches.push_back(visible);
				visible_vertices.insert(it->v_idx[0]);
				visible_vertices.insert(it->v_idx[1]);
				visible_vertices.insert(it->v_idx[2]);
				
			}
#ifdef DEBUG
			if (it->normal.dot(it->normal) < 0.001)
			{
				cout << "Error: normal of one patch is zero" << endl;
				return;
			}
#endif // DEBUG
		}
	}
private:
	vector<PointType> ori_vertices;
	vector<PatchType> ori_patches;
	VertexPool vPool;
	EdgePool ePool;
	FacePool fPool;
	void modeling(const vector<PointType>& vertices, const vector<PatchType>& patches) {
		vPool.init(vertices.size());
		for (int i = 0; i < vertices.size(); i++)
			vPool.insert(vertices[i]);
		ePool.init(vertices.size());
		fPool.init(patches.size());
		for (vector<PatchType>::const_iterator it = patches.begin(); it != patches.end(); it++)
		{
			fPool.insert(*it);
			ePool.insert(it->v_idx[0], it->v_idx[1], it->v_idx[2]);
			ePool.insert(it->v_idx[1], it->v_idx[2], it->v_idx[0]);
			ePool.insert(it->v_idx[2], it->v_idx[0], it->v_idx[1]);
		}
		//if (!ePool.checkIntegrity())
		//{
		//	cout << " the model is not a closed model!" << endl;
		//	return;
		//}
	}
};


static SubSurfaces ssModel;
static int ssLevel;

namespace SS
{
	enum SubLevel
	{
		INCREASE,
		DECREASE
	};
	vector<PointType> disVertices;
	vector<PatchType> disPatches;
	vector<PatchType> visiblePatches;
	set<int> visibleVertices;
	void initModel(const vector<PointType>& vertices, const vector<PatchType>& patches) {
		ssModel.initModel(vertices, patches);
		ssModel.generateDisplayModel(disVertices, disPatches, visiblePatches,visibleVertices);
		ssLevel = 0;
		/*cout << "Initializatin:" << endl;
		cout << "Number of vertices: " << disVertices.size() << endl;
		cout << "Number of patches: " << disPatches.size() << endl;
		cout << endl;*/
	}
	void subdivide() {
		int64 time_start = cv::getTickCount();
		SubLevel level = INCREASE;
		switch (level)
		{
		case INCREASE:
			ssLevel++;
			ssModel.subdivide();
			break;
		case DECREASE:
			if (ssLevel == 0)
				break;
			ssLevel--;
			ssModel.initModel();
			for (int i = 0; i < ssLevel; i++)
				ssModel.subdivide();
			break;
		default:
			break;
		}
		int64 time_end = cv::getTickCount();
		ssModel.generateDisplayModel(disVertices, disPatches, visiblePatches,visibleVertices);
		/*cout << "Subdivision Level: " << ssLevel << endl;
		cout << "Subdivision Time: " << (time_end - time_start) / cv::getTickFrequency() << " s" << endl;
		cout << "Number of vertices: " << disVertices.size() << endl;
		cout << "Number of patches: " << disPatches.size() << endl;
		cout << endl;*/
	}


	void SubdivisionTheHand(Model* model, int subdivisionLevel)
	{
		vector<PointType> _vertices;
		vector<PatchType> _patches;

		vector<float> _weight;

		for (int i = 0; i < model->weight_.rows(); i ++ )
		{
			double t = -1e10;
			int idx = 0;
			for (int j = 0; j < model->weight_.cols(); j++)
			{
				if (t < model->weight_(i, j)) {
					t = model->weight_(i, j);
					idx = j;                        //以weight的最大值对应的关节点作为该顶点所属的部分
				}
			}

			if (idx == 0 || idx ==21 || idx == 22)               //0和21和22分别代表手掌和手腕，前臂部分
			{
				_weight.push_back(1.0);
			
			}
			else
			{
				_weight.push_back(5.0);
				
			}
		}

		for (int i = 0; i < model->vertices_update_.rows(); i++)
		{
			_vertices.push_back(PointType(float(model->vertices_update_(i, 0)), float(model->vertices_update_(i, 1)), float(model->vertices_update_(i, 2)), 1, 1, 1,_weight[i]));
		}
		for (int i = 0; i < model->faces_.rows(); i++)
		{
			_patches.push_back(PatchType(model->faces_(i, 0), model->faces_(i, 1), model->faces_(i, 2)));
		}

		SS::initModel(_vertices, _patches);

		for (int i = 0; i < subdivisionLevel; i++)        //这里不要大于5次，因为5次之后的面可能特别多，变量空间不够用
		{
			SS::subdivide();
		}
	}
}

