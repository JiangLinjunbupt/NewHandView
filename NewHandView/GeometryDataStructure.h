#pragma once
#include <vector>
#include <list>
#include <map>
#include <algorithm>
#include <opencv2\opencv.hpp>
#include "Common.h"

using namespace std;
//using namespace cv;

#define NOT_VALID -1

struct Vertex {
	PointType pt;
	int vp_index;
};

class VertexPool
{
private:
	vector<Vertex> vertices;
	int actualSize;
public:
	VertexPool() {
		actualSize = 0;
	}
	~VertexPool() {}
	void init(int largestSize) {
		vertices.resize(largestSize);
		actualSize = 0;
	}
	int insert(PointType v) {
#ifdef DEBUG
		if (actualSize >= vertices.size())
		{
			cout << "Error: Out of VertexPool size, can't insert more new vertex!" << endl;
			return -1;
		}
#endif // DEBUG
		Vertex vt;
		vt.pt = v;
		vt.vp_index = NOT_VALID;
		vertices[actualSize++] = vt;
		return actualSize - 1;
	}
	int size()const {
		return actualSize;
	}
	const PointType& operator[](int nIndex)const {
		return vertices[nIndex].pt;
	};
	void getVertices(vector<PointType>& pts) {
		pts.resize(vertices.size());
		for (int i = 0; i < pts.size(); i++)
			pts[i] = vertices[i].pt;
	}
	void generateNewVertexPoints(const vector<set<int>>& neighbors, vector<PointType>& new_vertices, int& actualSize) {
		for (int i = 0; i < vertices.size(); i++)
		{
#ifdef DEBUG
			if (actualSize >= new_vertices.size())
			{
				cout << "Error: out of vector size in generateNewVertexPoints!" << endl;
				return;
			}
#endif // DEBUG
			Vertex& v = vertices[i];
			PointType vp;
			vp.position = cv::Point3f(0, 0, 0);
			const set<int>& nb = neighbors[i];
			if (nb.size() > 2)
			{
				for (set<int>::const_iterator it = nb.begin(); it != nb.end(); it++)
				{
					vp.position += vertices[*it].pt.position;
				}
				vp.position = vp.position / (float)nb.size();
				static const float f = 3.0 / 8.0;
				float s = pow(f + 0.25*cos(2 * PI / nb.size()), 2.0) + f;
				vp.position = vp.position*(1 - s) + v.pt.position*s;
				vp.color = v.pt.color;
				vp.weight = v.pt.weight;
				new_vertices[actualSize++] = vp;
				v.vp_index = actualSize - 1;
			}
			else
			{
				for (set<int>::const_iterator it = nb.begin(); it != nb.end(); it++)
				{
					vp.position += vertices[*it].pt.position;
				}
				vp.position = vp.position / 8.0f;
				vp.position = vp.position + v.pt.position*0.75;
				vp.color = v.pt.color;
				vp.weight = v.pt.weight;
				new_vertices[actualSize++] = vp;
				v.vp_index = actualSize - 1;
			}
		}
	}
	int getVertexPointIndex(int nIndex)const {
#ifdef DEBUG
		if (vertices[nIndex].vp_index == NOT_VALID)
		{
			cout << "Error: VertexPoint hasn't been calculated!" << endl;
			return -1;
		}
#endif // DEBUG
		return vertices[nIndex].vp_index;
	}
};

struct HalfEdge {
	int ed;
	int next_pt;
	int ep_index;
};

class EdgePool
{
public:
	typedef vector<list<HalfEdge>> EdgeMap;
private:
	EdgeMap edges;
	typedef list<HalfEdge>::iterator PtrHalfEdge;
	typedef list<HalfEdge>::const_iterator PtrConstHalfEdge;
	bool findEdge(int st, int ed)const {
		PtrConstHalfEdge ptr;
		return findEdge(st, ed, ptr);
	}
	bool findEdge(int st, int ed, PtrHalfEdge& ptr)
	{
		for (ptr = edges[st].begin(); ptr != edges[st].end(); ptr++)
			if (ptr->ed == ed)
				return true;
		ptr = edges[st].end();
		return false;
	}
	bool findEdge(int st, int ed, PtrConstHalfEdge& ptr)const
	{
		for (ptr = edges[st].begin(); ptr != edges[st].end(); ptr++)
			if (ptr->ed == ed)
				return true;
		ptr = edges[st].end();
		return false;
	}
public:
	EdgePool() {}
	~EdgePool() {}
	void init(const int num_vertex) {
		edges.clear();
		edges.resize(num_vertex);
	}
	void insert(const int st, const int ed, const int next_pt)
	{
#ifdef DEBUG
		if (findEdge(st, ed))
		{
			cout << "Error: this halfedge already exists in EdgePool!" << endl;
			return;
		}
#endif // DEBUG
		HalfEdge e;
		e.ed = ed;
		e.next_pt = next_pt;
		e.ep_index = NOT_VALID;
		edges[st].push_back(e);
	}
	void generateNewEdgePoints(const VertexPool& vPool, vector<PointType>& vertices, int& actualSize) {
		for (int st = 0; st < edges.size(); st++)
			for (PtrHalfEdge it = edges[st].begin(); it != edges[st].end(); it++)
			{
				PointType ep;
				PtrHalfEdge rv_he;
				if (findEdge(it->ed, st, rv_he))
				{
					if (st < it->ed)
					{
#ifdef DEBUG
						if (actualSize >= vertices.size())
						{
							cout << "Error: out of vector size in generateEdgePoints!" << endl;
							return;
						}
#endif // DEBUG

#ifdef DEBUG
						if (!findEdge(it->ed, st))
						{
							cout << "Error: can't find the reverse halfedge!" << endl;
							return;
						}
#endif // DEBUG
						ep.position = 3 * vPool[st].position + 3 * vPool[it->ed].position;
						ep.position += vPool[it->next_pt].position + vPool[rv_he->next_pt].position;
						ep.position /= 8.0;
						ep.color = vPool[st].color;
						ep.weight = (vPool[st].weight + vPool[it->ed].weight + vPool[it->next_pt].weight + vPool[rv_he->next_pt].weight) / 4.0;

						vertices[actualSize++] = ep;
						it->ep_index = rv_he->ep_index = actualSize - 1;

					}
				}
				else
				{
					ep.position = 0.5 * vPool[st].position + 0.5 * vPool[it->ed].position;
					ep.color = vPool[st].color;
					ep.weight = (vPool[st].weight + vPool[it->ed].weight)/2.0;
					vertices[actualSize++] = ep;
					it->ep_index = actualSize - 1;
				}
			}
	}
	void getVertexNeighbors(vector<set<int>>& neighbors)const {
		neighbors.clear();
		neighbors.resize(edges.size());
		for (int st = 0; st < neighbors.size(); st++)
			for (PtrConstHalfEdge it = edges[st].begin(); it != edges[st].end(); it++)
			{
				neighbors[st].insert(it->ed);
				neighbors[st].insert(it->next_pt);
			}
	}
	int getEdgePointIndex(int st, int ed) const {
		PtrConstHalfEdge he;
		findEdge(st, ed, he);
#ifdef DEBUG
		if (!findEdge(st, ed, he))
		{
			cout << "Error: no this halfedge in getEdgePointIndex!" << endl;
			return -1;
		}
		if (he->ep_index == NOT_VALID)
		{
			cout << "Error: edgePoint hasn't been calculated!" << endl;
			return -1;
		}
#endif // DEBUG
		return he->ep_index;
	}
	bool checkIntegrity()const {
		for (int st = 0; st < edges.size(); st++)
		{
			const list<HalfEdge>& neighbors = edges[st];
			for (list<HalfEdge>::const_iterator it = neighbors.begin(); it != neighbors.end(); it++)
				if (!findEdge(it->ed, st))
					return false;
		}
		return true;
	}
};

typedef PatchType Face;

class FacePool {
private:
	vector<Face> faces;
	int actualSize;
public:
	FacePool() {
		actualSize = 0;
	}
	~FacePool() {}
	void init(int largestSize) {
		faces.resize(largestSize);
		actualSize = 0;
	}
	int insert(const PatchType& patch) {
#ifdef DEBUG
		if (actualSize >= faces.size())
		{
			cout << "Error: Out of FacePool size, can't insert more new face!" << endl;
			return -1;
		}
#endif // DEBUG
		Face f = patch;
		faces[actualSize++] = f;
		return actualSize - 1;
	}
	void getPatches(vector<PatchType>& patches) {
		patches.resize(faces.size());
		for (int i = 0; i < patches.size(); i++)
			patches[i] = faces[i];
	}
	int size() const {
		return actualSize;
	}
	void generateNewPatches(const VertexPool& vPool, const EdgePool& ePool, vector<PatchType>& patches)const {
		patches.resize(faces.size() * 4);
		int cnt = 0;
		for (vector<Face>::const_iterator it = faces.begin(); it != faces.end(); it++)
		{
			cv::Vec3i edgePoints;
			edgePoints[0] = ePool.getEdgePointIndex(it->v_idx[0], it->v_idx[1]);
			edgePoints[1] = ePool.getEdgePointIndex(it->v_idx[1], it->v_idx[2]);
			edgePoints[2] = ePool.getEdgePointIndex(it->v_idx[2], it->v_idx[0]);

			patches[cnt].v_idx[0] = vPool.getVertexPointIndex(it->v_idx[0]);
			patches[cnt].v_idx[1] = edgePoints[0];
			patches[cnt++].v_idx[2] = edgePoints[2];

			patches[cnt].v_idx[0] = vPool.getVertexPointIndex(it->v_idx[1]);
			patches[cnt].v_idx[1] = edgePoints[1];
			patches[cnt++].v_idx[2] = edgePoints[0];

			patches[cnt].v_idx[0] = vPool.getVertexPointIndex(it->v_idx[2]);
			patches[cnt].v_idx[1] = edgePoints[2];
			patches[cnt++].v_idx[2] = edgePoints[1];

			patches[cnt].v_idx[0] = edgePoints[0];
			patches[cnt].v_idx[1] = edgePoints[1];
			patches[cnt++].v_idx[2] = edgePoints[2];
		}
	}
};