/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <map>
#include <vector>
#include <pcl/common/common.h>
#include "Triangle.h"

using namespace std;
using namespace pcl;

class Front
{
public:
	Front();
	~Front();

	void addEdges(const Triangle &_triangle);
	void join(Edge *_edge, const PointXYZ *_point, const pair<int, Triangle> &_pivotData);

	bool inFront(const int _pointIndex) const;
	bool InMesh(const int _pointIndex) const;
	bool getActiveEdge(Edge **_edge);

private:
	map<int, bool> frontPointMap;
	map<int, bool> meshPointMap;
	map<Edge, bool> edgeMap;
	map<Edge, bool> activeEdgeMap;
};
