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

	bool isPointInFront(const int _pointIndex) const;
	bool getActiveEdge(Edge **_edge) const;

private:
	map<int, bool> pointMap;
	map<Edge, bool> edgeMap;

	vector<Edge> edges;
	vector<Edge> activeEdges;
};
