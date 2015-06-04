/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include "Triangle.h"
#include <map>

class Front
{
public:
	Front();
	~Front();

	EdgePtr getActiveEdge() const;
	void addEdges(const TrianglePtr &_triangle);
	void joinAndFix(const pair<int, TrianglePtr> &_data);
	bool inFront(PointNormal *_point);

private:

	map<PointNormal *, int> frontPoints;
};
