/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include "Triangle.h"
#include <map>
#include <list>

using namespace std;

class Front
{
public:
	Front();
	~Front();

	EdgePtr getActiveEdge();
	void addEdges(const TrianglePtr &_triangle);
	void joinAndFix(const pair<int, TrianglePtr> &_data);
	bool inFront(PointNormal *_point);

private:
	list<EdgePtr> front;
	list<EdgePtr>::iterator pos;
	map<PointNormal *, int> frontPoints;
};
