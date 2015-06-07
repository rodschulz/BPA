/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include "Triangle.h"
#include "Pivoter.h"
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
	void joinAndFix(const pair<int, TrianglePtr> &_data, Pivoter &_pivoter);
	bool inFront(PointNormal *_point);

private:
	list<EdgePtr>::iterator isPresent(const EdgePtr &_edge);
	bool remove(PointNormal *_point);

//	inline void addPoint(PointData &_point, EdgePtr &_edge)
//	{
//		if (points.find(_point.first) == points.end())
//			points[_point.first] = vector<EdgePtr>();
//		points[_point.first].push_back(_edge);
//	}

	list<EdgePtr> front;
	list<EdgePtr>::iterator pos;
//	map<PointNormal *, vector<EdgePtr>> points;
	map<PointNormal *, int> frontPoints;
};
