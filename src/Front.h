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
	void setInactive(EdgePtr &_edge);

	inline bool inFront(const int _index)
	{
		return points.find(_index) != points.end();
	}

private:
	list<EdgePtr>::iterator isPresent(const EdgePtr &_edge);
	void addEdgePoints(list<EdgePtr>::iterator &_edge);
	void removeEdgePoints(EdgePtr &_edge);

	list<EdgePtr> front;
	list<EdgePtr>::iterator pos;
	//map<int, map<EdgePtr, bool> > points;
	map<int, map<EdgePtr, list<EdgePtr>::iterator> > points;
};
