/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include "Triangle.h"
#include "Pivoter.h"
#include <map>
#include <list>

class Front
{
public:
	Front();
	~Front();

	EdgePtr getActiveEdge();
	void addEdges(const TrianglePtr &_triangle);
	void joinAndFix(const std::pair<int, TrianglePtr> &_data, Pivoter &_pivoter);
	void setInactive(EdgePtr &_edge);

	inline bool inFront(const int _index)
	{
		return points.find(_index) != points.end();
	}

private:
	std::list<EdgePtr>::iterator isPresent(const EdgePtr &_edge);
	void addEdgePoints(std::list<EdgePtr>::iterator &_edge);
	void removeEdgePoints(EdgePtr &_edge);

	std::list<EdgePtr> front;
	std::list<EdgePtr>::iterator pos;
	std::map<int, std::map<EdgePtr, std::list<EdgePtr>::iterator> > points;
};
