/**
 * Author: rodrigo
 * 2015
 */
#include "Front.h"

Front::Front()
{
}

Front::~Front()
{
}

EdgePtr Front::getActiveEdge() const
{
	EdgePtr edge;
	return edge;
}

void Front::addEdges(const TrianglePtr &_triangle)
{
}

void Front::joinAndFix(const pair<int, TrianglePtr> &_data)
{
}

bool Front::inFront(PointNormal *_point)
{
	return frontPoints.find(_point) != frontPoints.end();
}
