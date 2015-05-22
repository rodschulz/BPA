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

void Front::addEdges(const Triangle &_triangle)
{
	for (int i = 0; i < 3; i++)
	{
		Edge newEdge = _triangle.getEdge(i);
		if (edgeMap.find(newEdge) == edgeMap.end())
		{
			activeEdges.push_back(newEdge);
			edgeMap[newEdge] = true;
		}

		pointMap[_triangle.getVertex(i).second] = true;
	}
}

bool Front::isPointInFront(const int _pointIndex) const
{
	return (pointMap.find(_pointIndex) != pointMap.end());
}

bool Front::getActiveEdge(Edge **_edge) const
{
	if (activeEdges.empty())
	{
		*_edge = NULL;
		return false;
	}
	else
	{
		*_edge = (Edge *) &activeEdges[0];
		return true;
	}
}
