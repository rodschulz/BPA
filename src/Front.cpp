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
			activeEdgeMap[newEdge] = true;
			edgeMap[newEdge] = true;
		}

		frontPointMap[_triangle.getVertex(i).second] = true;
		meshPointMap[_triangle.getVertex(i).second] = true;
	}
}

void Front::join(Edge *_edge, const PointXYZ *_point, const pair<int, Triangle> &_pivotData)
{
	// TODO check that the equal condition of edges is good enough

	// Check if point is already inside the mesh
	if (meshPointMap.find(_pivotData.first) == meshPointMap.end())
	{
		/**
		 * If the point isn't already in the mesh, then add it,
		 * add the corresponding edges and remove the given edge
		 */
		for (int i = 0; i < 3; i++)
		{
			Edge e = _pivotData.second.getEdge(i);
			if (e != *_edge)
			{
				edgeMap[e] = true;
				activeEdgeMap[e] = true;
			}
		}
		edgeMap.erase(*_edge);
		activeEdgeMap.erase(*_edge);
	}
	else
	{
		// Check if the point is already in the front
		if (frontPointMap.find(_pivotData.first) != frontPointMap.end())
		{
			// If it's not in the front then it can't be used, so the edge has to be marked as inactive
			_edge->setActive(false);
		}
		else
		{
			for (int i = 0; i < 3; i++)
			{
				Edge e = _pivotData.second.getEdge(i);
				if (e != *_edge)
				{
					edgeMap[e] = true;
					activeEdgeMap[e] = true;
				}
			}
			edgeMap.erase(*_edge);
			activeEdgeMap.erase(*_edge);
		}
	}
}

bool Front::inFront(const int _pointIndex) const
{
	return (frontPointMap.find(_pointIndex) != frontPointMap.end());
}

bool Front::getActiveEdge(Edge **_edge)
{
	if (activeEdgeMap.empty())
	{
		*_edge = NULL;
		return false;
	}
	else
	{
		map<Edge, bool>::iterator it = activeEdgeMap.begin();

		// If not active, then remove and get another
		while (!it->first.isActive() && !activeEdgeMap.empty())
		{
			activeEdgeMap.erase(it->first);
			it = activeEdgeMap.begin();
		}

		if (activeEdgeMap.empty())
		{
			*_edge = NULL;
			return false;
		}
		else
		{
			*_edge = (Edge *) &it->first;
			return true;
		}
	}
}
