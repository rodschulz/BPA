/**
 * Author: rodrigo
 * 2015
 */
#include "Front.h"

Front::Front()
{
	front.clear();
	pos = front.begin();
}

Front::~Front()
{
}

EdgePtr Front::getActiveEdge()
{
	EdgePtr edge;

	if (!front.empty())
	{
		bool firstLoop = true;
		for (list<EdgePtr>::iterator it = pos;; it++)
		{
			if (it == front.end())
				it = front.begin();
			if (!firstLoop && it == pos)
				break;

			if ((*it)->isActive())
			{
				pos = it;
				edge = *it;
				break;
			}

			firstLoop = false;
		}
	}

	return edge;
}

void Front::addEdges(const TrianglePtr &_triangle)
{
	for (int i = 0; i < 3; i++)
	{
		front.push_back(_triangle->getEdge(i));

		PointData data = front.back()->getVertex(0);
		frontPoints[data.first] = data.second;
		data = front.back()->getVertex(1);
		frontPoints[data.first] = data.second;
	}
}

void Front::joinAndFix(const pair<int, TrianglePtr> &_data)
{
}

bool Front::inFront(PointNormal *_point)
{
	return frontPoints.find(_point) != frontPoints.end();
}
