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
		// Since triangles were created in the correct sequence, then edges should be correctly oriented
		front.push_back(_triangle->getEdge(i));
		cout << "\tEdge added: " << *front.back() << "\n";

		PointData data = front.back()->getVertex(0);
		frontPoints[data.first] = data.second;
	}
}

void Front::joinAndFix(const pair<int, TrianglePtr> &_data, Pivoter &_pivoter)
{
	if (!_pivoter.isUsed(_data.first))
	{
		/**
		 * This is the easy case, the new point has not been used
		 */

		// Add new edges
		EdgePtr edge1 = _data.second->getEdge(1);
		EdgePtr edge2 = _data.second->getEdge(2);
		front.insert(pos, edge1);
		front.insert(pos, edge2);

		cout << "\tEdge added: " << *edge1 << "\n";
		cout << "\tEdge added: " << *edge2 << "\n";

		// Remove replaced edge
		cout << "\tEdge removed: " << **pos << "\n";
		front.erase(pos);

		// Move iterator to the first added new edge
		advance(pos, -2);

		// Finally mark the point as used
		_pivoter.setUsed(_data.first);
	}
	else
	{
		cout << "*** Glue operation not implemented yet!\n";
	}
}

bool Front::inFront(PointNormal *_point)
{
	return frontPoints.find(_point) != frontPoints.end();
}
