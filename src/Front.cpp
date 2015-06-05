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
		for (int i = 0; i < 2; i++)
		{
			EdgePtr edge = _data.second->getEdge(i);
			front.insert(pos, edge);
			cout << "\tEdge added: " << *edge << "\n";
		}

		// Add new point to the 'in front' map
		PointData data = _data.second->getVertex(1);
		frontPoints[data.first] = data.second;

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
		PointNormal *point = _pivoter.getPoint(_data.first);
		if (inFront(point))
		{
			/**
			 * Point in front, so orientation must be check, and join and glue must be done
			 */
			int added = 0;

			for (int i = 0; i < 2; i++)
			{
				EdgePtr edge = _data.second->getEdge(i);
				list<EdgePtr>::iterator it;
				if ((it = isPresent(edge)) != front.end())
				{
					// Remove the 'coincident' edge
					cout << "\tEdge removed: " << **it << "\n";
					front.erase(it);
				}
				else
				{
					front.insert(pos, edge);
					added--;
					cout << "\tEdge added: " << *edge << "\n";
				}
			}

			// Remove the old edge
			cout << "\tEdge removed: " << **pos << "\n";
			front.erase(pos);

			// Move iterator to the first added new edge
			if (added > 0)
				advance(pos, added);
			else
				pos = front.begin();

			// Delete point from the front
			frontPoints.erase(point);
			cout << "\tPoint removed from front: " << _data.first << "\n";
		}
		else
		{
			/**
			 * The point is not part of any front edge, hence is an internal
			 * point, so this edge can't be done. In consequence this a boundary
			 */
			(*pos)->setActive(false);
			cout << "Edge marked as boundary: " << **pos << "\n";
		}
	}
}

bool Front::inFront(PointNormal *_point)
{
	return frontPoints.find(_point) != frontPoints.end();
}

list<EdgePtr>::iterator Front::isPresent(const EdgePtr &_edge)
{
	int vertex0 = _edge->getVertex(0).second;
	int vertex1 = _edge->getVertex(1).second;

	for (list<EdgePtr>::iterator it = front.begin(); it != front.end(); it++)
	{
		int v0 = (*it)->getVertex(0).second;
		int v1 = (*it)->getVertex(1).second;
		if ((v0 == vertex1 && v1 == vertex0) || (v0 == vertex0 && v1 == vertex1))
			return it;
	}

	return front.end();
}
