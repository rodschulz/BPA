/**
 * Author: rodrigo
 * 2015
 */
#include "Front.h"
#include "Config.h"

Front::Front()
{
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
		for (std::list<EdgePtr>::iterator it = pos;; it++)
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
	DebugLevel debug = Config::getDebugLevel();
	for (int i = 0; i < 3; i++)
	{
		// Since triangles were created in the correct sequence, then edges should be correctly oriented
		front.push_back(_triangle->getEdge(i));
		addEdgePoints(--front.end());

		if (debug >= MEDIUM)
			std::cout << "\tEdge added: " << *front.back() << "\n";
	}
}

void Front::joinAndFix(const std::pair<int, TrianglePtr> &_data, Pivoter &_pivoter)
{
	DebugLevel debug = Config::getDebugLevel();

	if (!_pivoter.isUsed(_data.first))
	{
		/**
		 * This is the easy case, the new point has not been used
		 */
		// Add new edges
		for (int i = 0; i < 2; i++)
		{
			EdgePtr edge = _data.second->getEdge(i);
			std::list<EdgePtr>::iterator insertionPlace = front.insert(pos, edge);
			addEdgePoints(insertionPlace);

			if (debug >= MEDIUM)
				std::cout << "\tEdge added: " << *edge << "\n";
		}

		// Remove replaced edge
		if (debug >= MEDIUM)
			std::cout << "\tEdge removed: " << **pos << "\n";

		removeEdgePoints(*pos);
		front.erase(pos);

		// Move iterator to the first added new edge
		advance(pos, -2);

		// Mark the point as used
		_pivoter.setUsed(_data.first);
	}
	else
	{
		if (inFront(_data.first))
		{
			/**
			 * Point in front, so orientation must be check, and join and glue must be done
			 */
			int added = 0;
			for (int i = 0; i < 2; i++)
			{
				EdgePtr edge = _data.second->getEdge(i);
				std::list<EdgePtr>::iterator it;
				if ((it = isPresent(edge)) != front.end())
				{
					// Remove the 'coincident' edge
					if (debug >= MEDIUM)
						std::cout << "\tEdge removed: " << **it << "\n";

					removeEdgePoints(*it);
					front.erase(it);
				}
				else
				{
					std::list<EdgePtr>::iterator insertionPlace = front.insert(pos, edge);
					addEdgePoints(insertionPlace);
					added--;

					if (debug >= MEDIUM)
						std::cout << "\tEdge added: " << *edge << "\n";
				}
			}

			// Remove pivoting edge
			if (debug >= MEDIUM)
				std::cout << "\tEdge removed: " << **pos << "\n";

			removeEdgePoints(*pos);
			front.erase(pos);

			// Move iterator to the first added new edge
			if (added < 0)
				advance(pos, added);
			else
				// Reset the position
				pos = front.begin();
		}
		else
		{
			/**
			 * The point is not part of any front edge, hence is an internal
			 * point, so this edge can't be done. In consequence this a boundary
			 */
			setInactive(*pos);
			if (debug >= LOW)
				std::cout << "Edge marked as boundary: " << **pos << "\n";
		}
	}
}

void Front::setInactive(EdgePtr &_edge)
{
	_edge->setActive(false);
	removeEdgePoints(_edge);

	if (front.begin() == pos)
	{
		front.erase(pos);
		pos = front.begin();
	}
	else
	{
		front.erase(pos);
		advance(pos, -1);
	}
}

std::list<EdgePtr>::iterator Front::isPresent(const EdgePtr &_edge)
{
	int vertex0 = _edge->getVertex(0).second;
	int vertex1 = _edge->getVertex(1).second;

	if (points.find(vertex0) == points.end() || points.find(vertex0) == points.end())
		// Since points aren't both present, the vertex can't be present
		return front.end();
	else
	{
		// Look for a coincident edge
		for (std::map<EdgePtr, std::list<EdgePtr>::iterator>::iterator it = points[vertex1].begin(); it != points[vertex1].end(); it++)
		{
			int v0 = (*it->second)->getVertex(0).second;
			int v1 = (*it->second)->getVertex(1).second;
			if ((v0 == vertex1 && v1 == vertex0) || (v0 == vertex0 && v1 == vertex1))
				return it->second;
		}

		return front.end();
	}
}

void Front::addEdgePoints(std::list<EdgePtr>::iterator &_edge)
{
	for (int i = 0; i < 2; i++)
	{
		PointData data = (*_edge)->getVertex(i);
		if (points.find(data.second) == points.end())
			points[data.second] = std::map<EdgePtr, std::list<EdgePtr>::iterator>();

		points[data.second][(*_edge)] = _edge;
	}

}

void Front::removeEdgePoints(EdgePtr &_edge)
{
	for (int i = 0; i < 2; i++)
	{
		PointData data = _edge->getVertex(i);
		if (points.find(data.second) != points.end())
		{
			points[data.second].erase(_edge);

			// If no more edges use the point, then remove its entry from the map
			if (points[data.second].empty())
			{
				points.erase(data.second);
				//cout << "\tPoint removed from front: " << data.second << "\n";
			}
		}
		else
			std::cout << "WARNING, removing inexistent edge " << *_edge << "\n";
	}
}
