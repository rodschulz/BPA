/**
 * Author: rodrigo
 * 2015
 */
#include "Edge.h"

Edge::Edge(const PointXYZ *_point0, const PointXYZ *_point1)
{
	point0 = _point0;
	point1 = _point1;
	active = true;
}

Edge::~Edge()
{
}

void Edge::setActive(const bool _active)
{
	active = _active;
}

bool Edge::isActive() const
{
	return active;
}

PointXYZ *Edge::getPoint(const int _point)
{
	switch (_point)
	{
		case 0:
			return point0;
		case 1:
			return point1;
		default:
			return NULL;
	}
}
