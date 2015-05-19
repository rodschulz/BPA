/**
 * Author: rodrigo
 * 2015
 */
#include "Edge.h"
#include <eigen3/Eigen/src/Core/Matrix.h>

using namespace Eigen;

Edge::Edge()
{
	vertex0 = NULL;
	vertex1 = NULL;
	oppositeVertex = NULL;
	circleCenter = edgeMiddle = PointXYZ(0, 0, 0);
	pivotingRadius = 0;
	active = false;
}

Edge::Edge(const PointXYZ *_v0, const PointXYZ *_v1, const PointXYZ *_oppositeVertex, const PointXYZ &_circleCenter)
{
	vertex0 = (PointXYZ *) _v0;
	vertex1 = (PointXYZ *) _v1;
	oppositeVertex = (PointXYZ *) _oppositeVertex;

	circleCenter = _circleCenter;
	edgeMiddle = PointXYZ((vertex0->x + vertex1->x) / 2, (vertex0->y + vertex1->y) / 2, (vertex0->z + vertex1->z) / 2);

	Vector3f m = edgeMiddle.getVector3fMap();
	Vector3f c = circleCenter.getVector3fMap();
	pivotingRadius = (m - c).norm();

	active = true;
}

Edge::Edge(const Edge &_other)
{
	vertex0 = _other.vertex0;
	vertex1 = _other.vertex1;
	oppositeVertex = _other.oppositeVertex;

	circleCenter = _other.circleCenter;
	pivotingRadius = _other.pivotingRadius;

	edgeMiddle = _other.edgeMiddle;
	active = _other.active;
}

Edge &Edge::operator=(const Edge &_other)
{
	if (this != &_other)
	{
		vertex0 = _other.vertex0;
		vertex1 = _other.vertex1;
		oppositeVertex = _other.oppositeVertex;

		circleCenter = _other.circleCenter;
		pivotingRadius = _other.pivotingRadius;

		edgeMiddle = _other.edgeMiddle;
		active = _other.active;
	}

	return *this;
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

PointXYZ *Edge::getVertex(const int _point) const
{
	switch (_point)
	{
		case 0:
			return vertex0;
		case 1:
			return vertex1;
		default:
			return NULL;
	}
}

PointXYZ Edge::getMiddlePoint() const
{
	return edgeMiddle;
}

PointXYZ Edge::getCircleCenter() const
{
	return circleCenter;
}

double Edge::getPivotingRadius() const
{
	return pivotingRadius;
}
