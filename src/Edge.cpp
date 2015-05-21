/**
 * Author: rodrigo
 * 2015
 */
#include "Edge.h"
#include <eigen3/Eigen/src/Core/Matrix.h>

using namespace Eigen;

Edge::Edge()
{
	vertices.clear();
	oppositeVertex = make_pair<PointXYZ *, int>(NULL, -1);
	ballCenter = middlePoint = PointXYZ(0, 0, 0);
	pivotingRadius = 0;
	active = false;
}

Edge::Edge(const PointDat &_vertex0, const PointDat &_vertex1, const PointDat _oppositeVertex, const PointXYZ &_ballCenter)
{
	vertices.push_back(_vertex0);
	vertices.push_back(_vertex1);
	oppositeVertex = _oppositeVertex;

	ballCenter = _ballCenter;
	middlePoint = PointXYZ((_vertex0.first->x + _vertex1.first->x) * 0.5, (_vertex0.first->y + _vertex1.first->y) * 0.5, (_vertex0.first->z + _vertex1.first->z) * 0.5);

	Vector3f m = middlePoint.getVector3fMap();
	Vector3f c = ballCenter.getVector3fMap();
	pivotingRadius = (m - c).norm();

	active = true;
}

Edge::Edge(const Edge &_other)
{
	vertices = _other.vertices;
	oppositeVertex = _other.oppositeVertex;

	ballCenter = _other.ballCenter;
	pivotingRadius = _other.pivotingRadius;

	middlePoint = _other.middlePoint;
	active = _other.active;
}

Edge &Edge::operator=(const Edge &_other)
{
	if (this != &_other)
	{
		vertices = _other.vertices;
		oppositeVertex = _other.oppositeVertex;

		ballCenter = _other.ballCenter;
		pivotingRadius = _other.pivotingRadius;

		middlePoint = _other.middlePoint;
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

PointDat Edge::getVertex(const int _n) const
{
	if (_n < 2)
		return vertices[_n];
	else
		return make_pair<PointXYZ *, int>(NULL, -1);
}

PointDat Edge::getOppositeVertex() const
{
	return oppositeVertex;
}

PointXYZ Edge::getMiddlePoint() const
{
	return middlePoint;
}

PointXYZ Edge::getBallCenter() const
{
	return ballCenter;
}

double Edge::getPivotingRadius() const
{
	return pivotingRadius;
}
