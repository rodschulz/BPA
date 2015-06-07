/**
 * Author: rodrigo
 * 2015
 */
#include "Edge.h"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include "Helper.h"

using namespace Eigen;

ostream &operator<<(ostream &_stream, const Edge &_edge)
{
	_stream << "(" << _edge.vertices[0].second << ", " << _edge.vertices[1].second << ")";
	return _stream;
}

Edge::Edge()
{
	vertices.clear();
	oppositeVertex = make_pair<PointNormal *, int>(NULL, -1);
	ballCenter = middlePoint = PointNormal();
	pivotingRadius = 0;
	active = false;
	setId();
	str = "";
}

Edge::Edge(const PointData &_v0, const PointData &_v1, const PointData &_opposite, const PointNormal &_ballCenter)
{
	vertices.push_back(_v0);
	vertices.push_back(_v1);
	oppositeVertex = _opposite;

	ballCenter = _ballCenter;
	middlePoint = Helper::makePointNormal((_v0.first->x + _v1.first->x) * 0.5, (_v0.first->y + _v1.first->y) * 0.5, (_v0.first->z + _v1.first->z) * 0.5);

	// TODO check if the pivoting radius is really going to be used
	Vector3f m = middlePoint.getVector3fMap();
	Vector3f c = ballCenter.getVector3fMap();
	pivotingRadius = (m - c).norm();

	active = true;
	setId();

	str = this->toString();
}

Edge::Edge(const Edge &_other)
{
	vertices = _other.vertices;
	oppositeVertex = _other.oppositeVertex;

	ballCenter = _other.ballCenter;
	pivotingRadius = _other.pivotingRadius;

	middlePoint = _other.middlePoint;
	active = _other.active;
	id = _other.id;

	str = _other.str;
}

Edge::~Edge()
{
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
		id = _other.id;

		str = _other.str;
	}

	return *this;
}

bool Edge::operator<(const Edge &_other) const
{
	return pivotingRadius < _other.pivotingRadius;
}

bool Edge::operator==(const Edge &_other) const
{
	bool equals = (vertices[0] == _other.vertices[0] || vertices[0] == _other.vertices[1]) && (vertices[1] == _other.vertices[0] || vertices[1] == _other.vertices[1]);

	return equals;
}

bool Edge::operator!=(const Edge &_other) const
{
	return !this->operator==(_other);
}

void Edge::setActive(const bool _active)
{
	active = _active;
}

bool Edge::isActive() const
{
	return active;
}

PointData Edge::getVertex(const int _n) const
{
	if (_n < 2)
		return vertices[_n];
	else
		return make_pair<PointNormal *, int>(NULL, -1);
}

PointData Edge::getOppositeVertex() const
{
	return oppositeVertex;
}

PointNormal Edge::getMiddlePoint() const
{
	return middlePoint;
}

PointNormal Edge::getBallCenter() const
{
	return ballCenter;
}

double Edge::getPivotingRadius() const
{
	return pivotingRadius;
}
