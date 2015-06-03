/**
 * Author: rodrigo
 * 2015
 */
#include "Triangle.h"

Triangle::Triangle()
{
	vertices.clear();
	ballCenter = PointNormal();
	ballRadius = 0;
}

Triangle::Triangle(const PointNormal &_v0, const PointNormal &_v1, const PointNormal &_v2, const int _index0, const int _index1, const int _index2, const PointNormal &_ballCenter, const double _ballRadius)
{
	vertices.resize(3);
	vertices[0] = make_pair((PointNormal *) &_v0, _index0);
	vertices[1] = make_pair((PointNormal *) &_v1, _index1);
	vertices[2] = make_pair((PointNormal *) &_v2, _index2);
	ballCenter = _ballCenter;
	ballRadius = _ballRadius;
}

Triangle::Triangle(const Triangle &_other)
{
	vertices = _other.vertices;
	ballCenter = _other.ballCenter;
	ballRadius = _other.ballRadius;
}

Triangle::~Triangle()
{
}

Triangle &Triangle::operator=(const Triangle &_other)
{
	if (this != &_other)
	{
		vertices = _other.vertices;
		ballCenter = _other.ballCenter;
		ballRadius = _other.ballRadius;
	}

	return *this;
}
