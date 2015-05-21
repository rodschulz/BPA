/**
 * Author: rodrigo
 * 2015
 */
#include "Triangle.h"

Triangle::Triangle()
{
	vertices.clear();
	ballCenter = PointXYZ(0, 0, 0);
	ballRadius = 0;
}

Triangle::Triangle(const PointXYZ &_P0, const PointXYZ &_P1, const PointXYZ &_P2, const int _index0, const int _index1, const int _index2, const PointXYZ &_ballCenter, const double _ballRadius)
{
	vertices.resize(3);
	vertices[0] = make_pair((PointXYZ *) &_P0, _index0);
	vertices[1] = make_pair((PointXYZ *) &_P1, _index1);
	vertices[2] = make_pair((PointXYZ *) &_P2, _index2);
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

PointXYZ *Triangle::getVertex(const int _n) const
{
	PointXYZ *p = NULL;
	if (_n < 3)
		p = vertices[_n].first;

	return p;
}

int Triangle::getVertexIndex(const int _n) const
{
	int index = -1;
	if (_n < 3)
		index = vertices[_n].second;

	return index;
}

Edge Triangle::getEdge(const int _n) const
{
	int index0 = _n % 3;
	int index1 = (_n + 1) % 3;
	int index2 = (_n + 2) % 3;

	return Edge(vertices[index0].first, vertices[index1].first, vertices[index2].first, ballCenter);
}
