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

Triangle::Triangle(const PointXYZ &_vertex0, const PointXYZ &_vertex1, const PointXYZ &_vertex2, const int _index0, const int _index1, const int _index2, const PointXYZ &_ballCenter, const double _ballRadius)
{
	vertices.resize(3);
	vertices[0] = make_pair((PointXYZ *) &_vertex0, _index0);
	vertices[1] = make_pair((PointXYZ *) &_vertex1, _index1);
	vertices[2] = make_pair((PointXYZ *) &_vertex2, _index2);
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

PointDat Triangle::getVertex(const int _n) const
{
	if (_n < 3)
		return vertices[_n];
	else
		return make_pair<PointXYZ *, int>(NULL, -1);
}

int Triangle::getVertexIndex(const int _n) const
{
	if (_n < 3)
		return vertices[_n].second;
	else
		return -1;
}

Edge Triangle::getEdge(const int _n) const
{
	int index0 = _n % 3;
	int index1 = (_n + 1) % 3;
	int index2 = (_n + 2) % 3;
	return Edge(vertices[index0], vertices[index1], vertices[index2], ballCenter);
}
