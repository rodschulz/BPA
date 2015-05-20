/**
 * Author: rodrigo
 * 2015
 */
#include "Triangle.h"

Triangle::Triangle()
{
	vertex0.first = vertex1.first = vertex2.first = NULL;
	vertex0.second = vertex1.second = vertex2.second = -1;
}

Triangle::Triangle(const PointXYZ &_P0, const PointXYZ &_P1, const PointXYZ &_P2)
{
	vertex0.first = (PointXYZ *) &_P0;
	vertex1.first = (PointXYZ *) &_P1;
	vertex2.first = (PointXYZ *) &_P2;
	vertex0.second = vertex1.second = vertex2.second = -1;
}

Triangle::Triangle(const PointXYZ &_P0, const PointXYZ &_P1, const PointXYZ &_P2, const int _indexP0, const int _indexP1, const int _indexP2)
{
	vertex0.first = (PointXYZ *) &_P0;
	vertex1.first = (PointXYZ *) &_P1;
	vertex2.first = (PointXYZ *) &_P2;
	vertex0.second = _indexP0;
	vertex1.second = _indexP1;
	vertex2.second = _indexP2;
}

Triangle::Triangle(const Triangle &_other)
{
	vertex0 = _other.vertex0;
	vertex1 = _other.vertex1;
	vertex2 = _other.vertex2;
}

Triangle::~Triangle()
{
}

Triangle &Triangle::operator=(const Triangle &_other)
{
	if (this != &_other)
	{
		vertex0 = _other.vertex0;
		vertex1 = _other.vertex1;
		vertex2 = _other.vertex2;
	}

	return *this;
}

PointXYZ *Triangle::getVertex(const int _n) const
{
	switch (_n)
	{
		case 0:
			return vertex0.first;
		case 1:
			return vertex1.first;
		case 2:
			return vertex2.first;
		default:
			return NULL;
	}
}

int Triangle::getVertexIndex(const int _n) const
{
	switch (_n)
	{
		case 0:
			return vertex0.second;
		case 1:
			return vertex1.second;
		case 2:
			return vertex2.second;
		default:
			return -1;
	}
}
