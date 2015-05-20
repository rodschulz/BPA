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

Triangle::Triangle(const PointXYZ &_p0, const PointXYZ &_p1, const PointXYZ &_p2)
{
	vertex0.first = (PointXYZ *)&_p0;
	vertex1.first = (PointXYZ *)&_p1;
	vertex2.first = (PointXYZ *)&_p2;
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
