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
