/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/common/common.h>

using namespace pcl;
using namespace std;

class Triangle
{
public:
	Triangle();
	Triangle(const PointXYZ &_p0, const PointXYZ &_p1, const PointXYZ &_p2);
	Triangle(const Triangle &_other);
	~Triangle();

	Triangle &operator=(const Triangle &_other);
	PointXYZ *getVertex(const int _n) const;

private:
	// Pointers to the data and index of it
	pair<PointXYZ *, int> vertex0;
	pair<PointXYZ *, int> vertex1;
	pair<PointXYZ *, int> vertex2;
};
