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
	Triangle(const Triangle &_other);
	~Triangle();

	Triangle &operator=(const Triangle &_other);

private:
	// Pointers to the data and index of it
	pair<PointXYZ *, int> vertex0;
	pair<PointXYZ *, int> vertex1;
	pair<PointXYZ *, int> vertex2;
};
