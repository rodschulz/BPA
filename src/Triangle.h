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
	Triangle(const PointXYZ &_P0, const PointXYZ &_P1, const PointXYZ &_P2);
	Triangle(const PointXYZ &_P0, const PointXYZ &_P1, const PointXYZ &_P2, const int _indexP0, const int _indexP1, const int _indexP2);
	Triangle(const Triangle &_other);
	~Triangle();

	Triangle &operator=(const Triangle &_other);
	PointXYZ *getVertex(const int _n) const;
	int getVertexIndex(const int _n) const;

private:
	// Pointers to the data and index of it
	pair<PointXYZ *, int> vertex0;
	pair<PointXYZ *, int> vertex1;
	pair<PointXYZ *, int> vertex2;
};
