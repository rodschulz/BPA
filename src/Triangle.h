/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/common/common.h>
#include <vector>
#include "Edge.h"

using namespace pcl;
using namespace std;

class Triangle
{
public:
	Triangle();
	Triangle(const PointXYZ &_vertex0, const PointXYZ &_vertex1, const PointXYZ &_vertex2, const int _index0, const int _index1, const int _index2, const PointXYZ &_ballCenter, const double _ballRadius);
	Triangle(const Triangle &_other);
	~Triangle();

	Triangle &operator=(const Triangle &_other);
	PointDat getVertex(const int _n) const;
	int getVertexIndex(const int _n) const;
	Edge getEdge(const int _n) const;

private:
	vector<PointDat> vertices;
	PointXYZ ballCenter;
	double ballRadius;
};
