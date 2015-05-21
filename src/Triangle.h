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
	Triangle(const PointXYZ &_P0, const PointXYZ &_P1, const PointXYZ &_P2, const int _indexP0, const int _indexP1, const int _indexP2, const PointXYZ &_ballCenter, const double _ballRadius);
	Triangle(const Triangle &_other);
	~Triangle();

	Triangle &operator=(const Triangle &_other);
	PointXYZ *getVertex(const int _n) const;
	int getVertexIndex(const int _n) const;
	Edge getEdge(const int _n) const;

private:
	vector<pair<PointXYZ *, int> > vertices;
	PointXYZ ballCenter;
	double ballRadius;
};
