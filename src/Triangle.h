/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/common/common.h>
#include <vector>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include "Edge.h"

using namespace pcl;
using namespace std;
using namespace Eigen;

// Triangle smart pointer
class Triangle;
typedef boost::shared_ptr<Triangle> TrianglePtr;

class Triangle
{
public:
	Triangle();
	Triangle(const PointNormal &_v0, const PointNormal &_v1, const PointNormal &_v2, const int _index0, const int _index1, const int _index2, const PointXYZ &_ballCenter, const double _ballRadius);
	Triangle(const Triangle &_other);
	~Triangle();

	Triangle &operator=(const Triangle &_other);
	PointData getVertex(const int _n) const;
	int getVertexIndex(const int _n) const;
	Edge getEdge(const int _n) const;

private:
	vector<PointData> vertices;
	PointXYZ ballCenter;
	double ballRadius;
	Vector3f normal;
};
