/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <vector>
#include "Edge.h"
#include "Helper.h"
#include "Triangle.h"

using namespace pcl;
using namespace std;
using namespace Eigen;

class Pivoter
{
public:
	Pivoter(const PointCloud<PointNormal>::Ptr &_cloud, const double _ballRadius);
	~Pivoter();

	pair<int, TrianglePtr> pivot(const EdgePtr &_edge);
	TrianglePtr findSeed();

	bool isUsed(const int _index) const;
	void setUsed(const int _index);

private:
	pair<Vector3f, double> getCircumscribedCircle(const int _index0, const int _index1, const int _index2, Vector3f &_normal) const;
	bool getBallCenter(const int _index0, const int _index1, const int _index2, Vector3f &_center) const;

	bool isEmpty(const vector<int> &_data, const int _index0, const int _index1, const int _index2) const;
	vector<int> getNeighbors(const PointNormal &_point, const double _radius) const;

	KdTreeFLANN<PointNormal> kdtree;
	PointCloud<PointNormal>::Ptr cloud;
	vector<bool> used;
	double ballRadius;
};
