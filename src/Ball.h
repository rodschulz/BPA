/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <stdlib.h>
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

#define COMPARISON_EPSILON (1E-12)

class Ball
{
public:
	Ball(const DataHolder &_holder, const double &_ballRadius);
	~Ball();

	pair<PointXYZ *, int> pivot(const Edge &_edge);
	bool findSeedTriangle(Triangle &_seedTriangle);

private:
	typedef bool (*compareMethod)(const pair<double, int> &, const pair<double, int> &);
	static bool compareAngles(const pair<double, int> &_p1, const pair<double, int> &_p2);

	pair<Vector3f, double> getCircumscribedCircle(const int _index0, const int _index1, const int _index2, Vector3f &_normal);

	KdTreeFLANN<PointXYZ> kdtree;
	PointCloud<PointXYZ>::Ptr cloud;
	PointCloud<Normal>::Ptr normals;
	vector<bool> *used;
	double ballRadius;
};
