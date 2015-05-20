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

#define COMPARISON_EPSILON (1E-12)

class Ball
{
public:
	Ball(const DataHolder &_holder, const double &_ballRadius);
	~Ball();

	void pivot(const Edge &_edge);
	bool findSeedTriangle(Triangle &_seedTriangle);

private:
	KdTreeFLANN<PointXYZ> kdtree;
	PointCloud<PointXYZ>::Ptr cloud;
	PointCloud<Normal>::Ptr normals;
	vector<bool> *used;
	double ballRadius;
};
