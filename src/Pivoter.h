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

private:
	KdTreeFLANN<PointNormal> kdtree;
	PointCloud<PointNormal>::Ptr cloud;
	vector<bool> used;
	double ballRadius;
};

