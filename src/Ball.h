/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <vector>
#include "Edge.h"

using namespace pcl;
using namespace std;

class Ball
{
public:
	Ball(const PointCloud<PointXYZ>::Ptr &_cloud, const double &_ballRadius);
	~Ball();

	void pivot(const Edge &_edge);

private:
	KdTreeFLANN<PointXYZ> kdtree;
	PointCloud<PointXYZ>::Ptr cloud;
	double ballRadius;
};
