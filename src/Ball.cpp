/**
 * Author: rodrigo
 * 2015
 */
#include "Ball.h"

Ball::Ball(const PointCloud<PointXYZ>::Ptr &_cloud, const double &_ballRadius)
{
	kdtree.setInputCloud(_cloud);
	cloud = _cloud;
	ballRadius = _ballRadius;
}

Ball::~Ball()
{
}

void Ball::pivot(const Edge &_edge)
{
	PointXYZ target = _edge.getMiddlePoint();

	// Get neighborhood
	vector<int> indices;
	vector<float> squaredDistances;
	kdtree.radiusSearch(target, ballRadius * 2, indices, squaredDistances);
}
