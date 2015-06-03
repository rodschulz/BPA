/**
 * Author: rodrigo
 * 2015
 */
#include "Pivoter.h"

Pivoter::Pivoter(const PointCloud<PointNormal>::Ptr &_cloud, const double _ballRadius)
{
	cloud = _cloud;
	ballRadius = _ballRadius;

	used.clear();
	used.resize(_cloud->size(), false);
}

Pivoter::~Pivoter()
{
}

