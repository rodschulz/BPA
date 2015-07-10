/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/src/Core/Matrix.h>

class CudaUtil
{
public:
	static bool calculateBallCenters(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud);

private:
	CudaUtil(){}
	~CudaUtil(){}
};
