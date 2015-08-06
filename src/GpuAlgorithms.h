/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "GpuStructs.h"
#include <cuda_runtime.h>

class GpuAlgorithms
{
public:
	static bool calculateBallCenters(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud);
	static bool radiusSearch(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const int target, double _radius, std::vector<int> &_idxs);
	static gpu::BallCenter findSeed(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const std::vector<int> &_neighbors, const bool *_notUsed, const int _index0, const float _ballRadius, const gpu::DeviceKDTree &_kdtree);
	static gpu::DeviceKDTree buildKDTree(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud);
private:
	GpuAlgorithms()
	{
	}
	~GpuAlgorithms()
	{
	}

	static void allocPoints(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud);
	static void allocUsed(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const bool* _notUsed);
};
