/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "GpuStructs.h"
#include <cuda_runtime.h>

class GpuRoutines
{
public:
	static gpu::BallCenter findSeed(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const std::vector<int> &_neighbors, const bool *_notUsed, const int _index0, const float _ballRadius);
	static void buildInDeviceKDTree(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud);
	static void releaseMemory();
	static void prepareStackSize();
private:
	GpuRoutines()
	{
	}
	~GpuRoutines()
	{
	}

	static void allocPoints(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud);
	static void allocUsed(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const bool* _notUsed);
};
