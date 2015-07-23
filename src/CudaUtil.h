/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cuda_runtime.h>

#define cudaCheckErrors(msg) \
	do { \
		cudaError_t __err = cudaGetLastError(); \
		if (__err != cudaSuccess) { \
			fprintf(stderr, "Fatal error: %s (%s at %s:%d)\n", \
			msg, cudaGetErrorString(__err), \
			__FILE__, __LINE__); \
			fprintf(stderr, "*** FAILED - ABORTING\n"); \
			exit(1); \
		} \
	} while (0)

class CudaUtil
{
public:
	static bool calculateBallCenters(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud);
	static bool radiusSearch(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const int target, double _radius, std::vector<int> &_idxs);

	static bool findSeed(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const std::vector<int> &_neighbors, const bool *_notUsed, const int _index0);

private:
	CudaUtil()
	{
	}
	~CudaUtil()
	{
	}

	static void allocPoints(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud);
	static void allocUsed(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const bool* _notUsed);

	static size_t getAvailableMemory()
	{
		size_t freeMem, totalMem;
		cudaMemGetInfo(&freeMem, &totalMem);
		cudaCheckErrors("memInfo failed");
		return freeMem;
	}
};
