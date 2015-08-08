/**
 * Author: rodrigo
 * 2015
 */
#include "GpuRoutines.h"

#include <ostream>
#include <iostream>
#include "GpuUtils.h"

#define EPSILON		1E-10
#define MAX_BLOCKS	65535
#define MAX_THREADS	1024

// Pointers to memory in device
bool *devNotUsed = NULL;
gpu::Point *devPoints = NULL;
gpu::DeviceNode *devNodes = NULL;
gpu::BallCenter *devAuxCenter = NULL;
gpu::DeviceKDTree *devAuxTree = NULL;

// Global variable in device
__device__ gpu::BallCenter *devCenter;
__device__ gpu::DeviceKDTree *devKDTree;

// Debug variables
__device__ int devFound;
__device__ int devTreeSize;
__device__ int devTreeRoot;
__device__ int devLeftChild;
__device__ int devRightChild;

std::ostream &operator<<(std::ostream &_stream, const gpu::BallCenter &_center)
{
	_stream << "c=(" << _center.cx << ", " << _center.cy << ", " << _center.cz << ") / (" << _center.idx0 << ", " << _center.idx1 << ", " << _center.idx2 << ")";
	return _stream;
}

void GpuRoutines::allocPoints(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud)
{
	gpu::Point *hostPoints = (gpu::Point *) &_cloud->points[0];
	GpuUtils::createInDev<gpu::Point>(&devPoints, hostPoints, _cloud->size());
}

void GpuRoutines::allocUsed(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const bool* _notUsed)
{
	GpuUtils::createInDev<bool>(&devNotUsed, _notUsed, _cloud->size());
}

__device__ bool isOriented(const gpu::Point *_normal, const gpu::Point *_p0, const gpu::Point *_p1, const gpu::Point *_p2)
{
	int count = 0;
	count = _p0->normalDot(*_normal) < 0 ? count + 1 : count;
	count = _p1->normalDot(*_normal) < 0 ? count + 1 : count;
	count = _p2->normalDot(*_normal) < 0 ? count + 1 : count;

	return count <= 1;
}

__device__ float getCircumscribedCircle(const gpu::Point &_p0, const gpu::Point &_p1, const gpu::Point &_p2, gpu::BallCenter &_center)
{
	gpu::Point d10 = _p1 - _p0;
	gpu::Point d20 = _p2 - _p0;
	gpu::Point d01 = _p0 - _p1;
	gpu::Point d12 = _p1 - _p2;
	gpu::Point d21 = _p2 - _p1;
	gpu::Point d02 = _p0 - _p2;

	float norm01 = d01.norm();
	float norm12 = d12.norm();
	float norm02 = d02.norm();

	float norm01C12 = d01.cross(d12).norm();

	float alpha = (norm12 * norm12 * d01.dot(d02)) / (2 * norm01C12 * norm01C12);
	float beta = (norm02 * norm02 * d10.dot(d12)) / (2 * norm01C12 * norm01C12);
	float gamma = (norm01 * norm01 * d20.dot(d21)) / (2 * norm01C12 * norm01C12);

	gpu::Point circumscribedCircleCenter = (_p0 * alpha) + (_p1 * beta) + (_p2 * gamma);
	float circumscribedCircleRadius = (norm01 * norm12 * norm02) / (2 * norm01C12);

	_center.cx = circumscribedCircleCenter.x;
	_center.cy = circumscribedCircleCenter.y;
	_center.cz = circumscribedCircleCenter.z;

	return circumscribedCircleRadius;
}

__device__ bool getBallCenter(const gpu::Point *_point0, const gpu::Point *_point1, const gpu::Point *_point2, const float _ballRadius, gpu::BallCenter *_center)
{
	bool status = false;
	_center->isValid = false;

	// Local pointers
	const gpu::Point *p0 = _point0;
	const gpu::Point *p1 = _point1;
	const gpu::Point *p2 = _point2;

	gpu::Point v10 = p1->operator -(*p0);
	gpu::Point v20 = p2->operator -(*p0);
	gpu::Point normal = v10.cross(v20);

	// Calculate ball center only if points are not collinear
	if (normal.norm() > EPSILON)
	{
		// Normalize to avoid precision errors while checking the orientation
		normal.normalize();
		if (!isOriented(&normal, p0, p1, p2))
		{
			// Wrong orientation, swap vertices to get a CCW oriented triangle so face's normal pointing upwards
			int aux = _center->idx0;
			_center->idx0 = _center->idx1;
			_center->idx1 = aux;

			p0 = _point1;
			p1 = _point0;

			v10 = p1->operator -(*p0);
			v20 = p2->operator -(*p0);
			normal = v10.cross(v20);
			normal.normalize();
		}

		float circleRadius = getCircumscribedCircle(*p0, *p1, *p2, *_center);
		float squaredDistance = _ballRadius * _ballRadius - circleRadius * circleRadius;

		if (squaredDistance > 0)
		{
			float distance = sqrt(fabs(squaredDistance));
			_center->add(normal * distance);
			_center->isValid = true;
			status = true;
		}
	}

	return status;
}

__device__ bool isEmpty(const gpu::BallCenter *_center, const gpu::Point *_points, const int _pointNumber, const float _ballRadius)
{
	for (int i = 0; i < _pointNumber; i++)
	{
		if ((i == _center->idx0) || (i == _center->idx1) || (i == _center->idx2))
			continue;

		if (_center->dist(_points[i]) >= _ballRadius)
			continue;

		return false;
	}

	return true;
}

__global__ void checkForSeeds(const gpu::Point *_points, const int _pointNumber, const int *_neighbors, const int _neighborsSize, const bool *_notUsed, const int _index0, const float _ballRadius, const int _neighborsPerThread)
{
	///// Assign debug variables /////
	/*devTreeSize = devKDTree->size;
	 devTreeRoot = devKDTree->root;
	 devLeftChild = devKDTree->nodes[devKDTree->root].left;
	 devRightChild = devKDTree->nodes[devKDTree->root].right;*/
	//////////////////////////////////
	int start0 = blockIdx.x;
	int end0 = start0 + 1;

	int start1 = threadIdx.x * _neighborsPerThread;
	int end1 = start1 + _neighborsPerThread;

	for (int j = start0; j < end0 && j < _neighborsSize; j++)
	{
		if (devFound == 0)
		{
			int index1 = _neighbors[j];

			// Skip invalid combinations
			if (index1 == _index0 || !_notUsed[index1])
				continue;

			for (size_t k = start1; k < end1 && k < _neighborsSize && devFound == 0; k++)
			{
				int index2 = _neighbors[k];

				// Skip invalid combinations
				if (index1 == index2 || index2 == _index0 || !_notUsed[index2])
					continue;

				gpu::BallCenter center(_index0, index1, index2);
				if (getBallCenter(&_points[_index0], &_points[index1], &_points[index2], _ballRadius, &center))
				{
					//if (isEmpty(&center, _points, _pointNumber, _ballRadius))
					if (devKDTree->isEmptyRadius(&center, _ballRadius))
					{
						if (devFound == 0)
						{
							atomicExch(&devFound, 1);
							devCenter->cx = center.cx;
							devCenter->cy = center.cy;
							devCenter->cz = center.cz;
							devCenter->idx0 = center.idx0;
							devCenter->idx1 = center.idx1;
							devCenter->idx2 = center.idx2;
							devCenter->isValid = center.isValid;
							break;
						}
					}
				}
			}
		}
	}
}

gpu::BallCenter GpuRoutines::findSeed(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const std::vector<int> &_neighbors, const bool *_notUsed, const int _index0, const float _ballRadius)
{
	size_t neighborsSize = _neighbors.size();
	if (neighborsSize > MAX_BLOCKS)
	{
		std::cout << "ERROR: radius size too big" << std::endl;
		exit(1);
	}

	int blocks = neighborsSize < MAX_BLOCKS ? neighborsSize : MAX_BLOCKS;
	int threads = neighborsSize < MAX_THREADS ? neighborsSize : MAX_THREADS;
	int neighborsPerThread = ceil((double) neighborsSize / threads);

	size_t cloudSize = _cloud->size();

	// Prepare memory buffers
	if (devPoints == NULL)
		allocPoints(_cloud);
	if (devNotUsed == NULL)
		allocUsed(_cloud, _notUsed);

	// Copy not used data to dev
	size_t notUsedBytes = sizeof(bool) * _cloud->size();
	GpuUtils::setData<bool>(&devNotUsed, _notUsed, _cloud->size());

	// Create and prepare buffer with neighbors indices
	int *devNeighbors;
	GpuUtils::createInDev<int>(&devNeighbors, &_neighbors[0], neighborsSize);

	// Prepare global variable 'devFound'
	int found = 0;
	GpuUtils::setSymbol<int>(devFound, &found);

	// Prepare global variable 'devFoundCenter'
	gpu::BallCenter center = gpu::BallCenter();
	if (devAuxCenter == NULL)
		GpuUtils::allocMemory<gpu::BallCenter>(&devAuxCenter, 1);
	GpuUtils::setData<gpu::BallCenter>(&devAuxCenter, &center, 1);
	GpuUtils::setSymbol<gpu::BallCenter *>(devCenter, &devAuxCenter);

	// Execute kernel
	checkForSeeds<<<blocks, threads>>>(devPoints, _cloud->size(), devNeighbors, neighborsSize, devNotUsed, _index0, _ballRadius, neighborsPerThread);

	///// Retrieve debug variables /////
//	cudaMemcpyFromSymbol(&found, devFound, sizeof(int));
//	checkErrors("cudaMemcpyFromSymbol failed");
//
//	 int treeSize = -1;
//	 cudaMemcpyFromSymbol(&treeSize, devTreeSize, sizeof(int));
//	 checkErrors("cudaMemcpyFromSymbol failed");
//
//	 int treeRoot = -5;
//	 cudaMemcpyFromSymbol(&treeRoot, devTreeRoot, sizeof(int));
//	 checkErrors("cudaMemcpyFromSymbol failed");
//
//	 int leftChild = -5;
//	 cudaMemcpyFromSymbol(&leftChild, devLeftChild, sizeof(int));
//	 checkErrors("cudaMemcpyFromSymbol failed");
//
//	 int rightChild = -5;
//	 cudaMemcpyFromSymbol(&rightChild, devRightChild, sizeof(int));
//	 checkErrors("cudaMemcpyFromSymbol failed");
	////////////////////////////////////

	// Retrieve results
	GpuUtils::getData<gpu::BallCenter>(&center, devAuxCenter, 1);

	// Free allocated memory
	cudaFree(devNeighbors);
	checkErrors("cudaFree devNeighbors failed");

	return center;
}

void GpuRoutines::buildInDeviceKDTree(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud)
{
	// Build a KDTree using host memory
	gpu::HostKDTree tree = gpu::HostKDTree((const gpu::Point *) &_cloud->points[0]);
	for (size_t i = 0; i < _cloud->size(); i++)
		tree.insert((gpu::Point *) &_cloud->points[i], i);

	//*
	// Get the serialized version of the tree
	gpu::DeviceNode *hostMem = new gpu::DeviceNode[tree.size];
	tree.getSerializedRepresentation(hostMem);

	// Allocate cloud data if it has not already been done
	if (devPoints == NULL)
		allocPoints(_cloud);

	// Allocate memory for tree's nodes and copy data
	GpuUtils::createInDev<gpu::DeviceNode>(&devNodes, hostMem, tree.size);

	// Create the serialized tree
	gpu::DeviceKDTree serializedTree = gpu::DeviceKDTree();
	serializedTree.root = 0;
	serializedTree.size = tree.size;
	serializedTree.nodes = devNodes;
	serializedTree.points = devPoints;

	// Allocate memory for the tree itself
	GpuUtils::createInDev<gpu::DeviceKDTree>(&devAuxTree, &serializedTree, 1);
	GpuUtils::setSymbol<gpu::DeviceKDTree *>(devKDTree, &devAuxTree);
	//*/
}

void GpuRoutines::releaseMemory()
{
	cudaFree(devNotUsed);
	cudaFree(devPoints);
	cudaFree(devNodes);
	cudaFree(devAuxCenter);
	cudaFree(devAuxTree);
}

void GpuRoutines::prepareStackSize()
{
	size_t sizeLimit = 1024 * 5;
	cudaDeviceSetLimit(cudaLimitStackSize, sizeLimit);
	checkErrors("cudaDeviceSetLimit failed");
}
