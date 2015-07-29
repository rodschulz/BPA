/**
 * Author: rodrigo
 * 2015
 */
#include "GpuAlgorithms.h"
#include <ostream>
#include <iostream>
#include "GpuUtils.h"

#define EPSILON	1E-10

// Pointers to memory in device
Point *devPoints = NULL;
bool *devNotUsed = NULL;
BallCenter *auxPtr = NULL;

// Global variable in device
__device__ int devFound;
__device__ BallCenter *devCenter;
__device__ Point *devPointDbg;

std::ostream &operator<<(std::ostream &_stream, const BallCenter &_center)
{
	_stream << "c=(" << _center.cx << ", " << _center.cy << ", " << _center.cz << ") / (" << _center.idx0 << ", " << _center.idx1 << ", " << _center.idx2 << ")";
	return _stream;
}

void GpuAlgorithms::allocPoints(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud)
{
	Point *hostPoints = (Point *) &_cloud->points[0];
	GpuUtils::createInDev<Point>(&devPoints, hostPoints, _cloud->size());
}

void GpuAlgorithms::allocUsed(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const bool* _notUsed)
{
	GpuUtils::createInDev<bool>(&devNotUsed, _notUsed, _cloud->size());
}

__global__ void searchCloserPoints(const int _target, const Point *_points, const int _pointNumber, const double _searchRadius, const int _pointsPerThread, bool *_selected)
{
	int startIdx = (blockIdx.x * blockDim.x + threadIdx.x) * _pointsPerThread;
	double sqrRadius = _searchRadius * _searchRadius;

	for (int i = startIdx; i < startIdx + _pointsPerThread && i < _pointNumber; i++)
	{
		_selected[i] = _points[_target].sqrDist(_points[i]) < sqrRadius;
	}
}

bool GpuAlgorithms::radiusSearch(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const int _target, double _radius, std::vector<int> &_idxs)
{
	int blocks = 10;
	int threads = 256;
	size_t cloudSize = _cloud->size();

	// Copy points to device
	if (devPoints == NULL)
		allocPoints(_cloud);

	// Array to store points within radius
	bool *devSelected;
	cudaMalloc((void **) &devSelected, sizeof(bool) * cloudSize);
	checkErrors("cudaMalloc selected failed");

	// Calculate adequate number of blocks and threads
	while (cloudSize / blocks < 2)
		blocks /= 2;
	int pointsPerBlock = ceil((double) cloudSize / blocks);

	while (pointsPerBlock / threads < 1)
		threads /= 2;
	int pointsPerThread = ceil((double) pointsPerBlock / threads);

	// Execute kernel
	searchCloserPoints<<<blocks, threads>>>(_target, devPoints, cloudSize, _radius, pointsPerThread, devSelected);

	// Copy data to host
	bool *selected = (bool *) calloc(cloudSize, sizeof(bool));
	cudaMemcpy(selected, devSelected, sizeof(bool) * cloudSize, cudaMemcpyDeviceToHost);
	checkErrors("cudaMemcpy selected failed");
	//cudaFree(devSelected);
	//checkErrors("cudaFree selected failed");

	for (size_t i = 0; i < cloudSize; i++)
		if (selected[i])
			_idxs.push_back(i);

	free(selected);

	return true;
}

///////////////////////////////
__device__ bool isOriented(const Point *_normal, const Point *_p0, const Point *_p1, const Point *_p2)
{
	int count = 0;
	count = _p0->normalDot(*_normal) < 0 ? count + 1 : count;
	count = _p1->normalDot(*_normal) < 0 ? count + 1 : count;
	count = _p2->normalDot(*_normal) < 0 ? count + 1 : count;

	return count <= 1;
}

__device__ float getCircumscribedCircle(const Point &_p0, const Point &_p1, const Point &_p2, BallCenter &_center)
{
	Point d10 = _p1 - _p0;
	Point d20 = _p2 - _p0;
	Point d01 = _p0 - _p1;
	Point d12 = _p1 - _p2;
	Point d21 = _p2 - _p1;
	Point d02 = _p0 - _p2;

	float norm01 = d01.norm();
	float norm12 = d12.norm();
	float norm02 = d02.norm();

	float norm01C12 = d01.cross(d12).norm();

	float alpha = (norm12 * norm12 * d01.dot(d02)) / (2 * norm01C12 * norm01C12);
	float beta = (norm02 * norm02 * d10.dot(d12)) / (2 * norm01C12 * norm01C12);
	float gamma = (norm01 * norm01 * d20.dot(d21)) / (2 * norm01C12 * norm01C12);

	Point circumscribedCircleCenter = (_p0 * alpha) + (_p1 * beta) + (_p2 * gamma);
	float circumscribedCircleRadius = (norm01 * norm12 * norm02) / (2 * norm01C12);

	_center.cx = circumscribedCircleCenter.x;
	_center.cy = circumscribedCircleCenter.y;
	_center.cz = circumscribedCircleCenter.z;

	return circumscribedCircleRadius;
}

__device__ bool getBallCenter(const Point *_point0, const Point *_point1, const Point *_point2, const float _ballRadius, BallCenter *_center)
{
	bool status = false;
	_center->isValid = false;

	// Local pointers
	const Point *p0 = _point0;
	const Point *p1 = _point1;
	const Point *p2 = _point2;

	Point v10 = p1->operator -(*p0);
	Point v20 = p2->operator -(*p0);
	Point normal = v10.cross(v20);

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

__device__ bool isEmpty(const BallCenter *_center, const Point *_points, const int _pointNumber, const float _ballRadius)
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

__global__ void checkForSeeds(const Point *_points, const int _pointNumber, const int *_neighbors, const int _neighborsSize, const bool *_notUsed, const int _index0, const float _ballRadius)
{
	int startIdx = threadIdx.x;
	int endIdx = startIdx + 1;

	for (int j = startIdx; j < endIdx && j < _neighborsSize; j++)
	{
		if (devFound == 0)
		{
			int index1 = _neighbors[j];

			// Skip invalid combinations
			if (index1 == _index0 || !_notUsed[index1])
				continue;

			for (size_t k = 0; k < _neighborsSize && devFound == 0; k++)
			{
				int index2 = _neighbors[k];

				// Skip invalid combinations
				if (index1 == index2 || index2 == _index0 || !_notUsed[index2])
					continue;

				BallCenter center(_index0, index1, index2);
				if (getBallCenter(&_points[_index0], &_points[index1], &_points[index2], _ballRadius, &center))
				{
					//std::vector<int> neighborhood = getNeighbors(ballCenter, _ballRadius);
					if (isEmpty(&center, _points, _pointNumber, _ballRadius))
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

BallCenter GpuAlgorithms::findSeed(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const std::vector<int> &_neighbors, const bool *_notUsed, const int _index0, const float _ballRadius)
{
	int blocks = 1;
	int threads = _neighbors.size();
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
	GpuUtils::createInDev<int>(&devNeighbors, &_neighbors[0], _neighbors.size());

	// Prepare global variable 'devFound'
	int found = 0;
	GpuUtils::setSymbol<int>(devFound, &found);

	// Prepare global variable 'devFoundCenter'
	BallCenter center = BallCenter();
	if (auxPtr == NULL)
		GpuUtils::createInDev<BallCenter>(&auxPtr, &center, 1);
	GpuUtils::setData<BallCenter>(&auxPtr, &center, 1);
	GpuUtils::setSymbol<BallCenter *>(devCenter, &auxPtr);

	// Execute kernel
	checkForSeeds<<<blocks, threads>>>(devPoints, _cloud->size(), devNeighbors, _neighbors.size(), devNotUsed, _index0, _ballRadius);

	// Retrieve found status (just for debug)
	//cudaMemcpyFromSymbol(&found, devFound, sizeof(int));
	//checkErrors("cudaMemcpyFromSymbol failed");

	// Retrieve results
	GpuUtils::getData<BallCenter>(&center, auxPtr, 1);

	// Free allocated memory
	cudaFree(devNeighbors);
	checkErrors("cudaFree devNeighbors failed");

	return center;
}
