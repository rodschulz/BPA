/**
 * Author: rodrigo
 * 2015
 */
#include "CudaUtil.h"
#include <ostream>
#include <iostream>
#include "UtilsGPU.h"

#define COMPARISON_EPSILON	1E-8

struct Point
{
	float x, y, z, w;
	float nx, ny, nz, nw;
	float c;
	float fill[3];

	__device__ Point operator+(const Point &_p) const
	{
		Point result;
		result.x = x + _p.x;
		result.y = y + _p.y;
		result.z = z + _p.z;
		return result;
	}

	__device__ Point operator-(const Point &_p) const
	{
		Point result;
		result.x = x - _p.x;
		result.y = y - _p.y;
		result.z = z - _p.z;
		return result;
	}

	__device__ Point operator*(const float _scalar) const
	{
		Point result;
		result.x = x * _scalar;
		result.y = y * _scalar;
		result.z = z * _scalar;
		return result;
	}

	__device__ float sqrDist(const Point &_p) const
	{
		float dx = x - _p.x;
		float dy = y - _p.y;
		float dz = z - _p.z;
		return dx * dx + dy * dy + dz * dz;
	}

	__device__ float dist(const Point &_p) const
	{
		float dx = x - _p.x;
		float dy = y - _p.y;
		float dz = z - _p.z;
		return sqrt(dx * dx + dy * dy + dz * dz);
	}

	__device__ Point cross(const Point &_p) const
	{
		Point result;
		result.x = y * _p.z - z * _p.y;
		result.x = z * _p.x - x * _p.z;
		result.x = x * _p.y - y * _p.x;
		return result;
	}

	__device__ float dot(const Point &_p) const
	{
		return (x * _p.x) + (y * _p.y) + (z * _p.z);
	}

	__device__ float normalDot(const Point &_p) const
	{
		return (nx * _p.x) + (ny * _p.y) + (nz * _p.z);
	}

	__device__ float sqrNorm() const
	{
		return x * x + y * y + z * z;
	}

	__device__ float norm() const
	{
		return sqrt(x * x + y * y + z * z);
	}

	__device__ void normalize()
	{
		float factor = 1 / sqrt(x * x + y * y + z * z);
		x *= factor;
		y *= factor;
		z *= factor;
	}
};

struct BallCenter
{
	float cx, cy, cz;
	int idx0, idx1, idx2;
	bool isValid;

	__device__ __host__ BallCenter()
	{
		cx = cy = cz = 0;
		idx0 = idx1 = idx2 = 0;
		isValid = false;
	}

	__device__ BallCenter(const int _idx0, const int _idx1, const int _idx2)
	{
		idx0 = _idx0;
		idx1 = _idx1;
		idx2 = _idx2;
		cx = cy = cz = 0;
		isValid = false;
	}

	__device__ float sqrDist(const Point &_p) const
	{
		float dx = cx - _p.x;
		float dy = cy - _p.y;
		float dz = cz - _p.z;
		return dx * dx + dy * dy + dz * dz;
	}

	__device__ void add(const Point &_p)
	{
		cx += _p.x;
		cy += _p.y;
		cz += _p.z;
	}
};

// Pointers to memory in device
Point *devPoints = NULL;
bool *devNotUsed = NULL;

// Global variable in device
__device__ int devFound;
__device__ BallCenter *devCenter;

std::ostream &operator<<(std::ostream &_stream, const BallCenter &_center)
{
	_stream << "c=(" << _center.cx << ", " << _center.cy << ", " << _center.cz << ") / (" << _center.idx0 << ", " << _center.idx1 << ", " << _center.idx2 << ")";
	return _stream;
}

void CudaUtil::allocPoints(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud)
{
	size_t cloudBytes = sizeof(pcl::PointNormal) * _cloud->size();
	cudaMalloc((void **) &devPoints, cloudBytes);
	cudaCheckErrors("cudaMalloc points failed");

	cudaMemcpy(devPoints, &_cloud->points[0], cloudBytes, cudaMemcpyHostToDevice);
	cudaCheckErrors("cudaMemcpy points to dev failed");
}

void CudaUtil::allocUsed(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const bool* _notUsed)
{
	size_t bytes = sizeof(bool) * _cloud->size();
	cudaMalloc((void **) &devNotUsed, bytes);
	cudaCheckErrors("cudaMalloc notUsed failed");

	cudaMemset(devNotUsed, 0, bytes);
	cudaCheckErrors("cudaMemset notUsed failed");
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

bool CudaUtil::radiusSearch(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const int _target, double _radius, std::vector<int> &_idxs)
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
	cudaCheckErrors("cudaMalloc selected failed");

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
	cudaCheckErrors("cudaMemcpy selected failed");
	//cudaFree(devSelected);
	//cudaCheckErrors("cudaFree selected failed");

	for (size_t i = 0; i < cloudSize; i++)
		if (selected[i])
			_idxs.push_back(i);

	free(selected);

	return true;
}

///////////////////////////////
__device__ bool isOriented(const Point &_normal, const Point &_p0, const Point &_p1, const Point &_p2)
{
	int count = 0;
	count = _p0.normalDot(_normal) < 0 ? count + 1 : count;
	count = _p1.normalDot(_normal) < 0 ? count + 1 : count;
	count = _p2.normalDot(_normal) < 0 ? count + 1 : count;

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
	if (normal.sqrNorm() > COMPARISON_EPSILON)
	{
		// Normalize to avoid precision errors while checking the orientation
		normal.normalize();
		if (!isOriented(normal, *p0, *p1, *p2))
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
	bool empty = true;
	for (int i = 0; i < _pointNumber && empty; i++)
	{
		if (_center->sqrDist(_points[i]) < _ballRadius)
			empty = false;
	}
	return empty;
}

__global__ void checkForSeeds(const Point *_points, const int _pointNumber, const int *_neighbors, const int _neighborsSize, const bool *_notUsed, const int _index0)
{
	int startIdx = threadIdx.x;
	int endIdx = startIdx + 1;
	float _ballRadius = 0.005;

	devCenter->cy = devFound;
	devFound = devCenter->cx;
	devCenter->idx0 = _neighbors[0];
	devCenter->idx1 = _neighbors[1];
	devCenter->idx2 = _neighbors[2];
	devCenter->isValid = true;

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

bool CudaUtil::findSeed(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const std::vector<int> &_neighbors, const bool *_notUsed, const int _index0)
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
	cudaMemcpy(devNotUsed, _notUsed, notUsedBytes, cudaMemcpyHostToDevice);
	cudaCheckErrors("cudaMemcpy notUsed to dev failed");

	// Create and prepare buffer with neighbors indices
	int *devNeighbors;
	UtilsGPU::createInDev<int>(&devNeighbors, &_neighbors[0], _neighbors.size());

	// Prepare global variable 'devFound'
	int found = 7;
	UtilsGPU::setSymbol<int>(devFound, &found);

	// Prepare global variable 'devFoundCenter'
	BallCenter *center = new BallCenter();
	center->cx = 1313;
	BallCenter *auxPtr;
	UtilsGPU::createInDev<BallCenter>(&auxPtr, center, 1);
	UtilsGPU::setSymbol<BallCenter *>(devCenter, &auxPtr);

	// Execute kernel
	checkForSeeds<<<blocks, threads>>>(devPoints, _cloud->size(), devNeighbors, _neighbors.size(), devNotUsed, _index0);

	// Retrieve results
	cudaMemcpyFromSymbol(&found, devFound, sizeof(int));
	cudaCheckErrors("cudaMemcpyFromSymbol devFound failed");
	cudaMemcpy(center, auxPtr, sizeof(BallCenter), cudaMemcpyDeviceToHost);
	cudaCheckErrors("cudaMemcpy auxPtr failed");

	// Free allocated memory
	cudaFree(devNotUsed);
	cudaCheckErrors("cudaFree devNotUsed failed");
	cudaFree(devNeighbors);
	cudaCheckErrors("cudaFree devNeighbors failed");

	return true;
}
