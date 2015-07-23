/**
 * Author: rodrigo
 * 2015
 */
#include "CudaUtil.h"
#include <ostream>
#include <iostream>

#define BLOCKS		20
#define THREADS		256

// Pointer to memory in device
struct Point;
Point *devPoints = NULL;
bool *devNotUsed = NULL;

struct BallCenter
{
	float cx, cy, cz;
	int idx1, idx2, idx3;
	bool isValid;

	__device__ BallCenter(const int _idx1, const int _idx2, const int _idx3)
	{
		idx1 = _idx1;
		idx2 = _idx2;
		idx3 = _idx3;
		cx = cy = cz = 0;
		isValid = false;
	}
};

struct Point
{
	float x, y, z, w;
	float nx, ny, nz, nw;
	float c;
	float fill[3];

	__device__ Point operator-(const Point &_p)
	{
		Point result;
		result.x = x - _p.x;
		result.y = y - _p.y;
		result.z = z - _p.z;
		return result;
	}

	__device__ double sqrDist(const Point &_p) const
	{
		double dx = x - _p.x;
		double dy = y - _p.y;
		double dz = z - _p.z;
		return dx * dx + dy * dy + dz * dz;
	}

	__device__ double dist(const Point &_p) const
	{
		double dx = x - _p.x;
		double dy = y - _p.y;
		double dz = z - _p.z;
		return sqrt(dx * dx + dy * dy + dz * dz);
	}
};

std::ostream &operator<<(std::ostream &_stream, const BallCenter &_center)
{
	_stream << "c=(" << _center.cx << ", " << _center.cy << ", " << _center.cz << ") / (" << _center.idx1 << ", " << _center.idx2 << ", " << _center.idx3 << ")";
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
__device__ bool getBallCenter(const Point *_p1, const Point *_p2, const Point *_p3, BallCenter *_center)
{
	return false;
}

__global__ void checkForSeeds(const Point *_points, const int _pointNumber, const int *_neighbors, const int _neighborsSize, const bool *_notUsed, const int _index0)
{
	int startIdx = 0;	//(blockIdx.x * blockDim.x + threadIdx.x) * _pointsPerThread;
	int endIdx = 0;	//calcular_esto;

	__shared__ bool found;
	found = false;

	//__syncthreads();

	for (int j = startIdx; j < endIdx && j < _neighborsSize; j++)
	{
		if (!found)
		{
			int index1 = _neighbors[j];

			// Skip invalid combinations
			if (index1 == _index0 || !_notUsed[index1])
				continue;

			for (size_t k = 0; k < _neighborsSize && !found; k++)
			{
				int index2 = _neighbors[k];

				// Skip invalid combinations
				if (index1 == index2 || index2 == _index0 || !_notUsed[index2])
					continue;

				BallCenter center(_index0, index1, index2);
				if (!found && getBallCenter(&_points[_index0], &_points[index1], &_points[index2], &center))
				{
//					pcl::PointNormal ballCenter = Helper::makePointNormal(center);
//					std::vector<int> neighborhood = getNeighbors(ballCenter, ballRadius);
//					if (!found && isEmpty(neighborhood, index0, index1, index2, center))
//					{
//						if (!found)
//						{
//
//							ESTO TIENE
//							QUE SER
//							EN UN
//							BLOQUE CON
//							MUTEX
//							!
//
//							seed = TrianglePtr(new Triangle(cloud->at((int) sequence[0]), cloud->at((int) sequence[1]), cloud->at((int) sequence[2]), sequence[0], sequence[1], sequence[2], ballCenter, ballRadius));
//							devNotUsed.erase(index0);
//							devNotUsed.erase(index1);
//							devNotUsed.erase(index2);
//
//							found = true;
//
//							break;
//						}
//					}
				}
			}
		}
	}
}

bool CudaUtil::findSeed(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const std::vector<int> &_neighbors, const bool *_notUsed, const int _index0)
{
	int blocks = 10;
	int threads = 256;
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
	size_t neighborsBytes = sizeof(int) * _neighbors.size();
	cudaMalloc((void **) &devNeighbors, neighborsBytes);
	cudaCheckErrors("cudaMalloc neighbors failed");
	cudaMemcpy(devNeighbors, &_neighbors[0], neighborsBytes, cudaMemcpyHostToDevice);
	cudaCheckErrors("cudaMemcpy neighbors to dev failed");

	checkForSeeds<<<1, 1>>>(devPoints, _cloud->size(), devNeighbors, _neighbors.size(), devNotUsed, _index0);

	return true;
}
