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

struct BallCenter
{
	float cx, cy, cz;
	int idx1, idx2, idx3;
	bool isValid;
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

__global__ void calculateBalls(const Point *_points, BallCenter *_balls, const int _initialRow, const int _pointsPerThread, const int _pointNumber)
{
	_balls[blockIdx.x].cx = blockDim.x;
	_balls[blockIdx.x].cy = blockDim.y;
	_balls[blockIdx.x].cz = blockDim.z;
}

bool CudaUtil::calculateBallCenters(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud)
{
	bool statusOk = true;

	size_t pointNumber = _cloud->size();
	BallCenter *devBalls;
	BallCenter *balls = (BallCenter*) calloc(pointNumber * pointNumber * pointNumber, sizeof(BallCenter));

	size_t cloudBytes = sizeof(pcl::PointNormal) * pointNumber;
	size_t resultBytes = sizeof(BallCenter) * pointNumber;
	float usageFactor = 0.733333333; // this is (2 * 1.1) / 2,  that is a 10% over 2/3 of all the available memory

	size_t freeMem = getAvailableMemory();
	std::cout << "Available mem: " << freeMem << std::endl;

	// Check if there's available at least the minimum amount of memory needed
	if (cloudBytes + resultBytes < freeMem * usageFactor)
	{
		// Alloc memory on the device and copy cloud data to it
		cudaMalloc((void **) &devPoints, cloudBytes);
		cudaCheckErrors("cudaMalloc 1 failed");
		cudaMemcpy(devPoints, &_cloud->points[0], cloudBytes, cudaMemcpyHostToDevice);
		cudaCheckErrors("cudaMemcpy to dev failed");

		freeMem = getAvailableMemory();
		std::cout << "Available mem: " << freeMem << std::endl;

		// Get max number of "rows" that cant be simultaneously processed
		int rowsPerCall = 0;
		while (rowsPerCall * resultBytes < freeMem * usageFactor)
			rowsPerCall++;

		if (rowsPerCall == 0)
		{
			cudaFree(devPoints);
			statusOk = false;
		}
		else
		{
			resultBytes = sizeof(BallCenter) * pointNumber * rowsPerCall;

			// Alloc memory for the results
			cudaMalloc((void **) &devBalls, resultBytes);
			cudaCheckErrors("cudaMalloc 2 failed");
			cudaMemset(devBalls, 0, resultBytes);
			cudaCheckErrors("cudaMemset failed");

			// Determine the number rows to be processed in each block and the number of points procesed by each thread
			int rowsPerBlock = ceil((float) rowsPerCall / BLOCKS);
			int pointsPerThread = ceil((float) rowsPerBlock * pointNumber / THREADS);

			// Process the data
			int totalRows = pointNumber * (pointNumber - 1);
			for (int initialRow = 0; initialRow < totalRows; initialRow += rowsPerCall)
			{
				calculateBalls<<<BLOCKS, THREADS>>>(devPoints, devBalls, initialRow, pointsPerThread, pointNumber);

				// Copy data back to host
				cudaMemcpy(balls, devBalls, resultBytes, cudaMemcpyDeviceToHost);
				cudaCheckErrors("cudaMemcpy to host failed");
			}
		}
	}
	else
		statusOk = false;

	// Free allocated memory
	free(balls);

	return statusOk;
}

__global__ void searchCloserPoints(const int _target, const Point *_points, const int _pointNumber, const double _searchRadius, const int _pointsPerThread, bool *_selected)
{
	int startIdx = (blockIdx.x * blockDim.x + threadIdx.x) * _pointsPerThread;
	double sqrRadius = _searchRadius * _searchRadius;

	if (startIdx < _pointNumber)
	{
		for (int i = startIdx; i < startIdx + _pointsPerThread; i++)
		{
			_selected[i] = _points[_target].sqrDist(_points[i]) < sqrRadius;
			//_selected[i] = _points[_target].dist(_points[i]) < _searchRadius;
		}
	}
}

bool CudaUtil::radiusSearch(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const int _target, double _radius, std::vector<int> _idxs)
{
	int blocks = 10;
	int threads = 10;
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
	cudaFree(devSelected);
	cudaCheckErrors("cudaFree selected failed");

//	std::vector<int> idxs;
	for (size_t i = 0; i < cloudSize; i++)
		if (selected[i])
			_idxs.push_back(i);

	free(selected);

	return true;
}
