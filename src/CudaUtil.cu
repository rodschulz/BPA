/**
 * Author: rodrigo
 * 2015
 */
#include "CudaUtil.h"
#include <cuda_runtime.h>
#include <ostream>
#include <iostream>

#define BLOCKS		20
#define THREADS		256

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

	__host__ __device__ Point()
	{
		x = y = z = w = nx = ny = nz = nw = c = fill[0] = fill[1] = fill[2] = 0;
	}

	__device__ Point operator-(const Point &_p)
	{
		Point result;
		result.x = x - _p.x;
		result.y = y - _p.y;
		result.z = z - _p.z;
		return result;
	}
};

std::ostream &operator<<(std::ostream &_stream, const BallCenter &_center)
{
	_stream << "c=(" << _center.cx << ", " << _center.cy << ", " << _center.cz << ") / (" << _center.idx1 << ", " << _center.idx2 << ", " << _center.idx3 << ")";
	return _stream;
}

size_t getAvailableMemory()
{
	size_t freeMem, totalMem;
	cudaMemGetInfo(&freeMem, &totalMem);
	cudaCheckErrors("memInfo failed");
	return freeMem;
}

__global__ void calculateBalls(const Point *_points, BallCenter *_balls, const int _initialRow, const int _pointsPerThread, const int _pointNumber)
{


//	// Begin and end points
//	int beginPoint = ptsPerBlock * blockIdx.x + ptsPerThread * threadIdx.x;
//	int endPoint = beginPoint + ptsPerThread;
//
//	for (int i = beginPoint; i < endPoint; i++)
//	{
//		for (int j = 0; j < _pointNumber; j++)
//		{
//			for (int k = 0; k < _pointNumber; k++)
//			{
//			}
//		}
//	}
	_balls[blockIdx.x].cx = blockDim.x;
	_balls[blockIdx.x].cy = blockDim.y;
	_balls[blockIdx.x].cz = blockDim.z;

//	for (int i = )
//	{}

//	bool status = false;
//
//	Eigen::Vector3f p0 = cloud->at(_index0).getVector3fMap();
//	Eigen::Vector3f p1 = cloud->at(_index1).getVector3fMap();
//	Eigen::Vector3f p2 = cloud->at(_index2).getVector3fMap();
//	_sequence = Eigen::Vector3i(_index0, _index1, _index2);
//
//	Eigen::Vector3f v10 = p1 - p0;
//	Eigen::Vector3f v20 = p2 - p0;
//	Eigen::Vector3f normal = v10.cross(v20);
//
//	// Calculate ball center only if points are not collinear
//	if (normal.norm() > COMPARISON_EPSILON)
//	{
//	}
}

bool CudaUtil::calculateBallCenters(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud)
{
	bool statusOk = true;

	size_t pointNumber = _cloud->size();
	Point *devPoints;
	BallCenter *devBalls;
	BallCenter *balls = (BallCenter*) calloc(pointNumber * pointNumber * pointNumber, sizeof(BallCenter));

	size_t totalB = pointNumber * pointNumber * pointNumber * sizeof(BallCenter);
	size_t totaMB = totalB / 1E6;
	size_t totaGB = totalB / 1E9;

	size_t cloudBytes = sizeof(pcl::PointNormal) * pointNumber;
	size_t rowLength = pointNumber - 2;
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
