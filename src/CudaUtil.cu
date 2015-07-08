/**
 * Author: rodrigo
 * 2015
 */
#include "CudaUtil.h"
#include <cuda_runtime.h>
#include <ostream>
#include <iostream>

#define N	20
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
	float radius;
};

struct Point
{
	float x, y, z, w;
	float nx, ny, nz, nw;
	float c;
	float fill[3];
};

std::ostream &operator<<(std::ostream &_stream, const BallCenter &_center)
{
	_stream << "c=(" << _center.cx << ", " << _center.cy << ", " << _center.cz << ") / r=" << _center.radius;
	return _stream;
}

__global__ void calculateBalls(Point *_points, BallCenter *_balls)
{
	for (int i = 0; i < N; i++)
	{
		_balls[i].cx = i;
		_balls[i].cy = i;
		_balls[i].cz = i;
		_balls[i].radius = _points[i].x;
	}
}

void CudaUtil::calculateBallCenters(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud)
{
	size_t cloudSize = _cloud->size();

	// Alloc memory on the device and copy cloud data to it
	Point *devPoints;
	cudaMalloc((void **) &devPoints, sizeof(pcl::PointNormal) * cloudSize);
	cudaCheckErrors("cudaMalloc 1 failed");

	cudaMemcpy(devPoints, &_cloud->points[0], sizeof(pcl::PointNormal) * cloudSize, cudaMemcpyHostToDevice);
	cudaCheckErrors("cudaMemcpy to dev failed");

	// Alloc memory for the results
	BallCenter *devBalls;
	cudaMalloc((void **) &devBalls, sizeof(BallCenter) * cloudSize * cloudSize * cloudSize);
	cudaCheckErrors("cudaMalloc 2 failed");

	calculateBalls<<<1, 1>>>(devPoints, devBalls);

	// Alloc memory on host
	BallCenter *balls = (BallCenter*) malloc(sizeof(BallCenter) * cloudSize * cloudSize * cloudSize);
	cudaMemcpy(balls, devBalls, sizeof(BallCenter) * cloudSize * cloudSize * cloudSize, cudaMemcpyDeviceToHost);
	cudaCheckErrors("cudaMemcpy to host failed");

	for (int i = 0; i < N; i++)
		std::cout << balls[i] << std::endl;

	int x = 0;
}
