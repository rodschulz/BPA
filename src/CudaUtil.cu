/**
 * Author: rodrigo
 * 2015
 */
#include "CudaUtil.h"
#include <cuda_runtime.h>
#include <ostream>
#include <iostream>

#define BLOCKS		10
#define THREADS		512
#define N		

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

	__host__ __device__ BallCenter()
	{
		cx = cy = cz = 0;
		idx1 = idx2 = idx3 = 0;
	}
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

__global__ void calculateBalls(const Point *_points, BallCenter *_balls, const int _pointNumber)
{
	for (int i = 0; i < N; i++)
	{
		_balls[i].cx = i * i;
		_balls[i].cy = i;
		_balls[i].cz = i;
		_balls[i].idx1 = _pointNumber;
	}
}


__global__ void test1(const Point *_points, BallCenter *_balls, const int _pointNumber)
{
	_balls[0].cx = blockIdx.x * 100 + i;
}

__global__ void test2(const Point *_points, BallCenter *_balls, const int _pointNumber)
{
	_balls[0].cx = threadIdx.x * 100 + i;
}

void CudaUtil::calculateBallCenters(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud)
{
	size_t cloudSize = _cloud->size();

	size_t pn = sizeof(pcl::PointNormal);
	size_t bc = sizeof(BallCenter);
	size_t p = sizeof(Point);
	
	/** USE NEW INSTEAD OF MALLOC!! or maybe calloc */
	
	Point *devPoints;
	BallCenter *devBalls;
	BallCenter *balls = (BallCenter*) malloc(sizeof(BallCenter) * cloudSize * cloudSize * cloudSize);
	


	// Alloc memory on the device and copy cloud data to it
	cudaMalloc((void **) &devPoints, sizeof(pcl::PointNormal) * cloudSize);
	cudaCheckErrors("cudaMalloc 1 failed");
	cudaMemcpy(devPoints, &_cloud->points[0], sizeof(pcl::PointNormal) * cloudSize, cudaMemcpyHostToDevice);
	cudaCheckErrors("cudaMemcpy to dev failed");

	// Alloc memory for the results
	cudaMalloc((void **) &devBalls, sizeof(BallCenter) * cloudSize * cloudSize * cloudSize);
	cudaCheckErrors("cudaMalloc 2 failed");

	// Determine the number or threads and blocks to use
	//...

	//calculateBalls<<<1, 1>>>(devPoints, devBalls, cloudSize);
	test1<<<5,1>>>(devPoints, devBalls, cloudSize);
	
	// Alloc memory on host
	cudaMemcpy(balls, devBalls, sizeof(BallCenter) * cloudSize * cloudSize * cloudSize, cudaMemcpyDeviceToHost);
	cudaCheckErrors("cudaMemcpy to host failed");

	for (int i = 0; i < N; i++)
		std::cout << balls[i] << std::endl;


	test2<<<1,5>>>(devPoints, devBalls, cloudSize);
	
	// Alloc memory on host
	cudaMemcpy(balls, devBalls, sizeof(BallCenter) * cloudSize * cloudSize * cloudSize, cudaMemcpyDeviceToHost);
	cudaCheckErrors("cudaMemcpy to host failed");

	for (int i = 0; i < N; i++)
		std::cout << balls[i] << std::endl;




	int x = 0;
}
