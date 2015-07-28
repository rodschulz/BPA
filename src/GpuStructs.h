/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <cuda_runtime.h>

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

	__host__ BallCenter(const BallCenter &_other)
	{
		cx = _other.cx;
		cy = _other.cy;
		cz = _other.cz;
		idx0 = _other.idx0;
		idx1 = _other.idx1;
		idx2 = _other.idx2;
		isValid = _other.isValid;
	}

	__device__ float sqrDist(const Point &_p) const
	{
		float dx = cx - _p.x;
		float dy = cy - _p.y;
		float dz = cz - _p.z;
		return dx * dx + dy * dy + dz * dz;
	}

	__device__ float dist(const Point &_p) const
	{
		float dx = cx - _p.x;
		float dy = cy - _p.y;
		float dz = cz - _p.z;
		return sqrt(dx * dx + dy * dy + dz * dz);
	}

	__device__ void add(const Point &_p)
	{
		cx += _p.x;
		cy += _p.y;
		cz += _p.z;
	}
};
