/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <cuda_runtime.h>
#include "GpuUtils.h"

namespace gpu
{
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
			result.y = z * _p.x - x * _p.z;
			result.z = x * _p.y - y * _p.x;
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

	struct DeviceNode
	{
		int point;
		int left;
		int right;

		DeviceNode()
		{
			point = left = right = -1;
		}
	};

	struct DeviceKDTree
	{
		const Point *points;
		DeviceNode *nodes;
		int size;
		int root;

		DeviceKDTree()
		{
			points = NULL;
			nodes = NULL;
			size = 0;
			root = -1;
		}

		__device__ void radiusSearch(const Point *_target, const float _radius)
		{
		}
	};

	struct HostNode
	{
		int point;
		HostNode *left, *right;

		HostNode()
		{
			point = -1;
			left = right = NULL;
		}

		HostNode(const int _index)
		{
			point = _index;
			left = right = NULL;
		}
	};

	struct HostKDTree
	{
		const Point *points;
		HostNode *root;
		int size;

		HostKDTree(const Point *_points)
		{
			points = _points;
			root = NULL;
			size = 0;
		}

		void insert(const Point *_point, const int _index)
		{
			insert(&root, _point, _index, 0);
		}

		DeviceKDTree buildDeviceTree(DeviceNode *_devNodesMem, const Point *_devPointsMem)
		{
			// Create a host piece of memory to build the tree
			DeviceNode *hostMem = new DeviceNode[size];
			int startIndex = 0;
			buildDeviceTree(root, hostMem, startIndex);

			// Copy the tree data to gpu
			GpuUtils::setData<DeviceNode>(&_devNodesMem, hostMem, size);

			// Free the host memory allocated for construction
			delete hostMem;

			DeviceKDTree tree = DeviceKDTree();
			tree.points = _devPointsMem;
			tree.nodes = _devNodesMem;
			tree.root = 0;
			tree.size = size;

			return tree;
		}

	private:
		void buildDeviceTree(const HostNode *_root, DeviceNode *_mem, int &_addingIndex)
		{
			if (_root != NULL)
			{
				int currentNodeIndex = _addingIndex;
				_mem[currentNodeIndex].point = _root->point;

				// Set the left child
				if (_root->left != NULL)
				{
					_mem[currentNodeIndex].left = _addingIndex + 1;
					buildDeviceTree(_root->left, _mem, ++_addingIndex);
				}
				else
					_mem[currentNodeIndex].left = -1;

				// Set the right child
				if (_root->right != NULL)
				{
					_mem[currentNodeIndex].right = _addingIndex + 1;
					buildDeviceTree(_root->right, _mem, ++_addingIndex);
				}
				else
					_mem[currentNodeIndex].right = -1;
			}
		}

		void insert(HostNode **root, const Point *_point, const int _index, const int _dimesion)
		{
			if (*root == NULL)
			{
				*root = new HostNode(_index);
				size++;
			}
			else
			{
				bool goLeft = true;
				switch ((_dimesion % 3))
				{
					case 0: // x
						goLeft = _point->x <= points[(*root)->point].x;
						break;
					case 1: // y
						goLeft = _point->y <= points[(*root)->point].y;
						break;
					case 2: // z
						goLeft = _point->z <= points[(*root)->point].z;
						break;
				}

				if (goLeft)
					insert(&((*root)->left), _point, _index, _dimesion + 1);
				else
					insert(&((*root)->right), _point, _index, _dimesion + 1);
			}
		}
	};
}
