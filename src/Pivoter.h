/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <vector>
#include "Edge.h"
#include "Helper.h"
#include "Triangle.h"

using namespace pcl;
using namespace std;
using namespace Eigen;

class Pivoter
{
public:
	Pivoter(const PointCloud<PointNormal>::Ptr &_cloud, const double _ballRadius);
	~Pivoter();

	TrianglePtr findSeed();

private:
	pair<Vector3f, double> getCircumscribedCircle(const int _index0, const int _index1, const int _index2, Vector3f &_normal) const;
	inline bool getBallCenter(const pair<Vector3f, double> &_circumscribedCircle, const Vector3f &_normal, Vector3f &_ballCenter) const
	{
		bool status = false;

		double squaredDistance = ballRadius * ballRadius - _circumscribedCircle.second * _circumscribedCircle.second;
		if (squaredDistance > 0)
		{
			double distance = sqrt(fabs(squaredDistance));
			_ballCenter = _circumscribedCircle.first + distance * _normal;
			status = true;
		}

		return status;
	}
	bool isEmpty(const vector<int> &_data, const int _index0, const int _index1, const int _index2)
	{
		if (_data.size() > 3)
			return false;
		if (_data.empty())
			return true;

		for (size_t i = 0; i < _data.size(); i++)
		{
			if (_data[i] == _index0 || _data[i] == _index1 || _data[i] == _index2)
				continue;
			else
				return false;
		}
		return true;
	}

	KdTreeFLANN<PointNormal> kdtree;
	PointCloud<PointNormal>::Ptr cloud;
	vector<bool> used;
	double ballRadius;
};

