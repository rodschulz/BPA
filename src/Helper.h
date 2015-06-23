/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include "Edge.h"
#include "Triangle.h"

using namespace std;
using namespace pcl;
using namespace Eigen;

#define COMPARISON_EPSILON	1e-10

// Sign function
template<typename T> int sign(T val)
{
	return (T(0) < val) - (val < T(0));
}

class Helper
{
public:
	static void removeNANs(PointCloud<PointXYZ>::Ptr &_cloud);
	static PointCloud<Normal>::Ptr getNormals(const PointCloud<PointXYZ>::Ptr &_cloud, const double _searchRadius = -1);
	static bool getCloudAndNormals(const string &_inputFile, PointCloud<PointNormal>::Ptr &_cloud, const double _estimationRadius = -1);

	static bool isOriented(const Vector3f &_normal, const Vector3f &_normal0, const Vector3f &_normal1, const Vector3f &_normal2);
	static void fixNormals(PointCloud<PointNormal>::Ptr &_cloud);

	static PointNormal makePointNormal(const float _x, const float _y, const float _z, const float _nx = 0, const float _ny = 0, const float _nz = 0, const float _curvature = 0);
	static PointNormal makePointNormal(const Vector3f &_data);

private:
	Helper();
	~Helper();

	static PointCloud<PointNormal>::Ptr generateSurroundingSet(const PointNormal &_min, const PointNormal &_max);
};
