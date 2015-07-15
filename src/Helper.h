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

#define COMPARISON_EPSILON	1e-10

// Sign function
template<typename T> int sign(T val)
{
	return (T(0) < val) - (val < T(0));
}

class Helper
{
public:
	static int getRandomNumber(const int _min, const int _max);

	static void removeNANs(pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud);
	static pcl::PointCloud<pcl::Normal>::Ptr getNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, const double _searchRadius = -1);
	static bool getCloudAndNormals(const std::string &_inputFile, pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const double _estimationRadius = -1);

	static bool isOriented(const Eigen::Vector3f &_normal, const Eigen::Vector3f &_normal0, const Eigen::Vector3f &_normal1, const Eigen::Vector3f &_normal2);
	static void fixNormals(pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud);

	static pcl::PointNormal makePointNormal(const float _x, const float _y, const float _z, const float _nx = 0, const float _ny = 0, const float _nz = 0, const float _curvature = 0);
	static pcl::PointNormal makePointNormal(const Eigen::Vector3f &_data);

private:
	Helper();
	~Helper();

	static pcl::PointCloud<pcl::PointNormal>::Ptr generateSurroundingSet(const pcl::PointNormal &_min, const pcl::PointNormal &_max);
};
