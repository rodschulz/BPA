/**
 * Author: rodrigo
 * 2015
 */
#include "Helper.h"
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <ctype.h>

Helper::Helper()
{
}

Helper::~Helper()
{
}

void Helper::removeNANs(PointCloud<PointXYZ>::Ptr &_cloud)
{
	std::vector<int> mapping;
	removeNaNFromPointCloud(*_cloud, *_cloud, mapping);
}

PointCloud<Normal>::Ptr Helper::getNormals(const PointCloud<PointXYZ>::Ptr &_cloud, const double _searchRadius)
{
	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());

	search::KdTree<PointXYZ>::Ptr kdtree(new search::KdTree<PointXYZ>);
	NormalEstimationOMP<PointXYZ, Normal> normalEstimation;
	normalEstimation.setInputCloud(_cloud);

	if (_searchRadius > 0)
		normalEstimation.setRadiusSearch(_searchRadius);
	else
		normalEstimation.setKSearch(10);

	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	return normals;
}

bool Helper::getCloudAndNormals(const string &_inputFile, PointCloud<PointNormal>::Ptr &_cloud, const double _estimationRadius)
{
	bool status = false;

	PointCloud<PointXYZ>::Ptr dataXYZ(new PointCloud<PointXYZ>());
	if (io::loadPCDFile<PointXYZ>(_inputFile, *dataXYZ) == 0)
	{
		Helper::removeNANs(dataXYZ);
		PointCloud<Normal>::Ptr normals = Helper::getNormals(dataXYZ, _estimationRadius);

		_cloud->clear();
		concatenateFields(*dataXYZ, *normals, *_cloud);

		status = true;
	}
	else
		cout << "ERROR: Can't read file from disk (" << _inputFile << ")\n";

	return status;
}

int Helper::getActiveEdge(vector<Edge> &_front)
{
	for (size_t i = 0; i < _front.size(); i++)
	{
		if (_front[i].isActive())
		{
			return i;
		}
	}
	return -1;
}

PointNormal Helper::makePointNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature)
{
	PointNormal point;
	point.x = _x;
	point.y = _y;
	point.z = _z;
	point.normal_x = _nx;
	point.normal_y = _ny;
	point.normal_z = _nz;
	point.curvature = _curvature;
	return point;
}
