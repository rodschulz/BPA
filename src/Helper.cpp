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

void Helper::getNormals(const PointCloud<PointXYZ>::Ptr &_cloud, const double _searchRadius, PointCloud<Normal>::Ptr &_normals)
{
	// Search method used for the knn search
	search::KdTree<PointXYZ>::Ptr kdtree(new search::KdTree<PointXYZ>);

	NormalEstimationOMP<PointXYZ, Normal> normalEstimation;
	normalEstimation.setInputCloud(_cloud);
	normalEstimation.setRadiusSearch(_searchRadius);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*_normals);
}

bool Helper::getCloudAndNormals(const string &_inputFile, PointCloud<PointXYZ>::Ptr &_cloud, PointCloud<Normal>::Ptr &_normals)
{
	bool status = false;

	if (io::loadPCDFile<PointXYZ>(_inputFile, *_cloud) != 0)
		cout << "ERROR: Can't read file from disk (" << _inputFile << ")\n";
	else
	{
		// Remove NANs and calculate normals
		Helper::removeNANs(_cloud);
		Helper::getNormals(_cloud, 0.01, _normals);
		status = true;
	}

	return status;
}

bool Helper::getActiveEdge(vector<Edge> &_front, Edge *_edge)
{
	bool status = false;

	for (size_t i = 0; i < _front.size(); i++)
	{
		if (_front[i].isActive())
		{
			_edge = &_front[i];
			status = true;
			break;
		}
	}

	return status;
}
