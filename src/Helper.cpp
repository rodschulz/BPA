/**
 * Author: rodrigo
 * 2015
 */
#include "Helper.h"
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
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

	NormalEstimation<PointXYZ, Normal> normalEstimation;
	normalEstimation.setInputCloud(_cloud);
	normalEstimation.setRadiusSearch(_searchRadius);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*_normals);
}

void Helper::getCloudAndNormals(const string &_inputFile, PointCloud<PointXYZ>::Ptr &_cloud, PointCloud<Normal>::Ptr &_normals)
{
}
