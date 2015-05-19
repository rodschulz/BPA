/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include "Edge.h"

using namespace std;
using namespace pcl;

class Helper
{
public:
	static void removeNANs(PointCloud<PointXYZ>::Ptr &_cloud);
	static void getNormals(const PointCloud<PointXYZ>::Ptr &_cloud, const double _searchRadius, PointCloud<Normal>::Ptr &_normals);
	static bool getCloudAndNormals(const string &_inputFile, PointCloud<PointXYZ>::Ptr &_cloud, PointCloud<Normal>::Ptr &_normals);
	static bool getActiveEdge(vector <Edge> &_front, Edge &_edge);

private:
	Helper();
	~Helper();
};
