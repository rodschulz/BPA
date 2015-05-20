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

struct DataHolder
{
	PointCloud<PointXYZ>::Ptr cloud;
	PointCloud<Normal>::Ptr normals;
	vector<bool> used;

	DataHolder()
	{
		cloud.reset();
		normals.reset();
		used.clear();
	}

	DataHolder(PointCloud<PointXYZ>::Ptr _cloud, PointCloud<Normal>::Ptr _normals)
	{
		cloud = _cloud;
		normals = _normals;
		used = vector<bool>(_cloud->size(), false);
	}
};

class Helper
{
public:
	static void removeNANs(PointCloud<PointXYZ>::Ptr &_cloud);
	static void getNormals(const PointCloud<PointXYZ>::Ptr &_cloud, const double _searchRadius, PointCloud<Normal>::Ptr &_normals);
	static bool getCloudAndNormals(const string &_inputFile, PointCloud<PointXYZ>::Ptr &_cloud, PointCloud<Normal>::Ptr &_normals);
	static bool getActiveEdge(vector<Edge> &_front, Edge &_edge);

private:
	Helper();
	~Helper();
};
