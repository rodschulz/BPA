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

// Sign function
template<typename T> int sign(T val)
{
	return (T(0) < val) - (val < T(0));
}

struct DataHolder
{
	PointCloud<PointNormal>::Ptr cloud;
	vector<bool> used;

	DataHolder()
	{
		cloud.reset();
		used.clear();
	}

	DataHolder(PointCloud<PointNormal>::Ptr &_cloud)
	{
		cloud = _cloud;
		used = vector<bool>(_cloud->size(), false);
	}
};

class Helper
{
public:
	static void removeNANs(PointCloud<PointXYZ>::Ptr &_cloud);
	static PointCloud<Normal>::Ptr getNormals(const PointCloud<PointXYZ>::Ptr &_cloud, const double _searchRadius = -1);
	static bool getCloudAndNormals(const string &_inputFile, PointCloud<PointNormal>::Ptr &_cloud, const double _estimationRadius = -1);
	static int getActiveEdge(vector<Edge> &_front);

private:
	Helper();
	~Helper();
};
