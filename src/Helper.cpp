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
#include <pcl/filters/passthrough.h>

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
	NormalEstimation<PointXYZ, Normal> normalEstimation;
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

		// Create the filtering object
//		PointCloud<PointNormal>::Ptr filtered(new PointCloud<PointNormal>());
//		pcl::PassThrough<pcl::PointNormal> pass;
//		pass.setInputCloud(_cloud);
//		pass.setFilterFieldName("z");
//		pass.setFilterLimits(0.0, 1e20);
//		pass.filter(*filtered);
//
//		pass.setInputCloud(filtered);
//		pass.setFilterFieldName("x");
//		//pass.setFilterLimitsNegative (true);
//		pass.setFilterLimits(-100, 1);
//		pass.filter(*filtered);
//
//		pass.setInputCloud(filtered);
//		pass.setFilterFieldName("y");
//		//pass.setFilterLimitsNegative (true);
//		pass.setFilterLimits(0.04, 100);
//		pass.filter(*filtered);
//
//		io::savePCDFileASCII("./cloud.pcd", *_cloud);
//		io::savePCDFileASCII("./filtered.pcd", *filtered);

		status = true;
	}
	else
		cout << "ERROR: Can't read file from disk (" << _inputFile << ")\n";

	return status;
}

bool Helper::isOriented(const Vector3f &_normal, const Vector3f &_normal0, const Vector3f &_normal1, const Vector3f &_normal2)
{
	int count = 0;
	count = _normal0.dot(_normal) < 0 ? count + 1 : count;
	count = _normal1.dot(_normal) < 0 ? count + 1 : count;
	count = _normal2.dot(_normal) < 0 ? count + 1 : count;

	return count <= 1;
}

void Helper::fixNormals(PointCloud<PointNormal>::Ptr &_cloud)
{
	PointNormal minLimit, maxLimit;
	getMinMax3D(*_cloud, minLimit, maxLimit);

	//io::savePCDFileASCII("./orig.pcd", *_cloud);

	// Create the surrounding reference points
	PointCloud<PointNormal>::Ptr surrounds = generateSurroundingSet(minLimit, maxLimit);
	//io::savePCDFileASCII("./surround.pcd", *surrounds);

	KdTreeFLANN<pcl::PointNormal> kdtree;
	kdtree.setInputCloud(surrounds);

	int K = 3;
	for (size_t i = 0; i < _cloud->size(); i++)
	{
		PointNormal point = _cloud->at(i);

		if (i == 16749)
			int x = 11;

		vector<int> indices(K);
		vector<float> distances(K);
		if (kdtree.nearestKSearch(point, K, indices, distances) > 0)
		{
			int flip = 0;
			for (size_t j = 0; j < indices.size(); j++)
			{
				Vector3f direction = surrounds->at(indices[j]).getVector3fMap() - point.getVector3fMap();
				if (point.getNormalVector3fMap().dot(direction) < 0)
					flip += 1;
			}

			if (flip > K / 2)
			{
				point.normal_x *= -1;
				point.normal_y *= -1;
				point.normal_z *= -1;
				_cloud->at(i) = point;
			}
		}
	}
	//io::savePCDFileASCII("./fixed.pcd", *_cloud);
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

PointNormal Helper::makePointNormal(const Vector3f &_data)
{
	PointNormal point;
	point.x = _data.x();
	point.y = _data.y();
	point.z = _data.z();
	point.normal_x = 0;
	point.normal_y = 0;
	point.normal_z = 0;
	point.curvature = 0;
	return point;
}

PointCloud<PointNormal>::Ptr Helper::generateSurroundingSet(const PointNormal &_min, const PointNormal &_max)
{
	PointCloud<PointNormal>::Ptr surrounding(new PointCloud<PointNormal>());
	float factor = 0.25;

	float deltax = _max.x - _min.x;
	float deltay = _max.y - _min.y;
	float deltaz = _max.z - _min.z;

	float mx = _min.x - deltax * factor;
	float my = _min.y - deltay * factor;
	float mz = _min.z - deltaz * factor;

	float Mx = _max.x + deltax * factor;
	float My = _max.y + deltay * factor;
	float Mz = _max.z + deltaz * factor;

	float mMx = (mx + Mx) * 0.5;
	float mMy = (my + My) * 0.5;
	float mMz = (mz + Mz) * 0.5;

	// Add back face's points
	surrounding->push_back(makePointNormal(mx, my, mz));
	//surrounding->push_back(makePointNormal(mx, mMy, mz));
	surrounding->push_back(makePointNormal(mx, My, mz));

	//surrounding->push_back(makePointNormal(mx, my, mMz));
	//surrounding->push_back(makePointNormal(mx, mMy, mMz));
	//surrounding->push_back(makePointNormal(mx, My, mMz));

	surrounding->push_back(makePointNormal(mx, my, Mz));
	//surrounding->push_back(makePointNormal(mx, mMy, Mz));
	surrounding->push_back(makePointNormal(mx, My, Mz));

	// Add middle line
	//surrounding->push_back(makePointNormal(mMx, my, mz));
	//surrounding->push_back(makePointNormal(mMx, mMy, mz));
	//surrounding->push_back(makePointNormal(mMx, My, mz));

	//surrounding->push_back(makePointNormal(mMx, my, mMz));
	//surrounding->push_back(makePointNormal(mMx, My, mMz));

	//surrounding->push_back(makePointNormal(mMx, my, Mz));
	//surrounding->push_back(makePointNormal(mMx, mMy, Mz));
	//surrounding->push_back(makePointNormal(mMx, My, Mz));

	// Add front face's points
	surrounding->push_back(makePointNormal(Mx, my, mz));
	//surrounding->push_back(makePointNormal(Mx, mMy, mz));
	surrounding->push_back(makePointNormal(Mx, My, mz));

	//surrounding->push_back(makePointNormal(Mx, my, mMz));
	//surrounding->push_back(makePointNormal(Mx, mMy, mMz));
	//surrounding->push_back(makePointNormal(Mx, My, mMz));

	surrounding->push_back(makePointNormal(Mx, my, Mz));
	//surrounding->push_back(makePointNormal(Mx, mMy, Mz));
	surrounding->push_back(makePointNormal(Mx, My, Mz));

	return surrounding;
}

