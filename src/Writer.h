/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <map>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include "Triangle.h"

#define OUTPUT_FOLDER		"./output/"
#define SPHERE_EXTENSION	".sph"
#define POLYGON_EXTENSION	".off"

#define SSTR(x)			dynamic_cast< std::stringstream &>((std::stringstream() << std::dec << std::setfill('0') << std::setw(5) << x)).str()

enum WriteOptions
{
	ADD_SEQUENTIAL		= 0X01,
	DRAW_NORMALS 		= 0X02,
	DRAW_CLOUD 		= 0X04,
	DRAW_SPHERES		= 0x08,
};

#define addSequential(x)	((ADD_SEQUENTIAL & (x)) == ADD_SEQUENTIAL)
#define drawNormals(x)		((DRAW_NORMALS & (x)) == DRAW_NORMALS)
#define drawCloud(x)		((DRAW_CLOUD & (x)) == DRAW_CLOUD)
#define drawSphere(x)		((DRAW_SPHERES & (x)) == DRAW_SPHERES)

class Writer
{
public:
	static void writeCircumscribedSphere(const std::string &_filename, const Eigen::Vector3f &_center, const double _radius, const Triangle &_triangle, const pcl::PointCloud<pcl::PointNormal>::Ptr &_neighborhood, const bool _addSequential = false);
	static void writeMesh(const std::string &_filename, const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const std::vector<TrianglePtr> &_meshData, const int _mask = 0x00);
	static void writeMesh(const std::string &_filename, const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const std::vector<TrianglePtr> &_meshData, const TrianglePtr &_seed, const int _mask = 0x00);
	static void writeMesh(const std::string &_filename, const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const std::vector<TrianglePtr> &_meshData, const EdgePtr &_boundary, const int _mask = 0x00);
	static void writeTriangle(const std::string &_filename, const Triangle &_triangle);

private:
	Writer();
	~Writer();

	static void generateMesh(const std::vector<TrianglePtr> &_meshData, std::ofstream &_output);
	static void generateCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, std::ofstream &_output);
	static void generateSphere(const Eigen::Vector3f &_center, const double _radius, std::ofstream &_output);
	static void generateTriangle(const Triangle &_triangle, std::ofstream &_output);
	static void generateTriangleFace(const TrianglePtr &_triangle, std::ofstream &_output);
	static void generateNormals(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, std::ofstream &_output);
	static void generateEdge(const EdgePtr &_boundary, std::ofstream &_output);

	static inline std::string generateName(const std::string &_name, const std::string &_extension, bool _addSequential = false)
	{
		static int sequential = 0;

		std::string name = OUTPUT_FOLDER;
		if (_addSequential)
		{
			name += SSTR(sequential++)+ "_";
		}
		name += _name + _extension;

		return name;
	}
};
