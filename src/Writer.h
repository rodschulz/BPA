/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>
#include "Triangle.h"

using namespace std;

#define OUTPUT_FOLDER		"./output/"
#define SPHERE_EXTENSION	".sph"
#define POLYGON_EXTENSION	".off"

class Writer
{
public:
	static void writeCircumscribedSphere(const string &_filename, const PointXYZ &_center, const double _radius, const Triangle &_triangle, const PointCloud<PointXYZ>::Ptr &_neighborhood, const bool _addSequential = false);
	static void writeMesh(const string &_filename, const vector<Triangle> &_meshData, const bool _addSequential = false);
	static void writeMesh(const string &_filename, const PointCloud<PointXYZ>::Ptr &_cloud, const vector<Triangle> &_meshData, const bool _addSequential = false);
	static void writeTriangle(const string &_filename, const Triangle &_triangle);

private:
	Writer();
	~Writer();

	static void generateMesh(const vector<Triangle> &_meshData, ofstream &_output);
	static void generateCloud(const PointCloud<PointXYZ>::Ptr &_cloud, ofstream &_output);
	static void generateSphere(const PointXYZ &_center, const double _radius, ofstream &_output);
	static void generateTriangle(const Triangle &_triangle, ofstream &_output);
};
