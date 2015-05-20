/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>
#include "Triangle.h"

using namespace std;
using namespace pcl;

#define OUTPUT_FOLDER		"./output/"
#define SPHERE_EXTENSION	".sph"
#define POLYGON_EXTENSION	".off"

class Writer
{
public:
	static void writeCircumscribedSphere(const string &_filename, const PointXYZ &_center, const double _radius, const Triangle &_triangle);
	static void writePolygon();
	static void writeTriangle(const string &_filename, const Triangle &_triangle);

private:
	Writer();
	~Writer();
};
