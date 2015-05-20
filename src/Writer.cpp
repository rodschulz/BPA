/**
 * Author: rodrigo
 * 2015
 */
#include "Writer.h"
#include <fstream>

Writer::Writer()
{
}

Writer::~Writer()
{
}

void Writer::writeSphere(const string &_filename, const PointXYZ &_center, const double _radius)
{
	ofstream output;
	string name = OUTPUT_FOLDER + _filename + SPHERE_EXTENSION;
	output.open(name.c_str(), fstream::out);
	output.precision(15);

	output << fixed;
	output << "STESPHERE\n";
	output << _radius << "\n";
	output << _center.x << " " << _center.y << " " << _center.z << "\n";

	output.close();
}

void Writer::writePolygon()
{
}

void Writer::writeTriangle(const string &_filename, const Triangle &_triangle)
{
	ofstream output;
	string name = OUTPUT_FOLDER + _filename + POLYGON_EXTENSION;
	output.open(name.c_str(), fstream::out);
	output.precision(10);

	output << fixed;
	output << "OFF\n";
	output << 3 << " " << 1 << " " << 3 << "\n";

	for (size_t i = 0; i < 3; i++)
	{
		PointXYZ *p = _triangle.getVertex(i);
		output << p->x << " " << p->y << " " << p->z << "\n";
	}
	output << 3 << " " << 0 << " " << 1 << " " << 2 << " " << 0 << "\n";

	output.close();
}
