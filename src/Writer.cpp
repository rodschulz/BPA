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

void Writer::writeCircumscribedSphere(const string &_filename, const PointXYZ &_center, const double _radius, const Triangle &_triangle)
{
	ofstream output;
	string name = OUTPUT_FOLDER + _filename + POLYGON_EXTENSION;
	output.open(name.c_str(), fstream::out);
	output.precision(15);

	output << fixed;
	output << "appearance { -face +edge }\nLIST\n";

	// Generate a line to draw the triangle
	output << "{ OFF 3 1 3 ";
	for (size_t i = 0; i < 3; i++)
	{
		PointXYZ *p = _triangle.getVertex(i).first;
		output << p->x << " " << p->y << " " << p->z << " ";
	}
	output << "3 0 1 2 0 }\n";

	// Generate a line to draw the sphere
	output << "{ STESPHERE " << _radius << " " << _center.x << " " << _center.y << " " << _center.z << " }\n";

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
		PointXYZ *p = _triangle.getVertex(i).first;
		output << p->x << " " << p->y << " " << p->z << "\n";
	}
	output << 3 << " " << 0 << " " << 1 << " " << 2 << " " << 0 << "\n";

	output.close();
}
