/**
 * Author: rodrigo
 * 2015
 */
#include "Writer.h"
#include <fstream>
#include <map>
#include <sstream>

using namespace pcl;

#define SSTR( x ) dynamic_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

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

void Writer::writeMesh(const string &_filename, const vector<Triangle> &_meshData, const bool _addSequential)
{
	static int sequential = 0;

	ofstream output;
	string name = OUTPUT_FOLDER + _filename;
	if (_addSequential)
		name += SSTR(sequential++);
	name += POLYGON_EXTENSION;
	output.open(name.c_str(), fstream::out);

	output.precision(10);
	output << fixed;

	// Extract points and triangle data
	vector<vector<int> > sides;
	map<PointXYZ *, int> pointMap;
	int counter = 0;
	for (size_t k = 0; k < _meshData.size(); k++)
	{
		Triangle t = _meshData[k];
		sides.push_back(vector<int>());

		for (int i = 0; i < 3; i++)
		{
			PointXYZ * p = t.getVertex(i).first;
			if (pointMap.find(p) == pointMap.end())
				pointMap[p] = counter++;

			sides.back().push_back(pointMap[p]);
		}
	}

	output << "OFF\n#number of points, number of faces, number of edges\n";
	int points = pointMap.size();
	int faces = _meshData.size();
	output << points << " " << faces << " " << points << "\n";

	output << "#x, y, z\n";
	for (map<PointXYZ *, int>::iterator it = pointMap.begin(); it != pointMap.end(); it++)
		output << it->first->x << " " << it->first->y << " " << it->first->z << "\n";

	output << "#Polygons faces\n";
	for (size_t k = 0; k < sides.size(); k++)
		output << "3 " << sides[k][0] << " " << sides[k][1] << " " << sides[k][2] << "\n";

	output.close();
}

void Writer::writeTriangle(const string &_filename, const Triangle &_triangle)
{
	ofstream output;
	string name = OUTPUT_FOLDER + _filename + POLYGON_EXTENSION;
	output.open(name.c_str(), fstream::out);

	output.precision(10);
	output << fixed;

	output << "OFF\n3 1 3\n";
	for (size_t i = 0; i < 3; i++)
	{
		PointXYZ *p = _triangle.getVertex(i).first;
		output << p->x << " " << p->y << " " << p->z << "\n";
	}
	output << 3 << " " << 0 << " " << 1 << " " << 2 << " " << 0 << "\n";

	output.close();
}
