/**
 * Author: rodrigo
 * 2015
 */
#include "Writer.h"

#define PRECISION		12

Writer::Writer()
{
}

Writer::~Writer()
{
}

void Writer::writeCircumscribedSphere(const string &_filename, const Vector3f &_center, const double _radius, const Triangle &_triangle, const PointCloud<PointNormal>::Ptr &_neighborhood, const bool _addSequential)
{
	string name = generateName(_filename, POLYGON_EXTENSION, _addSequential);

	ofstream output;
	output.open(name.c_str(), fstream::out);

	output << fixed;
	output << "appearance { -face +edge }\n{ LIST\n\n";

	output << "{ ";
	generateTriangle(_triangle, output);
	output << "}\n\n";

	output << "{ ";
	generateSphere(_center, _radius, output);
	output << "}\n\n";

	output << "{ ";
	generateCloud(_neighborhood, output);
	output << "}\n\n";

	output << "}\n";
	output.close();
}

void Writer::writeMesh(const string &_filename, const PointCloud<PointNormal>::Ptr &_cloud, const vector<TrianglePtr> &_meshData, const bool _addSequential)
{
	string name = generateName(_filename, POLYGON_EXTENSION, _addSequential);

	ofstream output;
	output.open(name.c_str(), fstream::out);

	// Write header
	output << "appearance { -face +edge }\n{ LIST\n\n";

	output << "{ ";
	generateMesh(_meshData, output);
	output << "}\n\n";

	output << "{ ";
	generateCloud(_cloud, output);
	output << "}\n\n";

	output << "{ ";
	generateNormals(_cloud, output);
	output << "}\n\n";

	output << "}\n";
	output.close();
}

void Writer::writeMesh(const string &_filename, const vector<TrianglePtr> &_meshData, const bool _addSequential)
{
	string name = generateName(_filename, POLYGON_EXTENSION, _addSequential);

	ofstream output;
	output.open(name.c_str(), fstream::out);

	output << fixed;
	generateMesh(_meshData, output);

	output.close();
}

void Writer::writeMesh(const string &_filename, const PointCloud<PointNormal>::Ptr &_cloud, const vector<TrianglePtr> &_meshData, const TrianglePtr &_seed, const bool _addSequential)
{
	string name = generateName(_filename, POLYGON_EXTENSION, _addSequential);

	ofstream output;
	output.open(name.c_str(), fstream::out);

	// Write header
	output << "appearance { -face +edge }\n{ LIST\n\n";

	output << "{ ";
	generateMesh(_meshData, output);
	output << "}\n\n";

//	output << "{ ";
//	Vector3f center = _seed->getBallCenter().getVector3fMap();
//	generateSphere(center, _seed->getBallRadius(), output);
//	output << "}\n\n";

	output << "{ ";
	generateCloud(_cloud, output);
	output << "}\n\n";

	output << "{ ";
	generateNormals(_cloud, output);
	output << "}\n\n";

	output << "{ ";
	generateTriangleFace(_seed, output);
	output << "}\n\n";

	output << "}\n";
	output.close();
}

void Writer::writeMesh(const string &_filename, const PointCloud<PointNormal>::Ptr &_cloud, const vector<TrianglePtr> &_meshData, const EdgePtr &_boundary, const bool _addSequential)
{
	string name = generateName(_filename, POLYGON_EXTENSION, _addSequential);

	ofstream output;
	output.open(name.c_str(), fstream::out);

	// Write header
	output << "appearance { -face +edge }\n{ LIST\n\n";

	output << "{ ";
	generateMesh(_meshData, output);
	output << "}\n\n";

	output << "{ ";
	generateCloud(_cloud, output);
	output << "}\n\n";

	output << "{ ";
	generateNormals(_cloud, output);
	output << "}\n\n";

	output << "{ ";
	generateEdge(_boundary, output);
	output << "}\n\n";

	output << "}\n";
	output.close();
}

void Writer::writeTriangle(const string &_filename, const Triangle &_triangle)
{
	string name = generateName(_filename, POLYGON_EXTENSION);

	ofstream output;
	output.open(name.c_str(), fstream::out);

	output << fixed;
	generateTriangle(_triangle, output);

	output.close();
}

void Writer::generateCloud(const PointCloud<PointNormal>::Ptr &_cloud, ofstream &_output)
{
	_output.precision(PRECISION);
	_output << fixed;

	// Write header
	_output << "appearance { linewidth 4 } VECT\n\n";
	_output << "# num of lines, num of vertices, num of colors\n";
	_output << _cloud->size() << " " << _cloud->size() << " 1\n\n";

	_output << "# num of vertices in each line\n";
	for (size_t i = 0; i < _cloud->size(); i++)
		_output << "1 ";
	_output << "\n\n";

	_output << "# num of colors for each line\n1 ";
	for (size_t i = 1; i < _cloud->size(); i++)
		_output << "0 ";
	_output << "\n\n";

	_output << "# points coordinates\n";
	for (size_t i = 0; i < _cloud->size(); i++)
		_output << _cloud->points[i].x << " " << _cloud->points[i].y << " " << _cloud->points[i].z << "\n";
	_output << "\n";

	_output << "# Color for vertices in RGBA\n";
	_output << "1 0 0 1\n";
}

void Writer::generateMesh(const vector<TrianglePtr> &_meshData, ofstream &_output)
{
	_output.precision(PRECISION);
	_output << fixed;

	vector<vector<int> > sides;
	vector<PointNormal *> pointArray;
	map<PointNormal *, int> pointMap;
	int counter = 0;
	for (size_t k = 0; k < _meshData.size(); k++)
	{
		TrianglePtr t = _meshData[k];
		sides.push_back(vector<int>());

		for (int i = 0; i < 3; i++)
		{
			PointNormal *p = t->getVertex(i).first;
			if (pointMap.find(p) == pointMap.end())
			{
				pointMap[p] = pointArray.size();
				pointArray.push_back(p);
			}

			sides.back().push_back(pointMap[p]);
		}
	}

	_output << "appearance { +face +edge }\n";

	_output << "OFF\n# num of points, num of faces, num of edges\n";
	int points = pointMap.size();
	int faces = _meshData.size();
	_output << points << " " << faces << " " << points << "\n";

	_output << "# x, y, z\n";
	for (size_t i = 0; i < pointArray.size(); i++)
		_output << pointArray[i]->x << " " << pointArray[i]->y << " " << pointArray[i]->z << "\n";

	_output << "# polygon faces\n";
	for (size_t k = 0; k < sides.size(); k++)
		_output << "3 " << sides[k][0] << " " << sides[k][1] << " " << sides[k][2] << "\n";
}

void Writer::generateSphere(const Vector3f &_center, const double _radius, ofstream &_output)
{
	_output.precision(PRECISION);
	_output << fixed;

	_output << "STESPHERE\n" << _radius << "\n" << _center.x() << " " << _center.y() << " " << _center.z() << "\n";
}

void Writer::generateTriangle(const Triangle &_triangle, ofstream &_output)
{
	_output.precision(PRECISION);
	_output << fixed;

	_output << "OFF\n3 1 3\n";
	for (size_t i = 0; i < 3; i++)
	{
		PointNormal *p = _triangle.getVertex(i).first;
		_output << p->x << " " << p->y << " " << p->z << "\n";
	}
	_output << "\n3 0 1 2\n";
}

void Writer::generateTriangleFace(const TrianglePtr &_triangle, ofstream &_output)
{
	_output.precision(PRECISION);
	_output << fixed;

	// Draw faces edges
	_output << "{\n";

	_output << "appearance { linewidth 2 } VECT\n\n";
	_output << "# num of lines, num of vertices, num of colors\n";
	_output << "3 6 3\n\n";

	_output << "# num of vertices in each line\n";
	_output << "2 2 2\n\n";

	_output << "# num of colors for each line\n";
	_output << "1 1 1\n\n";

	_output << "# points coordinates\n";
	for (size_t i = 0; i < 3; i++)
	{
		PointData point = _triangle->getVertex(i);
		_output << point.first->x << " " << point.first->y << " " << point.first->z << "\n";
		point = _triangle->getVertex((i + 1) % 3);
		_output << point.first->x << " " << point.first->y << " " << point.first->z << "\n";
	}
	_output << "\n";

	_output << "# Color for vertices in RGBA\n";
	_output << "1 0 1 1\n"; // Magenta
	_output << "0 1 0 1\n"; // Green
	_output << "0 1 1 1\n"; // Cyan

	_output << "}\n";

	// Draw face's vertices
//	_output << "{\n";
//
//	_output << "appearance { linewidth 4 } VECT\n\n";
//	_output << "# num of lines, num of vertices, num of colors\n";
//	_output << "3 3 3\n\n";
//
//	_output << "# num of vertices in each line\n";
//	_output << "1 1 1\n\n";
//
//	_output << "# num of colors for each line\n";
//	_output << "1 1 1\n\n";
//
//	_output << "# points coordinates\n";
//	for (size_t i = 0; i < 3; i++)
//	{
//		PointData point = _triangle->getVertex(i);
//		_output << point.first->x << " " << point.first->y << " " << point.first->z << "\n";
//	}
//	_output << "\n";
//
//	_output << "# Color for vertices in RGBA\n";
//	_output << "1 0 1 1\n"; // Magenta
//	_output << "0 1 0 1\n"; // Green
//	_output << "0 1 1 1\n"; // Cyan
//
//	_output << "}\n";
}

void Writer::generateNormals(const PointCloud<PointNormal>::Ptr &_cloud, ofstream &_output)
{
	_output.precision(PRECISION);
	_output << fixed;

	// Write header
	_output << "appearance { linewidth 1 } VECT\n\n";
	_output << "# num of lines, num of vertices, num of colors\n";
	_output << _cloud->size() << " " << _cloud->size() * 2 << " 1\n\n";

	_output << "# num of vertices in each line\n";
	for (size_t i = 0; i < _cloud->size(); i++)
		_output << "2 ";
	_output << "\n\n";

	_output << "# num of colors for each line\n1 ";
	for (size_t i = 1; i < _cloud->size(); i++)
		_output << "0 ";
	_output << "\n\n";

	float factor = 0.002;
	_output << "# points coordinates\n";
	for (size_t i = 0; i < _cloud->size(); i++)
	{
		_output << _cloud->points[i].x << " " << _cloud->points[i].y << " " << _cloud->points[i].z << "\n";
		_output << _cloud->points[i].x + _cloud->points[i].normal_x * factor << " " << _cloud->points[i].y + _cloud->points[i].normal_y * factor << " " << _cloud->points[i].z + _cloud->points[i].normal_z * factor << "\n";
	}
	_output << "\n";

	_output << "# Color for vertices in RGBA\n";
	_output << "0 0 1 1\n";
}

void Writer::generateEdge(const EdgePtr &_boundary, ofstream &_output)
{
	_output.precision(PRECISION);
	_output << fixed;

	// Write header
	_output << "appearance { linewidth 2 } VECT\n\n";
	_output << "# num of lines, num of vertices, num of colors\n";
	_output << "1 2 1\n\n";

	_output << "# num of vertices in each line\n";
	_output << "2\n\n";

	_output << "# num of colors for each line\n";
	_output << "1\n\n";

	_output << "# points coordinates\n";
	PointNormal *vertex = _boundary->getVertex(0).first;
	_output << vertex->x << " " << vertex->y << " " << vertex->z << "\n";
	vertex = _boundary->getVertex(1).first;
	_output << vertex->x << " " << vertex->y << " " << vertex->z << "\n";
	_output << "\n";

	_output << "# Color for vertices in RGBA\n";
	_output << "1 1 0 1\n";
}
