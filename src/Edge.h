/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/common/common.h>
#include <boost/shared_ptr.hpp>
#include <vector>

using namespace pcl;
using namespace std;

typedef pair<PointXYZ *, int> PointDat;

class Edge
{
public:
	Edge();
	Edge(const PointDat &_vertex1, const PointDat &_vertex2, const PointDat _oppositeVertex, const PointXYZ &_ballCenter);
	Edge(const Edge &_other);
	~Edge();

	Edge &operator=(const Edge &_other);
	bool operator<(const Edge &_other) const;
	bool operator==(const Edge &_other) const;
	bool operator!=(const Edge &_other) const;

	void setActive(const bool _active);
	bool isActive() const;
	PointDat getVertex(const int _point) const;
	PointDat getOppositeVertex() const;
	PointXYZ getMiddlePoint() const;
	PointXYZ getBallCenter() const;
	double getPivotingRadius() const;

private:
	vector<PointDat> vertices;
	PointDat oppositeVertex;
	PointXYZ ballCenter;
	PointXYZ middlePoint;
	double pivotingRadius;
	bool active;
	long id;

	void setId();
};

typedef boost::shared_ptr<Edge> EdgePtr;
