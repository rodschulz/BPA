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

// Edge smart pointer
class Edge;
typedef boost::shared_ptr<Edge> EdgePtr;

typedef pair<PointXYZ *, int> PointData;

class Edge
{
public:
	Edge();
	Edge(const PointData &_v0, const PointData &_v1, const PointData &_opposite, const PointXYZ &_center);
	Edge(const Edge &_other);
	~Edge();

	Edge &operator=(const Edge &_other);
	bool operator<(const Edge &_other) const;
	bool operator==(const Edge &_other) const;
	bool operator!=(const Edge &_other) const;

	void setActive(const bool _active);
	bool isActive() const;
	PointData getVertex(const int _point) const;
	PointData getOppositeVertex() const;
	PointXYZ getMiddlePoint() const;
	PointXYZ getBallCenter() const;
	double getPivotingRadius() const;

private:
	vector<PointData> vertices;
	PointData oppositeVertex;
	PointXYZ ballCenter;
	PointXYZ middlePoint;
	double pivotingRadius;
	bool active;
	long id;

	void setId();
};
