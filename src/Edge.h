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

typedef pair<PointNormal *, int> PointData;

class Edge
{
public:
	Edge();
	Edge(const PointData &_v0, const PointData &_v1, const PointData &_opposite, const PointNormal &_center);
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
	PointNormal getMiddlePoint() const;
	PointNormal getBallCenter() const;
	double getPivotingRadius() const;

private:
	inline void setId()
	{
		static long currentId = 0;
		id = currentId++;
	}

	vector<PointData> vertices;
	PointData oppositeVertex;
	PointNormal ballCenter;
	PointNormal middlePoint;
	double pivotingRadius;
	bool active;
	long id;
};
