/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/common/common.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <ostream>

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

	friend ostream &operator<<(ostream &_stream, const Edge &_edge);

	Edge &operator=(const Edge &_other);
	bool operator<(const Edge &_other) const;
	bool operator==(const Edge &_other) const;
	bool operator!=(const Edge &_other) const;

	inline void setActive(const bool _active)
	{
		active = _active;
	}

	inline bool isActive() const
	{
		return active;
	}

	inline PointData getVertex(const int _n) const
	{
		if (_n < 2)
			return vertices[_n];
		else
			return make_pair<PointNormal *, int>(NULL, -1);
	}

	inline PointData getOppositeVertex() const
	{
		return oppositeVertex;
	}

	inline PointNormal getMiddlePoint() const
	{
		return middlePoint;
	}

	inline PointNormal getBallCenter() const
	{
		return ballCenter;
	}

	inline double getPivotingRadius() const
	{
		return pivotingRadius;
	}

	inline string toString() const
	{
		return dynamic_cast<std::stringstream &>((std::stringstream() << std::dec << vertices[0].second << "-" << vertices[1].second)).str();
	}

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
	string str;
};
