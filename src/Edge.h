/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/common/common.h>

using namespace pcl;

class Edge
{
public:
	Edge();
	Edge(const PointXYZ *_point1, const PointXYZ *_point2, const PointXYZ *_oppositeVertex, const PointXYZ &_circleCenter);
	Edge(const Edge &_other);
	~Edge();

	Edge &operator=(const Edge &_other);

	void setActive(const bool _active);
	bool isActive() const;
	PointXYZ *getVertex(const int _point) const;
	PointXYZ getMiddlePoint() const;
	PointXYZ getCircleCenter() const;
	double getPivotingRadius() const;

private:
	// Triangle's vertices
	PointXYZ *vertex0;
	PointXYZ *vertex1;
	PointXYZ *oppositeVertex;

	// Circle's center and radius of the pivoting locus of the center
	PointXYZ circleCenter;
	double pivotingRadius;

	PointXYZ edgeMiddle;
	bool active;
};
