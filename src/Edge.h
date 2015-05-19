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
	Edge(const PointXYZ *_point1, const PointXYZ *_point2);
	~Edge();

	void setActive(const bool _active);
	bool isActive() const;
	PointXYZ *getPoint(const int _point);

private:
	PointXYZ *point0;
	PointXYZ *point1;
	bool active;
};
