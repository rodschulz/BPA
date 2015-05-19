/**
 * Author: rodrigo
 * 2015
 */
#include "Ball.h"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Geometry/Hyperplane.h>

using namespace Eigen;

Ball::Ball(const PointCloud<PointXYZ>::Ptr &_cloud, const double &_ballRadius)
{
	kdtree.setInputCloud(_cloud);
	cloud = _cloud;
	ballRadius = _ballRadius;
}

Ball::~Ball()
{
}

void Ball::pivot(const Edge &_edge)
{
	PointXYZ middle = _edge.getMiddlePoint();

	// Create a plane passing for the middle point and perpendicular to the edge
	Vector3f m = middle.getVector3fMap();
	Vector3f edgePoint0 = _edge.getVertex(0)->getVector3fMap();
	Vector3f planeNormal = edgePoint0 - m;
	planeNormal.normalize();
	Hyperplane<float, 3> plane = Hyperplane<float, 3>(planeNormal, m);

	// Get neighborhood
	vector<int> indices;
	vector<float> squaredDistances;
	kdtree.radiusSearch(middle, ballRadius * 2, indices, squaredDistances);

	// Iterate over the neighborhood pivoting the ball
	for (size_t i = 0; i < indices.size(); i++)
	{
		Vector3f point = cloud->at(indices[i]).getVector3fMap();
		double distance = plane.absDistance(point);
		double dist2 = plane.signedDistance(point);

		// If the distance to the plane is less than the ball radius, then intersection exists
		if (distance <= ballRadius)
		{
			double intersectionCircleRadius = sqrt(ballRadius * ballRadius - distance * distance);
		}
	}
}
