/**
 * Author: rodrigo
 * 2015
 */
#include "Ball.h"
#include "Writer.h"
#include <queue>

Ball::Ball(const DataHolder &_holder, const double &_ballRadius)
{
	cloud = _holder.cloud;
	normals = _holder.normals;
	used = (vector<bool> *) &_holder.used;

	kdtree.setInputCloud(cloud);
	ballRadius = _ballRadius;
}

Ball::~Ball()
{
}

pair<PointXYZ *, int> Ball::pivot(const Edge &_edge)
{
	PointXYZ edgeMiddle = _edge.getMiddlePoint();
	double pivotingRadius = _edge.getPivotingRadius();

	// Create a plane passing for the middle point and perpendicular to the edge
	Vector3f middle = edgeMiddle.getVector3fMap();
	Vector3f ballCenter = _edge.getBallCenter().getVector3fMap();
	PointDat v0 = _edge.getVertex(0);
	Vector3f vertex0 = v0.first->getVector3fMap();

	Vector3f diff1 = 100 * (vertex0 - middle);
	Vector3f diff2 = 100 * (ballCenter - middle);

	Vector3f y = diff1.cross(diff2).normalized();
	Vector3f normal = diff2.cross(y).normalized();
	Hyperplane<float, 3> plane = Hyperplane<float, 3>(normal, middle);

	// Get neighborhood
	vector<int> indices;
	vector<float> squaredDistances;
	kdtree.radiusSearch(edgeMiddle, ballRadius * 2, indices, squaredDistances);

	PointDat opposite = _edge.getOppositeVertex();
	Vector3f zeroAngle = opposite.first->getVector3fMap() - middle;
	zeroAngle.normalize();
	zeroAngle = plane.projection(zeroAngle).normalized();

	// Iterate over the neighborhood pivoting the ball
	priority_queue<pair<double, int>, vector<pair<double, int> >, compareMethod> pointQueue(compareAngles);
	for (size_t i = 0; i < indices.size(); i++)
	{
		Vector3f point = cloud->at(indices[i]).getVector3fMap();
		double distanceToPlane = plane.absDistance(point);

		/**
		 * If the distance to the plane is less than the ball radius, then intersection between a ball
		 * centered in the point and the plane exists
		 */
		if (distanceToPlane <= ballRadius)
		{
			Vector3f projectedPoint = plane.projection(point);
			double inPlaneDistance = (projectedPoint - middle).norm();
			double intersectionRadius = sqrt(ballRadius * ballRadius - distanceToPlane * distanceToPlane);

			double a = (intersectionRadius * intersectionRadius - pivotingRadius * pivotingRadius + inPlaneDistance * inPlaneDistance) / (2 * inPlaneDistance);
			double h = sqrt(intersectionRadius * intersectionRadius - a * a);

			// Find the point in the middle between the intersection points
			Vector3f p = projectedPoint + (a / inPlaneDistance) * (middle - projectedPoint);

			// Find a vector pointing to the intersection point
			Vector3f v1 = (p - middle).normalized();
			Vector3f v2 = v1.cross(normal).normalized();

			// TODO check if the side is right as is or if it must be selected previously using normals
			Vector3f intersection1 = p + h * v2;
			Vector3f intersection2 = p - h * v2;

			Vector3f xx = plane.projection(intersection1);
			bool ss = intersection1 == xx;
		}
	}

	return make_pair<PointXYZ *, int>(NULL, -1);
}

bool Ball::findSeedTriangle(Triangle &_seedTriangle)
{
	bool status = false;

	for (size_t index0 = 0; index0 < cloud->size(); index0++)
	{
		// Find an unused point
		if (!used->at(index0))
		{
			// Get the point's neighborhood
			vector<int> indices;
			vector<float> squaredDistances;
			kdtree.radiusSearch(cloud->at(index0), ballRadius * 2, indices, squaredDistances);

			// Look for a valid seed
			for (size_t j = 0; j < indices.size(); j++)
			{
				int index1 = indices[j];
				if (index1 != index0 && !used->at(index1))
				{
					for (size_t k = 0; k < indices.size(); k++)
					{
						int index2 = indices[k];
						if (j != k && index2 != index0 && !used->at(index2))
						{
							cout << "Testing seed triangle in vertices " << index0 << " " << index1 << " " << index2 << "\n";

//							Vector3f p0 = cloud->at(index0).getVector3fMap();
//							Vector3f p1 = cloud->at(index1).getVector3fMap();
//							Vector3f p2 = cloud->at(index2).getVector3fMap();
//
//							// Find a normal to the plane holding the 3 points
//							Vector3f d10 = p1 - p0;
//							Vector3f d20 = p2 - p0;
//							Vector3f n = d10.cross(d20).normalized();
//
//							// Check the normal is pointing outwards
//							int count = 0;
//							count = normals->at(index0).getNormalVector3fMap().dot(n) < 0 ? count + 1 : count;
//							count = normals->at(index1).getNormalVector3fMap().dot(n) < 0 ? count + 1 : count;
//							count = normals->at(index2).getNormalVector3fMap().dot(n) < 0 ? count + 1 : count;
//							if (count >= 2)
//								n *= -1;
//
//							Vector3f d01 = p0 - p1;
//							Vector3f d12 = p1 - p2;
//							Vector3f d21 = p2 - p1;
//							Vector3f d02 = p0 - p2;
//
//							double norm01 = d01.norm();
//							double norm12 = d12.norm();
//							double norm02 = d02.norm();
//
//							double norm01C12 = d01.cross(d12).norm();
//
//							double alpha = (norm12 * norm12 * d01.dot(d02)) / (2 * norm01C12 * norm01C12);
//							double beta = (norm02 * norm02 * d10.dot(d12)) / (2 * norm01C12 * norm01C12);
//							double gamma = (norm01 * norm01 * d20.dot(d21)) / (2 * norm01C12 * norm01C12);
//
//							Vector3f circumscribedCircleCenter = alpha * p0 + beta * p1 + gamma * p2;
//							double circumscribedCircleRadius = (norm01 * norm12 * norm02) / (2 * norm01C12);

							Vector3f normal;
							pair<Vector3f, double> circle = getCircumscribedCircle(index0, index1, index2, normal);
							Vector3f circumscribedCircleCenter = circle.first;
							double circumscribedCircleRadius = circle.second;

							// Check if the sphere will be valid before go farer
							double squaredDistance = ballRadius * ballRadius - circumscribedCircleRadius * circumscribedCircleRadius;
							if (squaredDistance > 0)
							{
								double distance = sqrt(squaredDistance);
								Vector3f center = circumscribedCircleCenter + distance * normal;

								vector<int> neighborhood;
								vector<float> dists;
								PointXYZ ballCenter = PointXYZ(center.x(), center.y(), center.z());
								kdtree.radiusSearch(ballCenter, ballRadius, neighborhood, dists);

								if (neighborhood.empty())
								{
									cout << "Seed triangle found\n";

									_seedTriangle = Triangle(cloud->at(index0), cloud->at(index1), cloud->at(index2), index0, index1, index2, ballCenter, ballRadius);
									used->at(index0) = true;
									used->at(index1) = true;
									used->at(index2) = true;

									Writer::writeCircumscribedSphere("seedTriangle", ballCenter, ballRadius, _seedTriangle);
									return true;
								}
							}
						}
					}
				}
			}
		}
	}

	return status;
}

pair<Vector3f, double> Ball::getCircumscribedCircle(const int _index0, const int _index1, const int _index2, Vector3f &_normal)
{
	Vector3f p0 = cloud->at(_index0).getVector3fMap();
	Vector3f p1 = cloud->at(_index1).getVector3fMap();
	Vector3f p2 = cloud->at(_index2).getVector3fMap();

	// Find a normal to the plane holding the 3 points
	Vector3f d10 = p1 - p0;
	Vector3f d20 = p2 - p0;
	_normal = d10.cross(d20).normalized();

	// Check the normal is pointing outwards
	int count = 0;
	count = normals->at(_index0).getNormalVector3fMap().dot(_normal) < 0 ? count + 1 : count;
	count = normals->at(_index1).getNormalVector3fMap().dot(_normal) < 0 ? count + 1 : count;
	count = normals->at(_index2).getNormalVector3fMap().dot(_normal) < 0 ? count + 1 : count;
	if (count >= 2)
		_normal *= -1;

	Vector3f d01 = p0 - p1;
	Vector3f d12 = p1 - p2;
	Vector3f d21 = p2 - p1;
	Vector3f d02 = p0 - p2;

	double norm01 = d01.norm();
	double norm12 = d12.norm();
	double norm02 = d02.norm();

	double norm01C12 = d01.cross(d12).norm();

	double alpha = (norm12 * norm12 * d01.dot(d02)) / (2 * norm01C12 * norm01C12);
	double beta = (norm02 * norm02 * d10.dot(d12)) / (2 * norm01C12 * norm01C12);
	double gamma = (norm01 * norm01 * d20.dot(d21)) / (2 * norm01C12 * norm01C12);

	Vector3f circumscribedCircleCenter = alpha * p0 + beta * p1 + gamma * p2;
	double circumscribedCircleRadius = (norm01 * norm12 * norm02) / (2 * norm01C12);

	return make_pair(circumscribedCircleCenter, circumscribedCircleRadius);
}

bool Ball::compareAngles(const pair<double, int> &_p1, const pair<double, int> &_p2)
{
	return _p1.first < _p2.first;
}
