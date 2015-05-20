/**
 * Author: rodrigo
 * 2015
 */
#include "Ball.h"
#include "Writer.h"

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

void Ball::pivot(const Edge &_edge)
{
	PointXYZ middle = _edge.getMiddlePoint();
	double pivotingRadius = _edge.getPivotingRadius();

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

							Vector3f p0 = cloud->at(index0).getVector3fMap();
							Vector3f p1 = cloud->at(index1).getVector3fMap();
							Vector3f p2 = cloud->at(index2).getVector3fMap();

							// Find a normal to the plane holding the 3 points
							Vector3f d10 = p1 - p0;
							Vector3f d20 = p2 - p0;
							Vector3f n = d10.cross(d20).normalized();

							// Check the normal is pointing outwards
							int count = 0;
							count = normals->at(index0).getNormalVector3fMap().dot(n) < 0 ? count + 1 : count;
							count = normals->at(index1).getNormalVector3fMap().dot(n) < 0 ? count + 1 : count;
							count = normals->at(index2).getNormalVector3fMap().dot(n) < 0 ? count + 1 : count;
							if (count >= 2)
								n *= -1;

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

							// Check if the sphere will be valid before go farer
							double squaredDistance = ballRadius * ballRadius - circumscribedCircleRadius * circumscribedCircleRadius;
							if (squaredDistance > 0)
							{
								double distance = sqrt(squaredDistance);
								Vector3f ballCenter = circumscribedCircleCenter + distance * n;

								vector<int> neighborhood;
								vector<float> dists;
								PointXYZ center = PointXYZ(ballCenter.x(), ballCenter.y(), ballCenter.z());
								kdtree.radiusSearch(center, ballRadius, neighborhood, dists);

								if (neighborhood.empty())
								{
									cout << "Seed triangle found\n";
									_seedTriangle = Triangle(cloud->at(index0), cloud->at(index1), cloud->at(index2), index0, index1, index2);
									Writer::writeCircumscribedSphere("seedTriangle", center, ballRadius, _seedTriangle);
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
