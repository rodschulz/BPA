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

bool Ball::findSeedTriangle(DataHolder &_holder, Triangle &_seedTriangle)
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

							/**
							 * The idea here is generate a plane holding the 3 points,
							 * then find the circumcircle of those in the plane and using
							 * that find the center of the sphere on the outer side
							 */
							Vector3f p0 = cloud->at(index0).getVector3fMap();
							Vector3f p1 = cloud->at(index1).getVector3fMap();
							Vector3f p2 = cloud->at(index2).getVector3fMap();

							// Find a normal to the plane holding the 3 points
							Vector3f diff1 = p1 - p0;
							Vector3f diff2 = p2 - p0;
							Vector3f n = diff1.cross(diff2).normalized();

							// Check the normal is pointing outwards
							int count = 0;
							count = normals->at(index0).getNormalVector3fMap().dot(n) < 0 ? count + 1 : count;
							count = normals->at(index1).getNormalVector3fMap().dot(n) < 0 ? count + 1 : count;
							count = normals->at(index2).getNormalVector3fMap().dot(n) < 0 ? count + 1 : count;
							if (count >= 2)
								n *= -1;

							Vector3f x = diff1.normalized();
							Vector3f y = n.cross(x).normalized();

							// Solve a circumcircle of the 3 vector in the plane holding them
							Vector2f v0 = Vector2f(0, 0);
							Vector2f v1 = Vector2f(diff1.norm(), 0);
							Vector2f v2 = Vector2f(diff2.dot(x), diff2.dot(y));
							pair<pair<double, double>, double> circle = getCircumcircle2D(v0, v1, v2);
							double squaredDistance = ballRadius * ballRadius - circle.second * circle.second;

							double rad = getCircumcircleRadius(cloud->at(index0), cloud->at(index1), cloud->at(index2));

							// Check if the sphere will be valid before go farer
							if (squaredDistance > 0)
							{
								double distance = sqrt(squaredDistance);
								Vector3f circleCenter = circle.first.first * x + circle.first.second * y;
								Vector3f ballCenter = circleCenter + distance * n;

								vector<int> neighborhood;
								vector<float> dists;
								PointXYZ center = PointXYZ(ballCenter.x(), ballCenter.y(), ballCenter.z());
								kdtree.radiusSearch(center, ballRadius, neighborhood, dists);

								Writer::writeSphere("sphere", center, ballRadius);
								Writer::writeTriangle("triangle", Triangle(cloud->at(index0), cloud->at(index1), cloud->at(index2)));

								int tt = 0;
							}
						}
					}
				}
			}
		}
	}

	return status;
}

pair<pair<double, double>, double> Ball::getCircumcircle2D(const Vector2f &_v0, const Vector2f &_v1, const Vector2f &_v2) const
{
	pair<pair<double, double>, double> circle = make_pair(make_pair(0.0, 0.0), 0.0);

	double bx = _v0.x();
	double by = _v0.y();
	double cx = _v1.x();
	double cy = _v1.y();
	double dx = _v2.x();
	double dy = _v2.y();
	double temp = cx * cx + cy * cy;
	double bc = (bx * bx + by * by - temp) / 2.0;
	double cd = (temp - dx * dx - dy * dy) / 2.0;
	double det = (bx - cx) * (cy - dy) - (cx - dx) * (by - cy);

	if (fabs(det) < COMPARISON_EPSILON)
	{
		circle.first.first = 1.0;
		circle.first.second = 1.0;
		circle.second = 0.0;
	}
	else
	{
		det = 1 / det;

		circle.first.first = (bc * (cy - dy) - cd * (by - cy)) * det;
		circle.first.second = ((bx - cx) * cd - (cx - dx) * bc) * det;
		cx = circle.first.first;
		cy = circle.first.second;
		circle.second = sqrt((cx - bx) * (cx - bx) + (cy - by) * (cy - by));
	}

	return circle;
}
