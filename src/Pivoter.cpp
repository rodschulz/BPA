/**
 * Author: rodrigo
 * 2015
 */
#include "Pivoter.h"
#include "Writer.h"

Pivoter::Pivoter(const PointCloud<PointNormal>::Ptr &_cloud, const double _ballRadius)
{
	cloud = _cloud;
	ballRadius = _ballRadius;
	kdtree.setInputCloud(cloud);

	used.clear();
	used.resize(_cloud->size(), false);
}

Pivoter::~Pivoter()
{
}

TrianglePtr Pivoter::findSeed()
{
	// TODO this could be faster by storing only indices of points actually unused

	TrianglePtr seed;
	for (size_t index0 = 0; index0 < cloud->size(); index0++)
	{
		if (used[index0])
			continue;

		// Get the point's neighborhood
		vector<int> indices;
		vector<float> squaredDistances;
		kdtree.radiusSearch(cloud->at(index0), ballRadius * 2, indices, squaredDistances);
		if (indices.size() < 3)
			continue;

		// Look for a valid seed
		for (size_t j = 0; j < indices.size(); j++)
		{
			int index1 = indices[j];
			if (index1 == index0 || used[index1])
				continue;

			for (size_t k = 0; k < indices.size(); k++)
			{
				int index2 = indices[k];
				if (index1 == index2 || index2 == index0 || used[index2])
					continue;

				cout << "Testing (" << index0 << ", " << index1 << ", " << index2 << ")\n";

				Vector3f normal;
				pair<Vector3f, double> circle = getCircumscribedCircle(index0, index1, index2, normal);
				if (circle.second < 0)
					continue;

				Vector3f center;
				if (getBallCenter(circle, normal, center))
				{
					vector<int> neighborhood;
					vector<float> dists;
					PointNormal ballCenter = Helper::makePointNormal(center.x(), center.y(), center.z());
					kdtree.radiusSearch(ballCenter, ballRadius, neighborhood, dists);

					//Writer::writeMesh("seedTesting", cloud, vector<TrianglePtr>(), TrianglePtr(new Triangle(cloud->at(index0), cloud->at(index1), cloud->at(index2), index0, index1, index2, ballCenter, ballRadius)), true);

					if (isEmpty(neighborhood, index0, index1, index2))
					{
						cout << "Seed found (" << index0 << ", " << index1 << ", " << index2 << ")\n";

						seed = TrianglePtr(new Triangle(cloud->at(index0), cloud->at(index1), cloud->at(index2), index0, index1, index2, ballCenter, ballRadius));
						used[index0] = used[index1] = used[index2] = true;
						return seed;
					}
				}
			}
		}
	}

	return seed;
}

pair<Vector3f, double> Pivoter::getCircumscribedCircle(const int _index0, const int _index1, const int _index2, Vector3f &_normal) const
{
	Vector3f p0 = cloud->at(_index0).getVector3fMap();
	Vector3f p1 = cloud->at(_index1).getVector3fMap();
	Vector3f p2 = cloud->at(_index2).getVector3fMap();

	// Find a normal to the plane holding the 3 points
	Vector3f d10 = p1 - p0;
	Vector3f d20 = p2 - p0;
	_normal = d10.cross(d20);

	// The three points are aligned
	if (_normal.norm() < 0.001 * d10.norm() * d20.norm())
		return make_pair(Vector3f(0, 0, 0), -1);

	_normal.normalize();

	// Check the normal is pointing outwards
	int count = 0;
	count = cloud->at(_index0).getNormalVector3fMap().dot(_normal) < 0 ? count + 1 : count;
	count = cloud->at(_index1).getNormalVector3fMap().dot(_normal) < 0 ? count + 1 : count;
	count = cloud->at(_index2).getNormalVector3fMap().dot(_normal) < 0 ? count + 1 : count;
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

