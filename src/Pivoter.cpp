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

bool Pivoter::isUsed(const int _index) const
{
	return used[_index];
}

void Pivoter::setUsed(const int _index)
{
	used[_index] = true;
}

pair<int, TrianglePtr> Pivoter::pivot(const EdgePtr &_edge)
{
	PointData v0 = _edge->getVertex(0);
	PointData v1 = _edge->getVertex(1);
	PointData op = _edge->getOppositeVertex();

	PointNormal edgeMiddle = _edge->getMiddlePoint();
	double pivotingRadius = _edge->getPivotingRadius();

	// Create a plane passing for the middle point and perpendicular to the edge
	Vector3f middle = edgeMiddle.getVector3fMap();
	Vector3f diff1 = 100 * (v0.first->getVector3fMap() - middle);
	Vector3f diff2 = 100 * (_edge->getBallCenter().getVector3fMap() - middle);

	Vector3f y = diff1.cross(diff2).normalized();
	Vector3f normal = diff2.cross(y).normalized();
	Hyperplane<float, 3> plane = Hyperplane<float, 3>(normal, middle);

	// Get neighborhood
	vector<int> indices;
	vector<float> squaredDistances;
	kdtree.radiusSearch(edgeMiddle, ballRadius * 2, indices, squaredDistances);

	Vector3f zeroAngle = ((Vector3f) (op.first->getVector3fMap() - middle)).normalized();
	zeroAngle = plane.projection(zeroAngle).normalized();

	double currentAngle = M_PI;
	pair<int, TrianglePtr> output = make_pair(-1, TrianglePtr());

	// Iterate over the neighborhood pivoting the ball
	for (size_t i = 0; i < indices.size(); i++)
	{
		int index = indices[i];
		if (v0.second == index || v1.second == index || op.second == index || used[index])
			continue;

		/**
		 * If the distance to the plane is less than the ball radius, then intersection between a ball
		 * centered in the point and the plane exists
		 */
		Vector3f point = cloud->at(index).getVector3fMap();
		if (plane.absDistance(point) <= ballRadius)
		{
			Vector3f center;
			if (getBallCenter(v0.second, v1.second, index, center))
			{
				PointNormal ballCenter = Helper::makePointNormal(center);
				vector<int> neighborhood = getNeighbors(ballCenter, ballRadius);
				if (!isEmpty(neighborhood, _edge->getVertex(0).second, _edge->getVertex(1).second, index))
					continue;

				Vector3f projectedCenter = plane.projection(center);
				double cosAngle = zeroAngle.dot(projectedCenter.normalized());
				if (fabs(cosAngle) > 1)
					cosAngle = sign<double>(cosAngle);
				double angle = acos(cosAngle);

				// TODO fix point selection according to the angle
				if (output.first == -1 || currentAngle > angle)
				{
					currentAngle = angle;
					output = make_pair(index, TrianglePtr(new Triangle(v0.first, v1.first, &cloud->points[index], v0.second, v1.second, index, center, ballRadius)));
				}

			}
		}
	}

	// Extract result
	return output;
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
		vector<int> indices = getNeighbors(cloud->at(index0), ballRadius * 2);
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

				cout << "\tTesting (" << index0 << ", " << index1 << ", " << index2 << ")\n";

				Vector3f center;
				if (getBallCenter(index0, index1, index2, center))
				{
					PointNormal ballCenter = Helper::makePointNormal(center);
					vector<int> neighborhood = getNeighbors(ballCenter, ballRadius);

					//Writer::writeMesh("seedTesting", cloud, vector<TrianglePtr>(), TrianglePtr(new Triangle(cloud->at(index0), cloud->at(index1), cloud->at(index2), index0, index1, index2, ballCenter, ballRadius)), true);

					if (isEmpty(neighborhood, index0, index1, index2))
					{
						cout << "\tSeed found (" << index0 << ", " << index1 << ", " << index2 << ")\n";

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

bool Pivoter::getBallCenter(const int _index0, const int _index1, const int _index2, Vector3f &_center) const
{
	bool status = false;

	Vector3f normal;
	pair<Vector3f, double> circle = getCircumscribedCircle(_index0, _index1, _index2, normal);
	if (circle.second > 0)
	{
		double squaredDistance = ballRadius * ballRadius - circle.second * circle.second;
		if (squaredDistance > 0)
		{
			double distance = sqrt(fabs(squaredDistance));
			_center = circle.first + distance * normal;
			status = true;
		}
	}
	return status;
}

bool Pivoter::isEmpty(const vector<int> &_data, const int _index0, const int _index1, const int _index2) const
{
	if (_data.size() > 3)
		return false;
	if (_data.empty())
		return true;

	for (size_t i = 0; i < _data.size(); i++)
	{
		if (_data[i] == _index0 || _data[i] == _index1 || _data[i] == _index2)
			continue;
		else
			return false;
	}

	return true;
}

vector<int> Pivoter::getNeighbors(const PointNormal &_point, const double _radius) const
{
	vector<int> indices;
	vector<float> distances;
	kdtree.radiusSearch(_point, _radius, indices, distances);
	return indices;
}
