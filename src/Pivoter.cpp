/**
 * Author: rodrigo
 * 2015
 */
#include "Pivoter.h"
#include "Writer.h"
#include "Config.h"

#define IN_BALL_THRESHOLD	1e-7

Pivoter::Pivoter(const PointCloud<PointNormal>::Ptr &_cloud, const double _ballRadius)
{
	cloud = _cloud;
	ballRadius = _ballRadius;
	kdtree.setInputCloud(cloud);

	for (size_t i = 0; i < _cloud->size(); i++)
		used[i] = false;
}

Pivoter::~Pivoter()
{
}

pair<int, TrianglePtr> Pivoter::pivot(const EdgePtr &_edge)
{
	DebugLevel debug = Config::getDebugLevel();

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

	Vector3f zeroAngle = ((Vector3f) (op.first->getVector3fMap() - middle)).normalized();
	zeroAngle = plane.projection(zeroAngle).normalized();

	double currentAngle = M_PI;
	pair<int, TrianglePtr> output = make_pair(-1, TrianglePtr());

	// Iterate over the neighborhood pivoting the ball
	vector<int> indices = getNeighbors(edgeMiddle, ballRadius * 2);
	for (size_t t = 0; t < indices.size(); t++)
	{
		int index = indices[t];
		if (v0.second == index || v1.second == index || op.second == index)
			continue;

		if (debug >= MEDIUM)
			cout << "Testing " << index << "\n";

		/**
		 * If the distance to the plane is less than the ball radius, then intersection between a ball
		 * centered in the point and the plane exists
		 */
		Vector3f point = cloud->at(index).getVector3fMap();
		if (plane.absDistance(point) <= ballRadius)
		{
			Vector3f center;
			Vector3i sequence;
			if (getBallCenter(v0.second, v1.second, index, center, sequence))
			{
				PointNormal ballCenter = Helper::makePointNormal(center);
				vector<int> neighborhood = getNeighbors(ballCenter, ballRadius);
				if (!isEmpty(neighborhood, v0.second, v1.second, index, center))
				{
					if (debug >= HIGH)
					{
						cout << "\tDiscarded for neighbors: " << index << "\n";
						Writer::writeCircumscribedSphere("discarded_neighbors", center, ballRadius, Triangle(v0.first, v1.first, &cloud->at(index), v0.second, v1.second, index, center, ballRadius), cloud, true);
					}
					continue;
				}

				// Check the face is pointing upwards
				Vector3f Vij = v1.first->getVector3fMap() - v0.first->getVector3fMap();
				Vector3f Vik = point - v0.first->getVector3fMap();
				Vector3f faceNormal = Vik.cross(Vij).normalized();
				if (!Helper::isOriented(faceNormal, (Vector3f) v0.first->getNormalVector3fMap(), (Vector3f) v1.first->getNormalVector3fMap(), (Vector3f) cloud->at(index).getNormalVector3fMap()))
				{
					if (debug >= HIGH)
					{
						cout << "\tDiscarded for normal: " << index << "\n";
						vector<TrianglePtr> data;
						data.push_back(TrianglePtr(new Triangle(v0.first, v1.first, &cloud->at(index), v0.second, v1.second, index, center, ballRadius)));
						Writer::writeMesh("discarded_normal", cloud, data, true);
					}

					continue;
				}

				Vector3f projectedCenter = plane.projection(center);
				double cosAngle = zeroAngle.dot(projectedCenter.normalized());
				if (fabs(cosAngle) > 1)
					cosAngle = sign<double>(cosAngle);
				double angle = acos(cosAngle);

				// TODO fix point selection according to the angle
				if (output.first == -1 || currentAngle > angle)
				{
					if (debug >= LOW)
						cout << "\tPoint selected: " << index << "\n";

					currentAngle = angle;
					output = make_pair(index, TrianglePtr(new Triangle(v0.first, &cloud->points[index], v1.first, v0.second, index, v1.second, center, ballRadius)));
				}

			}
			else
			{
				if (debug >= MEDIUM)
					cout << "Can't find ball for " << index << "\n";
			}
		}
		else
		{
			if (debug >= MEDIUM)
				cout << "No intersection for " << index << "\n";
		}
	}

	return output;
}

TrianglePtr Pivoter::findSeed()
{
	DebugLevel debug = Config::getDebugLevel();

	TrianglePtr seed;
	for (map<int, bool>::iterator it = used.begin(); it != used.end(); it++)
	{
		int index0 = it->first;

		// Get the point's neighborhood
		vector<int> indices = getNeighbors(cloud->at(index0), ballRadius * 2);
		if (indices.size() < 3)
			continue;

		// Look for a valid seed
		for (size_t j = 0; j < indices.size(); j++)
		{
			int index1 = indices[j];
			if (index1 == index0 || used.find(index1) == used.end())
				continue;

			for (size_t k = 0; k < indices.size(); k++)
			{
				int index2 = indices[k];
				if (index1 == index2 || index2 == index0 || used.find(index2) == used.end())
					continue;

				if (debug >= MEDIUM)
					cout << "\tTesting (" << index0 << ", " << index1 << ", " << index2 << ")\n";

				Vector3f center;
				Vector3i sequence;
				if (getBallCenter(index0, index1, index2, center, sequence))
				{
					PointNormal ballCenter = Helper::makePointNormal(center);
					vector<int> neighborhood = getNeighbors(ballCenter, ballRadius);
					if (isEmpty(neighborhood, index0, index1, index2, center))
					{
						cout << "\tSeed found (" << sequence[0] << ", " << sequence[1] << ", " << sequence[2] << ")\n";

						seed = TrianglePtr(new Triangle(cloud->at((int) sequence[0]), cloud->at((int) sequence[1]), cloud->at((int) sequence[2]), sequence[0], sequence[1], sequence[2], ballCenter, ballRadius));
						used.erase(index0);
						used.erase(index1);
						used.erase(index2);
						return seed;
					}
				}
			}
		}
	}

	return seed;
}

pair<Vector3f, double> Pivoter::getCircumscribedCircle(const Vector3f &_p0, const Vector3f &_p1, const Vector3f &_p2) const
{
	Vector3f d10 = _p1 - _p0;
	Vector3f d20 = _p2 - _p0;
	Vector3f d01 = _p0 - _p1;
	Vector3f d12 = _p1 - _p2;
	Vector3f d21 = _p2 - _p1;
	Vector3f d02 = _p0 - _p2;

	double norm01 = d01.norm();
	double norm12 = d12.norm();
	double norm02 = d02.norm();

	double norm01C12 = d01.cross(d12).norm();

	double alpha = (norm12 * norm12 * d01.dot(d02)) / (2 * norm01C12 * norm01C12);
	double beta = (norm02 * norm02 * d10.dot(d12)) / (2 * norm01C12 * norm01C12);
	double gamma = (norm01 * norm01 * d20.dot(d21)) / (2 * norm01C12 * norm01C12);

	Vector3f circumscribedCircleCenter = alpha * _p0 + beta * _p1 + gamma * _p2;
	double circumscribedCircleRadius = (norm01 * norm12 * norm02) / (2 * norm01C12);

	return make_pair(circumscribedCircleCenter, circumscribedCircleRadius);
}

bool Pivoter::getBallCenter(const int _index0, const int _index1, const int _index2, Vector3f &_center, Vector3i &_sequence) const
{
	bool status = false;

	Vector3f p0 = cloud->at(_index0).getVector3fMap();
	Vector3f p1 = cloud->at(_index1).getVector3fMap();
	Vector3f p2 = cloud->at(_index2).getVector3fMap();
	_sequence = Vector3i(_index0, _index1, _index2);

	Vector3f v10 = p1 - p0;
	Vector3f v20 = p2 - p0;
	Vector3f normal = v10.cross(v20);

	// Calculate ball center only if points are not collinear
	if (normal.norm() > COMPARISON_EPSILON)
	{
		// Normalize to avoid precision errors while checking the orientation
		normal.normalize();
		if (!Helper::isOriented(normal, (Vector3f) cloud->at(_index0).getNormalVector3fMap(), (Vector3f) cloud->at(_index1).getNormalVector3fMap(), (Vector3f) cloud->at(_index2).getNormalVector3fMap()))
		{
			// Wrong orientation, swap vertices to get a CCW oriented triangle so face's normal pointing upwards
			p0 = cloud->at(_index1).getVector3fMap();
			p1 = cloud->at(_index0).getVector3fMap();
			_sequence = Vector3i(_index1, _index0, _index2);

			v10 = p1 - p0;
			v20 = p2 - p0;
			normal = v10.cross(v20);
			normal.normalize();
		}

		pair<Vector3f, double> circle = getCircumscribedCircle(p0, p1, p2);
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

bool Pivoter::isEmpty(const vector<int> &_data, const int _index0, const int _index1, const int _index2, const Vector3f &_ballCenter) const
{
//	if (_data.size() > 3)
//		return false;
	// TODO make this a little faster by making the query to the cloud here and using the distances already given by the query
	if (_data.empty())
		return true;

	for (size_t i = 0; i < _data.size(); i++)
	{
		if (_data[i] == _index0 || _data[i] == _index1 || _data[i] == _index2)
			continue;

		Vector3f dist = cloud->at(_data[i]).getVector3fMap() - _ballCenter;
		if (fabs(dist.norm() - ballRadius) < IN_BALL_THRESHOLD)
			continue;

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
