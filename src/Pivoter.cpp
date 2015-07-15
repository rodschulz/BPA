/**
 * Author: rodrigo
 * 2015
 */
#include "Pivoter.h"
#include "Writer.h"
#include "Config.h"

#define IN_BALL_THRESHOLD	1e-7

Pivoter::Pivoter(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const double _ballRadius)
{
	cloud = _cloud;
	ballRadius = _ballRadius;
	kdtree.setInputCloud(cloud);

	for (size_t i = 0; i < _cloud->size(); i++)
		notUsed[i] = false;
}

Pivoter::~Pivoter()
{
}

std::pair<int, TrianglePtr> Pivoter::pivot(const EdgePtr &_edge)
{
	DebugLevel debug = Config::getDebugLevel();

	PointData v0 = _edge->getVertex(0);
	PointData v1 = _edge->getVertex(1);
	PointData op = _edge->getOppositeVertex();

	pcl::PointNormal edgeMiddle = _edge->getMiddlePoint();
	double pivotingRadius = _edge->getPivotingRadius();

	// Create a plane passing for the middle point and perpendicular to the edge
	Eigen::Vector3f middle = edgeMiddle.getVector3fMap();
	Eigen::Vector3f diff1 = 100 * (v0.first->getVector3fMap() - middle);
	Eigen::Vector3f diff2 = 100 * (_edge->getBallCenter().getVector3fMap() - middle);

	Eigen::Vector3f y = diff1.cross(diff2).normalized();
	Eigen::Vector3f normal = diff2.cross(y).normalized();
	Eigen::Hyperplane<float, 3> plane = Eigen::Hyperplane<float, 3>(normal, middle);

	Eigen::Vector3f zeroAngle = ((Eigen::Vector3f) (op.first->getVector3fMap() - middle)).normalized();
	zeroAngle = plane.projection(zeroAngle).normalized();

	double currentAngle = M_PI;
	std::pair<int, TrianglePtr> output = std::make_pair(-1, TrianglePtr());

	// Iterate over the neighborhood pivoting the ball
	std::vector<int> indices = getNeighbors(edgeMiddle, ballRadius * 2);
	for (size_t t = 0; t < indices.size(); t++)
	{
		int index = indices[t];
		if (v0.second == index || v1.second == index || op.second == index)
			continue;

		if (debug >= MEDIUM)
			std::cout << "Testing " << index << "\n";

		/**
		 * If the distance to the plane is less than the ball radius, then intersection between a ball
		 * centered in the point and the plane exists
		 */
		Eigen::Vector3f point = cloud->at(index).getVector3fMap();
		if (plane.absDistance(point) <= ballRadius)
		{
			Eigen::Vector3f center;
			Eigen::Vector3i sequence;
			if (getBallCenter(v0.second, v1.second, index, center, sequence))
			{
				pcl::PointNormal ballCenter = Helper::makePointNormal(center);
				std::vector<int> neighborhood = getNeighbors(ballCenter, ballRadius);
				if (!isEmpty(neighborhood, v0.second, v1.second, index, center))
				{
					if (debug >= HIGH)
					{
						std::cout << "\tDiscarded for neighbors: " << index << "\n";
						Writer::writeCircumscribedSphere("discarded_neighbors", center, ballRadius, Triangle(v0.first, v1.first, &cloud->at(index), v0.second, v1.second, index, center, ballRadius), cloud, true);
					}
					continue;
				}

				// Check the face is pointing upwards
				Eigen::Vector3f Vij = v1.first->getVector3fMap() - v0.first->getVector3fMap();
				Eigen::Vector3f Vik = point - v0.first->getVector3fMap();
				Eigen::Vector3f faceNormal = Vik.cross(Vij).normalized();
				if (!Helper::isOriented(faceNormal, (Eigen::Vector3f) v0.first->getNormalVector3fMap(), (Eigen::Vector3f) v1.first->getNormalVector3fMap(), (Eigen::Vector3f) cloud->at(index).getNormalVector3fMap()))
				{
					if (debug >= HIGH)
					{
						std::cout << "\tDiscarded for normal: " << index << "\n";
						std::vector<TrianglePtr> data;
						data.push_back(TrianglePtr(new Triangle(v0.first, v1.first, &cloud->at(index), v0.second, v1.second, index, center, ballRadius)));
						Writer::writeMesh("discarded_normal", cloud, data, DRAW_CLOUD);
					}

					continue;
				}

				Eigen::Vector3f projectedCenter = plane.projection(center);
				double cosAngle = zeroAngle.dot(projectedCenter.normalized());
				if (fabs(cosAngle) > 1)
					cosAngle = sign<double>(cosAngle);
				double angle = acos(cosAngle);

				// TODO fix point selection according to the angle
				if (output.first == -1 || currentAngle > angle)
				{
					if (debug >= LOW)
						std::cout << "\tPoint selected: " << index << "\n";

					currentAngle = angle;
					output = std::make_pair(index, TrianglePtr(new Triangle(v0.first, &cloud->points[index], v1.first, v0.second, index, v1.second, center, ballRadius)));
				}

			}
			else
			{
				if (debug >= MEDIUM)
					std::cout << "Can't find ball for " << index << "\n";
			}
		}
		else
		{
			if (debug >= MEDIUM)
				std::cout << "No intersection for " << index << "\n";
		}
	}

	return output;
}

TrianglePtr Pivoter::findSeed()
{
	DebugLevel debug = Config::getDebugLevel();
	double neighborhoodSize = 1.3;

	TrianglePtr seed;
	bool found = false;
	std::map<unsigned long, bool> tested;
	std::cout << "Remaining points " << notUsed.size() << std::endl;

	for (std::map<int, bool>::iterator it = notUsed.begin(); it != notUsed.end() && !found; it++)
	{
		int index0 = it->first;

		// Get the point's neighborhood
		std::vector<int> indices = getNeighbors(cloud->at(index0), ballRadius * neighborhoodSize);
		if (indices.size() < 3)
			continue;

		// Look for a valid seed
#pragma omp parallel for
		for (size_t j = 0; j < indices.size(); j++)
		{
			if (!found)
			{
				int index1 = indices[j];
				if (index1 == index0 || notUsed.find(index1) == notUsed.end())
					continue;

				for (size_t k = 0; k < indices.size() && !found; k++)
				{
					int index2 = indices[k];

					std::vector<int> trio;
					trio.push_back(index0);
					trio.push_back(index1);
					trio.push_back(index2);
					std::sort(trio.begin(), trio.end());
					unsigned long code = trio[0] + 1e5 * trio[1] + 1e10 * trio[2];

					if (tested.find(code) != tested.end() || index1 == index2 || index2 == index0 || notUsed.find(index2) == notUsed.end())
						continue;
#pragma omp critical
					{
						tested[code] = true;
					}

					if (debug >= MEDIUM)
						std::cout << "\tTesting (" << index0 << ", " << index1 << ", " << index2 << ")\n";

					Eigen::Vector3f center;
					Eigen::Vector3i sequence;
					if (!found && getBallCenter(index0, index1, index2, center, sequence))
					{
						pcl::PointNormal ballCenter = Helper::makePointNormal(center);
						std::vector<int> neighborhood = getNeighbors(ballCenter, ballRadius);
						if (!found && isEmpty(neighborhood, index0, index1, index2, center))
						{
#pragma omp critical
							{
								if (!found)
								{
									std::cout << "\tSeed found (" << sequence[0] << ", " << sequence[1] << ", " << sequence[2] << ")\n";

									seed = TrianglePtr(new Triangle(cloud->at((int) sequence[0]), cloud->at((int) sequence[1]), cloud->at((int) sequence[2]), sequence[0], sequence[1], sequence[2], ballCenter, ballRadius));
									notUsed.erase(index0);
									notUsed.erase(index1);
									notUsed.erase(index2);

									found = true;
								}
							}
							//return seed;
						}
					}
				}
			}
		}
	}

	return seed;
}

std::pair<Eigen::Vector3f, double> Pivoter::getCircumscribedCircle(const Eigen::Vector3f &_p0, const Eigen::Vector3f &_p1, const Eigen::Vector3f &_p2) const
{
	Eigen::Vector3f d10 = _p1 - _p0;
	Eigen::Vector3f d20 = _p2 - _p0;
	Eigen::Vector3f d01 = _p0 - _p1;
	Eigen::Vector3f d12 = _p1 - _p2;
	Eigen::Vector3f d21 = _p2 - _p1;
	Eigen::Vector3f d02 = _p0 - _p2;

	double norm01 = d01.norm();
	double norm12 = d12.norm();
	double norm02 = d02.norm();

	double norm01C12 = d01.cross(d12).norm();

	double alpha = (norm12 * norm12 * d01.dot(d02)) / (2 * norm01C12 * norm01C12);
	double beta = (norm02 * norm02 * d10.dot(d12)) / (2 * norm01C12 * norm01C12);
	double gamma = (norm01 * norm01 * d20.dot(d21)) / (2 * norm01C12 * norm01C12);

	Eigen::Vector3f circumscribedCircleCenter = alpha * _p0 + beta * _p1 + gamma * _p2;
	double circumscribedCircleRadius = (norm01 * norm12 * norm02) / (2 * norm01C12);

	return std::make_pair(circumscribedCircleCenter, circumscribedCircleRadius);
}

bool Pivoter::getBallCenter(const int _index0, const int _index1, const int _index2, Eigen::Vector3f &_center, Eigen::Vector3i &_sequence) const
{
	bool status = false;

	Eigen::Vector3f p0 = cloud->at(_index0).getVector3fMap();
	Eigen::Vector3f p1 = cloud->at(_index1).getVector3fMap();
	Eigen::Vector3f p2 = cloud->at(_index2).getVector3fMap();
	_sequence = Eigen::Vector3i(_index0, _index1, _index2);

	Eigen::Vector3f v10 = p1 - p0;
	Eigen::Vector3f v20 = p2 - p0;
	Eigen::Vector3f normal = v10.cross(v20);

// Calculate ball center only if points are not collinear
	if (normal.norm() > COMPARISON_EPSILON)
	{
// Normalize to avoid precision errors while checking the orientation
		normal.normalize();
		if (!Helper::isOriented(normal, (Eigen::Vector3f) cloud->at(_index0).getNormalVector3fMap(), (Eigen::Vector3f) cloud->at(_index1).getNormalVector3fMap(), (Eigen::Vector3f) cloud->at(_index2).getNormalVector3fMap()))
		{
			// Wrong orientation, swap vertices to get a CCW oriented triangle so face's normal pointing upwards
			p0 = cloud->at(_index1).getVector3fMap();
			p1 = cloud->at(_index0).getVector3fMap();
			_sequence = Eigen::Vector3i(_index1, _index0, _index2);

			v10 = p1 - p0;
			v20 = p2 - p0;
			normal = v10.cross(v20);
			normal.normalize();
		}

		std::pair<Eigen::Vector3f, double> circle = getCircumscribedCircle(p0, p1, p2);
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

bool Pivoter::isEmpty(const std::vector<int> &_data, const int _index0, const int _index1, const int _index2, const Eigen::Vector3f &_ballCenter) const
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

		Eigen::Vector3f dist = cloud->at(_data[i]).getVector3fMap() - _ballCenter;
		if (fabs(dist.norm() - ballRadius) < IN_BALL_THRESHOLD)
			continue;

		return false;
	}

	return true;
}

std::vector<int> Pivoter::getNeighbors(const pcl::PointNormal &_point, const double _radius) const
{
	std::vector<int> indices;
	std::vector<float> distances;
	kdtree.radiusSearch(_point, _radius, indices, distances);
	return indices;
}
