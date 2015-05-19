/**
 * Author: rodrigo
 * 2015
 */
#include <stdlib.h>
#include <string>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include "Helper.h"

using namespace std;
using namespace pcl;

int main(int _argn, char **_argv)
{
	system("rm -rf ./output/*");

	if (_argn < 2)
	{
		cout << "Not enough arguments\n";
		return EXIT_FAILURE;
	}

	string inputFile = _argv[1];
	double ballRadius = 0.01;

	// Read a PCD file from disk and calculate normals
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
	if (!Helper::getCloudAndNormals(inputFile, cloud, normals))
	{
		cout << "ERROR: loading failed\n";
		return EXIT_FAILURE;
	}
	cout << "Loaded " << cloud->size() << " points in cloud\n";

	vector<Edge> front;
	while (true)
	{
		Edge edge;
		while (getActiveEdge(edge))
		{
			PointXYZ *p = pivot(front, edge, ballRadius);
			if (!used(p) && onFront(p))
			{
				// Here triangles must be stored in an output vector

				Triangle t = Triangle(edge, p);
				join(edge, p, front);

				if (inFront(Edge(edge.getPoint(0), p), front))
					glue();
				if (inFront(Edge(edge.getPoint(1), p), front))
					glue();
			}
			else
				// Mark edge as boundary
				edge.setActive(false);
		}

		Triangle seed;
		if (findSeedTriangle(seed))
		{
			// Seed triangle must be stored in an output vector
			insertEdges(front, seed);
		}
		else
			break;
	}

//	KdTreeFLANN<PointXYZ> kdtree;
//	kdtree.setInputCloud(cloud);
//
//	vector<int> pointIndices;
//	vector<float> pointRadiusSquaredDistance;
//	kdtree.radiusSearch(_searchPoint, ballRadius, pointIndices, pointRadiusSquaredDistance);

	cout << "Finished\n";
	return EXIT_SUCCESS;
}
