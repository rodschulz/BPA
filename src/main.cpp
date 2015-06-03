/**
 * Author: rodrigo
 * 2015
 */
#include <stdlib.h>
#include <string>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include "Helper.h"
#include "Triangle.h"
#include "Writer.h"
#include "Pivoter.h"

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
	double ballRadius = 0.002;

	cout << "Loading file " << inputFile << "\n";

	PointCloud<PointNormal>::Ptr cloud(new PointCloud<PointNormal>());
	if (!Helper::getCloudAndNormals(inputFile, cloud))
	{
		cout << "ERROR: loading failed\n";
		return EXIT_FAILURE;
	}
	cout << "Loaded " << cloud->size() << " points in cloud\n";

	Pivoter pivoter(cloud, ballRadius);
	vector<Triangle> outputMesh;
	cout << "Beginning mesh construction using ball r=" << ballRadius << "\n";
	while (true)
	{
		// Pivot from the current front
		//EdgePtr edge;

		// Find a new seed
		Triangle seed;
//		if (ball.findSeedTriangle(seed))
//		{
//			outputMesh.push_back(seed);
//			front.addEdges(seed);
//		}
//		else
//			break;

		Writer::writeMesh("mesh", cloud, outputMesh, true);
	}

	////////////////////////////////

//	DataHolder holder(cloud);
//	Ball ball = Ball(holder, ballRadius);
//	Front front;
//
//	// Create the mesh
//	while (true)
//	{
//		Edge *edge;
//		while (front.getActiveEdge(&edge))
//		{
//			pair<int, Triangle> pivotData = ball.pivot(edge);
//			if (pivotData.first != -1 && (!holder.used[pivotData.first] || front.inFront(pivotData.first)))
//			{
//				holder.used[pivotData.first] = true;
//				outputMesh.push_back(pivotData.second);
//
//				// TODO join and glue operations must set edges as active/disabled
//				//front.join(edge, &cloud->at(pivotData.first), pivotData);
////				join(edge, p, front);
////				if (inFront(Edge(edge.getPoint(0), p), front))
////					glue();
////				if (inFront(Edge(edge.getPoint(1), p), front))
////					glue();
//
//				Writer::writeMesh("mesh", cloud, outputMesh, true);
//			}
//			else
//			{
//				// Mark edge as boundary
//				edge->setActive(false);
//			}
//		}
//
//		Triangle seed;
//		if (ball.findSeedTriangle(seed))
//		{
//			outputMesh.push_back(seed);
//			front.addEdges(seed);
//		}
//		else
//			break;
//
//		Writer::writeMesh("mesh", cloud, outputMesh, true);
//	}

	cout << "Writing output mesh\n";
	Writer::writeMesh("mesh", cloud, outputMesh);

	cout << "Finished\n";
	return EXIT_SUCCESS;
}
