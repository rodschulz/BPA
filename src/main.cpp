/**
 * Author: rodrigo
 * 2015
 */
#include <stdlib.h>
#include <string>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include "Helper.h"
#include "Ball.h"
#include "Triangle.h"
#include "Front.h"
#include "Writer.h"

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
	double ballRadius = 0.001;

	cout << "Loading file " << inputFile << "\n";

	// Read a PCD file from disk and calculate normals
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
	if (!Helper::getCloudAndNormals(inputFile, 0.005, cloud, normals))
	{
		cout << "ERROR: loading failed\n";
		return EXIT_FAILURE;
	}
	cout << "Loaded " << cloud->size() << " points in cloud\n";

	// Save normals
	PointCloud<PointNormal>::Ptr pn(new PointCloud<PointNormal>());
	concatenateFields(*cloud, *normals, *pn);
	io::savePCDFileASCII("./output/normals.pcd", *normals);
	io::savePCDFileASCII("./output/pointNormals.pcd", *pn);

	DataHolder holder(cloud, normals);
	Ball ball = Ball(holder, ballRadius);
	Front front;
	vector<Triangle> outputMesh;

	cout << "Constructing mesh using ball radius " << ballRadius << "\n";

	// Create the mesh
	while (true)
	{
		Edge *edge;
		while (front.getActiveEdge(&edge))
		{
			pair<int, Triangle> pivotData = ball.pivot(edge);
			if (pivotData.first != -1 && (!holder.used[pivotData.first] || front.inFront(pivotData.first)))
			{
				holder.used[pivotData.first] = true;
				outputMesh.push_back(pivotData.second);

				// TODO join and glue operations must set edges as active/disabled
				front.join(edge, &cloud->at(pivotData.first), pivotData);
//				join(edge, p, front);
//				if (inFront(Edge(edge.getPoint(0), p), front))
//					glue();
//				if (inFront(Edge(edge.getPoint(1), p), front))
//					glue();
			}
			else
			{
				// Mark edge as boundary
				edge->setActive(false);
			}

			Writer::writeMesh("mesh", outputMesh, true);
		}

		Triangle seed;
		if (ball.findSeedTriangle(seed))
		{
			outputMesh.push_back(seed);
			front.addEdges(seed);
		}
		else
			break;

		Writer::writeMesh("mesh", outputMesh, true);
	}

	cout << "Writing output mesh\n";
	Writer::writeMesh("mesh", outputMesh);

	cout << "Finished\n";
	return EXIT_SUCCESS;
}
