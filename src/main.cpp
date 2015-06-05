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
#include "Front.h"

using namespace std;
using namespace pcl;

int main(int _argn, char **_argv)
{
	// TODO check if the pivoting method is selecting correctly the next point according to the angle

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
	Front front;
	vector<TrianglePtr> mesh;

	cout << "Beginning mesh construction using ball r=" << ballRadius << "\n";
	while (true)
	{
		// Pivot from the current front
		EdgePtr edge;
		while ((edge = front.getActiveEdge()) != NULL)
		{
			cout << "Testing edge " << *edge << "\n";

			pair<int, TrianglePtr> data = pivoter.pivot(edge);
			if (data.first != -1 && (!pivoter.isUsed(data.first) || front.inFront(&cloud->points[data.first])))
			{
				cout << "Adding point " << data.first << " to front\n";
				mesh.push_back(data.second);
				front.joinAndFix(data, pivoter);
				Writer::writeMesh("addedPoint_" + SSTR(data.first), cloud, mesh, data.second);
			}
			else
			{
				cout << "Edge marked as boundary" << *edge << "\n";
				edge->setActive(false);
			}
		}

		// Find a new seed
		TrianglePtr seed;
		if ((seed = pivoter.findSeed()) != NULL)
		{
			mesh.push_back(seed);
			front.addEdges(seed);
			Writer::writeMesh("seed", cloud, mesh, seed, true);
		}
		else
			break;

		Writer::writeMesh("mesh", cloud, mesh, true);
	}

	cout << "Writing output mesh\n";
	Writer::writeMesh("mesh", cloud, mesh);

	cout << "Finished\n";
	return EXIT_SUCCESS;
}
