/**
 * Author: rodrigo
 * 2015
 */
#include <stdlib.h>
#include <string>
#include <pcl/point_types.h>
#include <ctime>
#include "Helper.h"
#include "Triangle.h"
#include "Writer.h"
#include "Pivoter.h"
#include "Front.h"
#include "Config.h"
#include "CudaUtil.h"

int main(int _argn, char **_argv)
{
	if (system("rm -rf ./output/*") != 0)
		std::cout << "ERROR: bad command\n";

	if (_argn < 2)
	{
		std::cout << "Not enough arguments\n";
		return EXIT_FAILURE;
	}

	std::string inputFile = _argv[1];

	Config::load("./config/config");
	double ballRadius = Config::getBallRadius();
	DebugLevel debug = Config::getDebugLevel();
	int debugMask = ADD_SEQUENTIAL | DRAW_CLOUD | DRAW_NORMALS | (Config::drawSpheres() ? DRAW_SPHERES : 0x00);

	std::cout << "Loading file " << inputFile << "\n";
	clock_t begin = clock();

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
	if (!Helper::getCloudAndNormals(inputFile, cloud))
	{
		std::cout << "ERROR: loading failed\n";
		return EXIT_FAILURE;
	}
	std::cout << "Loaded " << cloud->size() << " points in cloud\n";

	/////////////////////////
	//CudaUtil::calculateBallCenters(cloud);
//	std::vector<int> idxs2;
//	CudaUtil::radiusSearch(cloud, 1, ballRadius, idxs2);
//
//	double accCuda = 0;
//	double accPCL = 0;
//	for (int k = 0; k < 100; k++)
//	{
//		int targetIdx = Helper::getRandomNumber(0, cloud->size() - 1);
//		pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
//		kdtree.setInputCloud(cloud);
//
//		clock_t startCUDA = clock();
//		std::vector<int> idxs;
//		CudaUtil::radiusSearch(cloud, targetIdx, ballRadius, idxs);
//		clock_t endCUDA = clock();
//
//		clock_t startPCL = clock();
//		std::vector<int> indices;
//		std::vector<float> distances;
//		kdtree.radiusSearch(cloud->at(targetIdx), ballRadius, indices, distances);
//		clock_t endPCL = clock();
//
//		double factor = 1000;
//		double timeCUDA = (double) (endCUDA - startCUDA) * factor / CLOCKS_PER_SEC;
//		double timePCL = (double) (endPCL - startPCL) * factor / CLOCKS_PER_SEC;
//
//		accCuda += timeCUDA;
//		accPCL += timePCL;
//
//		std::cout << std::setprecision(3) << "CUDA: " << timeCUDA << " [ms]\nPCL : " << timePCL << " [ms]\n----------" << std::endl;
//
//		// validate points
//		std::map<int, bool> validation;
//		for (size_t i = 0; i < idxs.size(); i++)
//			validation[idxs[i]] = true;
//
//		for (size_t i = 0; i < indices.size(); i++)
//			if (validation.find(indices[i]) == validation.end())
//				std::cout << "Can't find index " << indices[i] << " for point " << targetIdx << std::endl;
//	}
//	std::cout << std::setprecision(3) << "Acc CUDA: " << accCuda << " [ms]\nAcc PCL : " << accPCL << " [ms]" << std::endl;
//
//	return EXIT_SUCCESS;

	/////////////////////////

	Pivoter pivoter(cloud, ballRadius);
	Front front;
	std::vector<TrianglePtr> mesh;

	std::cout << "Building mesh with ball r=" << ballRadius << "\n";
	while (true)
	{
		// Pivot from the current front
		EdgePtr edge;
		while ((edge = front.getActiveEdge()) != NULL)
		{
			std::cout << "Testing edge " << *edge << "\n";

			std::pair<int, TrianglePtr> data = pivoter.pivot(edge);
			if (data.first != -1 && (!pivoter.isUsed(data.first) || front.inFront(data.first)))
			{
				if (debug >= LOW)
					std::cout << "Adding point " << data.first << " to front\n";

				mesh.push_back(data.second);
				front.joinAndFix(data, pivoter);

				if (debug >= LOW)
					Writer::writeMesh("addedPoint_" + SSTR(data.first), cloud, mesh, data.second, debugMask);
			}
			else
			{
				front.setInactive(edge);

				if (debug >= LOW)
					std::cout << "Edge marked as boundary" << *edge << "\n";
				if (debug >= MEDIUM)
					Writer::writeMesh("boundary_" + edge->toString(), cloud, mesh, edge, debugMask);
			}
		}

		// Find a new seed
		TrianglePtr seed;
		std::cout << "Searching a seed\n";
		if ((seed = pivoter.findSeed()) != NULL)
		{
			mesh.push_back(seed);
			front.addEdges(seed);

			if (debug >= LOW)
				Writer::writeMesh("seed", cloud, mesh, seed, debugMask);
		}
		else
			break;
	}

	clock_t end = clock();

	std::cout << "Writing output mesh\n";
	Writer::writeMesh("mesh", cloud, mesh);

	double elapsedTime = (double) (end - begin) / CLOCKS_PER_SEC;
	std::cout << "Finished in " << std::setprecision(5) << std::fixed << elapsedTime << " [s]\n";
	return EXIT_SUCCESS;
}
