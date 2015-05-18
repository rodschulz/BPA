/**
 * Author: rodrigo
 * 2015
 */
#include <stdlib.h>
#include <string>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include "Helper.h"

using namespace std;
using namespace pcl;

int main(int _argn, char **_argv)
{
	system("rm -rf ./output/*");

	if (_argn < 3)
	{
		cout << "Not enough arguments\n";
		return EXIT_FAILURE;
	}

	string inputFile = _argv[1];

	// Read a PCD file from disk and calculate normals
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	if (io::loadPCDFile<PointXYZ>(inputFile, *cloud) != 0)
	{
		cout << "ERROR: Can't read file from disk (" << _argv[1] << ")\n";
		return EXIT_FAILURE;
	}
	cout << "Cloud loaded points: " << cloud->size() << "\n";

	// Remove NANs and calculate normals
	Helper::removeNANs(cloud);
	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
	Helper::getNormals(cloud, 0.01, normals);

	cout << "Finished\n";
	return EXIT_SUCCESS;
}
