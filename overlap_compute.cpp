#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <ctime>
#include <string>
#include <map>
#include <Windows.h>

using namespace std;

#pragma region [configuration]
string directory = "C:\\Users\\km\\Desktop\\MAG\\FloatingObjectFilter\\data";
string file_name = "459_100.pcd";
#pragma endregion

#pragma region [auxiliary]

#pragma endregion

int main(int argc, char** argv) 
{
	if (argc == 3) {
		directory = argv[1];
		file_name = argv[2];
	}
	else {
		return 1;
	}


	// read the input file <point maxdim> => 1.0,2.0,3.0 5.0
	std::vector<pcl::PointXYZ> points;
	std::vector<float> radii;

	ifstream infile;
	infile.open(directory + "\\" + file_name, ios::in);
	string line;
	while (getline(infile, line)) {
		
		std::vector<std::string> results;
		boost::split(results, line, [](char c) {return c == ' '; });
		float rbnn_r = atof(results[1].c_str);

		results.clear();
		boost::split(results, line, [](char c) {return c == ','; });
		float x = atof(results[0].c_str);
		float y = atof(results[1].c_str);
		float z = atof(results[2].c_str);

		points.push_back(pcl::PointXYZ(x, y, z));
		radii.push_back(rbnn_r);
	}
	

	// create the kd-tree and commit the computations

	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < points.size(); i++) {
		cloud->push_back(points[i]);
	}

	// initialize kd tree
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	
	set<int> discardedPoints;
	for (int i = 0; i < points.size(); i++) {

		if (discardedPoints.find(i) != discardedPoints.end())
			continue; // don't investigate points that we have already discarded

		if (kdtree.radiusSearch(cloud->points[i], radii[i], pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			if (pointIdxRadiusSearch.empty()) {
				// we are safe
			}
			else {
				// add the overlapping point to the set of deleted points
				for (int j = 0; j < pointIdxRadiusSearch.size(); j++)
					discardedPoints.insert(pointIdxRadiusSearch[j]);
			}
		}
	}
	

	// write to the output file
	ofstream outfile;
	outfile.open(directory + "\\" + "result" + file_name, ios::out);
	stringstream ss;
	for (auto & i : discardedPoints) {
		ss << i << " ";
	}
	outfile << ss.str();
	outfile.close();
	cout << "results written to disc. Press any key to exit..." << endl;


	return 0;
}