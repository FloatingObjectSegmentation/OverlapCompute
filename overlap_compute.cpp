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
string directory = "E:\\workspaces\\LIDAR_WORKSPACE\\test\\";
string file_name = "augs";
#pragma endregion


/*

problem specification: A program that receives an array of objects A defined in 3D space that may be overlapping eachother.
					   The program picks a subset of objects in A, for which it is guaranteed that the objects within it do
					   not overlap eachother.

input: argv[1] = directory, argv[2] = filename. 
	   A file containing A list of records where each record is described as:
		- central pivot point C [x,y,z] - represents the center or the origin of the object's coordinate frame
		- list of boundary points B_i [x,y,z] - represents points on the boundary of the convex hull or boundary box of the object
		- maximalDimension - the value of the longest dimension of the object
output: Write file to {argv[1]}/result{argv[2]} 
	A list of indices into the input list, indicating which objects should be discarded to remove overlapped objects. (Each in new line)


*/
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
	std::vector<int> centerPointIndices;
	std::map<int, int> CentralPointMap; // accepts index of any point and returns the central point index
	std::vector<float> radii;

	ifstream infile;
	infile.open(directory + "\\" + file_name, ios::in);
	string line;
	int currIdx = 0;
	while (getline(infile, line)) {
		
		// get the max dimension
		std::vector<std::string> results;
		boost::split(results, line, [](char c) {return c == ' '; });
		float rbnn_r = atof(results[9].c_str());
		radii.push_back(rbnn_r);

		// get the central point
		std::vector<std::string> centralPointStrings;
		boost::split(centralPointStrings, results[0], [](char c) {return c == ','; });
		float x = atof(centralPointStrings[0].c_str());
		float y = atof(centralPointStrings[1].c_str());
		float z = atof(centralPointStrings[2].c_str());
		points.push_back(pcl::PointXYZ(x, y, z));
		int currentCentralPointIndex = currIdx++;
		CentralPointMap[currentCentralPointIndex] = currentCentralPointIndex;
		centerPointIndices.push_back(currentCentralPointIndex);

		// get the boundary points
		for (int i = 1; i < 9; i++) {
			std::vector<std::string> boundaryPointStrings;
			boost::split(boundaryPointStrings, results[i], [] (char c) {return c == ','; });
			float x = atof(boundaryPointStrings[0].c_str());
			float y = atof(boundaryPointStrings[1].c_str());
			float z = atof(boundaryPointStrings[2].c_str());
			points.push_back(pcl::PointXYZ(x, y, z));
			CentralPointMap[currIdx++] = currentCentralPointIndex;
		}
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
	
	///// HEART OF THE ALGORITHM

	set<int> discardedPoints;
	set<int> acceptedPoints;
	for (int i = 0; i < centerPointIndices.size(); i++) {

		int idx = centerPointIndices[i];

		if (discardedPoints.find(idx) != discardedPoints.end())
			continue; // don't investigate points that we have already discarded

		if (kdtree.radiusSearch(cloud->points[idx], radii[i], pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			if (pointIdxRadiusSearch.empty()) {
				// we are safe
				acceptedPoints.insert(idx);
			}
			else {
				// verify that it does not overlap with a point that we have already chosen
				bool overlaps_with_already_accepted = false;
				for (int j = 0; j < pointIdxRadiusSearch.size(); j++) {

					int nbridx = CentralPointMap[pointIdxRadiusSearch[j]]; // index of the central point of the current point

					if (acceptedPoints.find(nbridx) != acceptedPoints.end()) 
					{
						discardedPoints.insert(i);
						overlaps_with_already_accepted = true;
						break;
					}
				}

				// if it overlaps with one we already accepted, then we have to discard it and move on
				if (overlaps_with_already_accepted) continue;

				// if it has no accepted neighbors, then we discard all of them
				for (int j = 0; j < pointIdxRadiusSearch.size(); j++) {
					int nbridx = CentralPointMap[pointIdxRadiusSearch[j]]; // index of the central point of the current point
					if (nbridx == idx) continue;
					discardedPoints.insert(nbridx / 9); // indices are actually multiples of 9, as evidenced by the way we have inserted them into the points array
				}

				// finally accept the point
				acceptedPoints.insert(idx);
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
	outfile << ss.str().substr(0, ss.str().length() - 1);
	outfile.close();
	cout << "results written to disc. Press any key to exit..." << endl;


	return 0;
}