#include "pch.h"
#include "Statistics.h"

using namespace std;

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> detectErrors(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> idealPC, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> physicalPC) {

	// Create KD tree of actual
	pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
	kdTree.setInputCloud(idealPC);

	vector<int> distances = {};

	// Iterate over all points in the scanned and try to find a nearest point in the actual
	for (int i = 0; i < physicalPC->points.size(); i++) {

		// Find a nearest point in actual
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);

		kdTree.nearestKSearch(physicalPC->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance);

		// Record distances in an array
		distances.push_back(pointNKNSquaredDistance.at(0));
	}

	float standardDeviation = calculateSD(distances);
	float mean = calculateMean(distances);
	float outerFence = findOuterFence(distances);

	// Point Cloud representing points that exceed the standard deviation
	pcl::PointCloud<pcl::PointXYZ>::Ptr defects(new pcl::PointCloud<pcl::PointXYZ>);

	defects->width = 0;
	defects->height = 1;
	defects->is_dense = false;
	defects->points.resize(defects->width * defects->height);

	int count = 0;
	defects->width = physicalPC->width;
	defects->points.resize(defects->width * defects->height);


	// Add all points in scanned that exceed the standard deviation to a set of error points
	for (int i = 0; i < physicalPC->points.size(); i++) {
		if (distances.at(i) > outerFence) {
			defects->points[count].x = physicalPC->points[i].x;
			defects->points[count].y = physicalPC->points[i].y;
			defects->points[count].z = physicalPC->points[i].z;
			count++;
		}
	}

	defects->width = count;
	defects->points.resize(defects->width * defects->height);

	return defects;
}