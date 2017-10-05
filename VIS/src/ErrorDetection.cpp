#include "pch.h"
#include "ErrorDetection.h"

namespace vis
{

	boost::shared_ptr<ErrorPointCloud> createErrorCloud(const boost::shared_ptr<PointCloud>& spReference, const PointCloud& alignedTarget)
	{
		// Create KD tree of target
		pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
		kdTree.setInputCloud(spReference);

		auto spErrorCloud = boost::make_shared<ErrorPointCloud>();

		// Iterate over all points in the scanned and try to find a nearest point in the reference
		for (int i = 0; i < alignedTarget.points.size(); i++) {

			// Find a nearest point in actual
			std::vector<int> pointIdxNKNSearch(1);
			std::vector<float> pointNKNSquaredDistance(1);

			const auto& targetPoint = alignedTarget.points[i];
			kdTree.nearestKSearch(targetPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
			
			pcl::PointXYZI errorPoint;
			errorPoint.x = targetPoint.x;
			errorPoint.y = targetPoint.y;
			errorPoint.z = targetPoint.z;
			errorPoint.intensity = pointNKNSquaredDistance[0];
			spErrorCloud->push_back(errorPoint);
		}

		return spErrorCloud;
	}

}
