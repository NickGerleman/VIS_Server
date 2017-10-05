#include "pch.h"
#include "Visualization.h"

namespace vis
{

	std::shared_ptr<pcl::visualization::PCLVisualizer> createAlignmentVisualizer(const boost::shared_ptr<PointCloud>& spReference, const boost::shared_ptr<PointCloud>& spTarget)
	{
		auto spVisualizer = std::make_shared<pcl::visualization::PCLVisualizer>("Alignment");
		spVisualizer->setBackgroundColor(0, 0, 0);

		spVisualizer->addPointCloud<pcl::PointXYZ>(spReference, "ref");
		spVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ref");
		spVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "ref");

		spVisualizer->addPointCloud<pcl::PointXYZ>(spTarget, "target");
		spVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target");
		spVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "target");

		spVisualizer->initCameraParameters();
		return spVisualizer;
	}


	std::shared_ptr<pcl::visualization::PCLVisualizer> createErrorVisualizer(const ErrorPointCloud& cloud)
	{
		// Use the first statistical outlier as the beginning of the gradient
		std::vector<float> orderedErrors;
		for (const auto& point : cloud)
			orderedErrors.push_back(point.intensity);
		std::sort(orderedErrors.begin(), orderedErrors.end());

		auto quartile1 = orderedErrors[orderedErrors.size() / 4];
		auto quartile3 = orderedErrors[(orderedErrors.size() / 4) * 3];
		auto iqr = quartile3 - quartile1;
		
		auto gradientMin = quartile3 + (1.5f * iqr);
		auto gradientMax = *orderedErrors.rbegin();
		auto scale = (gradientMin == 0.0) ? gradientMax : (gradientMax / gradientMin);

		// Add to colored point cloud
		auto spColorCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
		for (const auto& point : cloud)
		{
			auto normalizedError = (point.intensity - gradientMin) / scale;
			normalizedError = std::fmin(1.0f, std::fmax(0.0f, normalizedError));
			auto greenBlueComponent = static_cast<uint8_t>((1.0 - normalizedError) * 255);

			pcl::PointXYZRGB colorPoint(255, greenBlueComponent, greenBlueComponent);
			colorPoint.x = point.x;
			colorPoint.y = point.y;
			colorPoint.z = point.z;
			spColorCloud->push_back(colorPoint);
		}

		// Visualize
		auto spVisualizer = std::make_shared<pcl::visualization::PCLVisualizer>("Error");
		spVisualizer->setBackgroundColor(0, 0, 0);
		spVisualizer->addPointCloud<pcl::PointXYZRGB>(spColorCloud);
		spVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);

		return spVisualizer;
	}

}
