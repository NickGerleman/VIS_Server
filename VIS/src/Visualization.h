#pragma once

#include "Geometry.h"

namespace vis
{

	///
	/// Create a visualizer showing the alignment between two point clouds
	/// @param spReference the reference cloud, shown in white
	/// @param spTarget the target, shown in blue
	///
	std::shared_ptr<pcl::visualization::PCLVisualizer> createAlignmentVisualizer(const boost::shared_ptr<PointCloud>& spReference, const boost::shared_ptr<PointCloud>& spTarget);


	///
	/// Create a visualizer showing the relative error in an error cloud (in red)
	/// @param cloud the error cloud
	///
	std::shared_ptr<pcl::visualization::PCLVisualizer> createErrorVisualizer(const ErrorPointCloud& cloud);

}
