#pragma once

#include "Geometry.h"

namespace vis
{

	///
	/// Calculate a point cloud containing the coordinates of a target cloud,
	/// augmented with intensity data corresponding to squared error of the point
	/// relative to the closest in a reference
	/// @param spReference the reference point cloud
	/// @param alignedCapture the physical point cloud
	///
	boost::shared_ptr<ErrorPointCloud> createErrorCloud(const boost::shared_ptr<PointCloud>& spReference, const PointCloud& alignedTarget);
}
