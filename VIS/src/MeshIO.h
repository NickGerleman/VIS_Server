#pragma once

#include "Geometry.h"

namespace vis
{

	/// Serialize a point cloud into the SPC format, then write to a stream
	/// @param cloud the point cloud
	std::vector<uint8_t> convertCloudToSpc(const PointCloud& cloud);


	/// Serialize a point cloud into the SPC format, then write to a stream
	/// @param cloud the point cloud
	std::vector<uint8_t> convertCloudToSpc(const ErrorPointCloud& cloud);


	/// Load a simple OBJ file into a mesh. Returns null if the file cannot be
	/// read. No materials and only the first group will be loaded. This method is
	/// run synchronusly and will block the currently running thread. Only meshes
	/// with 3 vertices per face are supported.
	/// @param filename the path of the OBJ file to load
	std::shared_ptr<Mesh> tryLoadObjFile(const std::string& filename);


	/// Load a binary STL file into a mesh. Returns null if the file cannot be
	/// read. This method is run synchronusly and will block the currently running
	/// thread.
	/// @param filename the path of the STL file to load
	std::shared_ptr<Mesh> tryLoadBinaryStlFile(const std::string& filename);

}
