#pragma once

/// Expected Filesystem errors that should be handled
enum class FsError
{
	NOT_FOUND,
	ILLEGAL_PATH,
};


/// Variant that may be either an expected result from a filesystem operation, or an
/// error
template <typename TSuccess>
using FsResult = boost::variant<FsError, TSuccess>;


/// Initialize the directory structure needed for VIS. Should be called before any other
/// FS related methods
void initFileStorage();


/// Get the absolute filesystem path from a model path exposed to the client
/// @param modelRelativePath the model path exposed to the client
FsResult<boost::filesystem::path> absoluteModelPath(const boost::filesystem::path& modelRelativePath);


/// Reads the uploaded calibration volume if it exists
FsResult<std::shared_ptr<Eigen::Matrix4f>> readCalibrationVolume();


/// Writes the calibration volume to disk
/// @param trasnform the transform representing the volume
boost::optional<FsError> writeCalibrationVolume(const Eigen::Matrix4f& transform);

/// Find the absolute file path from a path relative to the VIS directory, performing
/// validation on the path
/// @param relPath the relative path
/// @param checkValid whether to verify the path is valid and exists
static FsResult<boost::filesystem::path> visRelToAbsPath(const boost::filesystem::path& relPath, bool checkValid = true);
