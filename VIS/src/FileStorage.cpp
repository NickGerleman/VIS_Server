#include "pch.h"
#include "FileStorage.h"

namespace fs = boost::filesystem;

static const std::string VIS_PATH = "VIS";
static const std::string MODEL_PATH = "Models";
static const std::string CALIBRATION_FILENAME = "calibration_volume.json";


void initFileStorage()
{
	// Do nothing if we've already created our folder structure
	auto meshPath = absoluteModelPath(fs::path());
	if (meshPath.type() != typeid(FsError))
		return;

	boost::system::error_code creationError;
	auto modelPath = boost::get<fs::path>(visRelToAbsPath(MODEL_PATH, false/*checkValid*/));
	if (!fs::create_directories(modelPath, creationError))
	{
		throw std::runtime_error(std::string("Error Creating Directory Structure:") + creationError.message());
	}
}


FsResult<boost::filesystem::path> absoluteModelPath(const fs::path& modelRelativePath)
{
	fs::path modelPath(MODEL_PATH);
	return visRelToAbsPath(modelPath.append(modelRelativePath.wstring()));
}


FsResult<std::shared_ptr<Eigen::Matrix4f>> readCalibrationVolume()
{
	auto calFilePath = visRelToAbsPath(CALIBRATION_FILENAME);
	if (calFilePath.type() == typeid(FsError))
		return boost::get<FsError>(calFilePath);

	std::ifstream jsonStream(boost::get<fs::path>(calFilePath).c_str());
	auto transformArr = web::json::value::parse(jsonStream).as_array();
	if (transformArr.size() != 16)
		throw std::runtime_error("Invalid Calibration File is Present");

	auto spMatrix = std::make_shared<Eigen::Matrix4f>();
	for (int i = 0; i < 16; i++)
	{
		int row = i / 4;
		int col = i % 4;
		(*spMatrix)(row, col) = static_cast<float>(transformArr[i].as_double());
	}

	return spMatrix;
}


boost::optional<FsError> writeCalibrationVolume(const Eigen::Matrix4f& transform)
{
	// We do this somewhat manually so that our transform is more readable as a matrix
	std::stringstream json;

	json << "[" << std::endl;
	for (int row = 0; row < 4; row++)
	{
		json << "    ";
		for (int col = 0; col < 4; col++)
		{
			std::stringstream numStream;
			numStream << transform(row, col);
			if (!(row == 3 && col == 3))
				numStream << ", ";

			// Give enough space for the first number, all digits, comma, and negative sign
			json.width(json.precision() + 5);
			json << std::left << numStream.str();
		}

		json << std::endl;
	}
	json << "]" << std::endl;

	auto outputPath = visRelToAbsPath(CALIBRATION_FILENAME);
	if (outputPath.type() == typeid(FsError))
		return boost::get<FsError>(outputPath);

	std::ofstream outStream(boost::get<fs::path>(outputPath).c_str());
	outStream << json.str();
	return boost::optional<FsError>();
}


static FsResult<fs::path> visRelToAbsPath(const fs::path& relPath, bool checkValid)
{
	char documentPath[MAX_PATH];
	SHGetFolderPath(nullptr, CSIDL_PERSONAL, nullptr, SHGFP_TYPE_CURRENT, documentPath);
	fs::path modelPath = fs::path(documentPath).append(VIS_PATH).append(relPath.wstring());

	if (checkValid)
	{
		if (!fs::exists(modelPath))
			return FsError::NOT_FOUND;

		// Prevent Traversal exploits
		for (auto& subPath : modelPath)
		{
			if (subPath.filename_is_dot_dot())
				return FsError::ILLEGAL_PATH;
		}
	}

	return modelPath;
}
