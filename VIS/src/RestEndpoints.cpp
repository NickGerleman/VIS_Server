#include "pch.h"
#include "RestEndpoints.h"
#include "Camera.h"
#include "ErrorDetection.h"
#include "FileStorage.h"
#include "Geometry.h"
#include "MeshIO.h"
#include "Reconstruction.h"
#include "ServerCommon.h"
#include "Visualization.h"

using namespace boost;
using namespace web;
using namespace http;
namespace fs = filesystem;

namespace vis
{

	std::vector<HttpRoute> AppServer::s_routes
	{
		HttpRoute(methods::GET,   L"/object-scan",         &scanObject),
		HttpRoute(methods::GET,   L"/room-scan",           &scanRoom),
		HttpRoute(methods::GET,   L"/mesh-file/all",       &listMeshFiles),
		HttpRoute(methods::GET,   L"/mesh-file",           &downloadMeshFile),
		HttpRoute(methods::POST,  L"/calibration-volume",  &setCalibrationVolume)
	};


	http_response scanObject(const web::http::http_request& request, const DeviceContext& ctx)
	{
		http_response res;
		ProgressVisualizer::get()->notifyNewScan();

		auto queryParams = uri::split_query(request.request_uri().query());
		if (!queryParams.count(L"mesh-path"))
		{
			res.set_status_code(status_codes::BadRequest);
			return res;
		}

		auto maybeModelPath = absoluteModelPath(queryParams[L"mesh-path"]);
		if (maybeModelPath.type() == typeid(FsError))
		{
			switch (boost::get<FsError>(maybeModelPath))
			{
			case FsError::ILLEGAL_PATH:
				res.set_status_code(status_codes::Unauthorized);
				break;
			case FsError::NOT_FOUND:
				res.set_status_code(status_codes::NotFound);
				break;
			}
			return res;
		}
		auto spIdealMesh = tryLoadBinaryStlFile(boost::get<fs::path>(maybeModelPath).string());

		// Platform rotation is not thread-safe
		static std::mutex scanMutex;
		std::lock_guard<std::mutex> scanLock(scanMutex);

		auto& platform = *ctx.getPlatformControls();
		auto& camera = *ctx.getCamera();

		auto maybeCalibrationMat = readCalibrationVolume();
		if (maybeCalibrationMat.type() == typeid(FsError))
		{
			if (boost::get<FsError>(maybeCalibrationMat) == FsError::NOT_FOUND)
			{
				res.set_status_code(status_codes::BadRequest);
				return res;
			}
			
			throw std::runtime_error("Unexpected FS Error reading calibration matrix");
		}

		auto spCalibrationMat = boost::get<std::shared_ptr<Eigen::Matrix4f>>(maybeCalibrationMat);
		auto spTestClip = std::make_shared<ClipVolume>(*spCalibrationMat);
		Reconstructor recon(spTestClip);
		
		platform.startRotation();
		while (platform.isRotating())
		{
			auto spDepthFrame = camera.captureFrame();
			recon.submitFrame(spDepthFrame);
		}

		auto spCaptureMesh = recon.waitForResult();
		if (spCaptureMesh->sizeVertices() == 0)
		{
			res.set_status_code(status_codes::BadRequest);
			return res;
		}

		auto spCaptureCloud = boost::make_shared<PointCloud>(spCaptureMesh->getVertexDataCloud());
		vis::AlignmentQuality quality;
		auto spIdealSurface = alignPointCloud(*spIdealMesh, spCaptureCloud, quality);
		auto spErrorCloud = createErrorCloud(spIdealSurface, *spCaptureCloud);
		ProgressVisualizer::get()->notifyErrorCloud(spErrorCloud);
		
		return createPointCloudResponse(request, *spErrorCloud);
	}


	http_response scanRoom(const web::http::http_request& request, const DeviceContext& ctx)
	{
		ProgressVisualizer::get()->notifyNewScan();

		auto spRoomCloud = ctx.getCamera()->captureFrame()->generatePointCloud();
		return createPointCloudResponse(request, *spRoomCloud);
	}


	http_response listMeshFiles(const http_request& request, const DeviceContext& ctx)
	{
		http_response res;

		auto modelPathResult = absoluteModelPath(fs::path());
		if (modelPathResult.type() == typeid(FsError))
		{
			res.set_status_code(status_codes::InternalError);
			return res;
		}
		auto modelPath = boost::get<fs::path>(modelPathResult);

		auto resObj = json::value::object();
		auto& filesArr = resObj[L"files"] = json::value::array();

		fs::recursive_directory_iterator iter(modelPath);
		size_t i = 0;
		for (const auto& file : iter)
		{
			if (file.status().type() != fs::file_type::regular_file)
				continue;

			auto& fileObj = filesArr[i] = json::value::object();
			auto& relativePath = fs::relative(file.path(), modelPath);

			fileObj[L"path"] = json::value(relativePath.generic_wstring());
			fileObj[L"filename"] = json::value(relativePath.filename().generic_wstring());
			i++;
		}

		res.set_status_code(status_codes::OK);
		res.set_body(resObj);
		return res;
	}


	http_response downloadMeshFile(const web::http::http_request& request, const DeviceContext& ctx)
	{
		http_response res;

		auto queryParams = uri::split_query(request.request_uri().query());
		if (queryParams.find(L"path") == queryParams.end())
		{
			res.set_status_code(status_codes::BadRequest);
			return res;
		}

		auto decodedPath = uri::decode(queryParams[L"path"]);
		auto absPath = absoluteModelPath(fs::path(decodedPath));
		if (absPath.type() == typeid(FsError))
		{
			switch (boost::get<FsError>(absPath))
			{
			case FsError::ILLEGAL_PATH:
				res.set_status_code(status_codes::Forbidden);
				return res;

			case FsError::NOT_FOUND:
				res.set_status_code(status_codes::NotFound);
				return res;
			}
		}
		
		auto pathStr = boost::get<fs::path>(absPath).generic_wstring();
		auto fstream = concurrency::streams::fstream::open_istream(pathStr).get();
		res.set_body(fstream);

		res.set_status_code(status_codes::OK);
		return res;
	}


	web::http::http_response setCalibrationVolume(const web::http::http_request& request, const DeviceContext& ctx)
	{
		http_response res;

		// The real client currently doesn't set content type correctly, hack around this
		auto jsonBody = request.extract_json(true/*ignore_content_type*/).get();
		auto transformArr = jsonBody[L"transform"].as_array();
		if (transformArr.size() != 16)
		{
			res.set_status_code(status_codes::BadRequest);
			return res;
		}

		// The array is in column major format
		Eigen::Matrix4f transformMat;
		for (int i = 0; i < 16; i++)
		{
			int row = i % 4;
			int col = i / 4;
			transformMat(row, col) = static_cast<float>(transformArr[i].as_double());
		}

		auto maybeError = writeCalibrationVolume(transformMat);
		if (maybeError)
			throw std::runtime_error("Failure writing calibration volume");

		res.set_status_code(status_codes::OK);
		return res;
	}

}
