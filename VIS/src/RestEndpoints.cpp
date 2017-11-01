#include "pch.h"
#include "RestEndpoints.h"
#include "Camera.h"
#include "ErrorDetection.h"
#include "Geometry.h"
#include "MeshIO.h"
#include "ServerCommon.h"
#include "Visualization.h"

using namespace boost;
using namespace web;
using namespace http;
namespace fs = filesystem;

namespace vis
{
	ServoPlatformControls spc(new Serial("\\\\.\\COM4"));

	std::vector<HttpRoute> AppServer::s_routes
	{
		HttpRoute(methods::GET,  L"/object-scan",    &scanObject),
		HttpRoute(methods::GET,  L"/room-scan",      &scanRoom),
		HttpRoute(methods::GET,  L"/mesh-file/all",  &listMeshFiles),
		HttpRoute(methods::GET,  L"/mesh-file",      &downloadMeshFile)
	};


	void scanObject(const web::http::http_request& request, const DeviceContext& ctx)
	{
		// We only want one client to be able to do scanning/rotation at once
		static std::mutex scanMutex;
		std::lock_guard<std::mutex> scanLock(scanMutex);

		// Rotate the platform if connected
		spc.startRotation();

		// Wait for the platform to finish rotating if it started
		while (spc.isRotating())
		{ }

		// For now, use our mock data to generate an error cloud
		auto spIdealMesh = tryLoadBinaryStlFile("models/cube.stl");
		auto spCaptureMesh = tryLoadBinaryStlFile("captures/cubert_realsense.stl");
		auto spCaptureCloud = boost::make_shared<vis::PointCloud>(spCaptureMesh->getVertexDataCloud());
		
		vis::AlignmentQuality quality;
		auto spIdealSurface = alignPointCloud(*spIdealMesh, spCaptureCloud, quality);
		auto spErrorCloud = createErrorCloud(spIdealSurface, *spCaptureCloud);
		ProgressVisualizer::get()->notifyErrorCloud(spErrorCloud);

		respondWithCloud(request, *spErrorCloud);
	}


	void scanRoom(const web::http::http_request& request, const DeviceContext& ctx)
	{
		auto spRoomCloud = ctx.getCamera()->captureFrame()->generatePointCloud();
		respondWithCloud(request, *spRoomCloud);
	}


	void listMeshFiles(const http_request& request, const DeviceContext& ctx)
	{
		auto modelPath = vis::ensureModelPath();
		if (!modelPath)
		{
			request.reply(status_codes::InternalError);
			return;
		}

		auto resObj = json::value::object();
		auto& filesArr = resObj[L"files"] = json::value::array();

		fs::recursive_directory_iterator iter(*modelPath);
		
		size_t i = 0;
		for (const auto& file : iter)
		{
			if (file.status().type() != fs::file_type::regular_file)
				continue;

			auto& fileObj = filesArr[i] = json::value::object();
			auto& relativePath = fs::relative(file.path(), *modelPath);

			fileObj[L"path"] = json::value(relativePath.generic_wstring());
			fileObj[L"filename"] = json::value(relativePath.filename().generic_wstring());
			i++;
		}

		request.reply(status_codes::OK, resObj);
	}


	void downloadMeshFile(const web::http::http_request& request, const DeviceContext& ctx)
	{
		auto queryParams = uri::split_query(request.request_uri().query());
		if (queryParams.find(L"path") == queryParams.end())
		{
			request.reply(status_codes::BadRequest);
			return;
		}

		auto decodedPath = uri::decode(queryParams[L"path"]);
		auto modelPath = ensureModelPath();
		if (!modelPath)
		{
			request.reply(status_codes::InternalError);
			return;
		}

		// Prevent FS traversal exploits
		auto absolutePath = modelPath.get().append(decodedPath);
		for (auto& path : absolutePath)
		{
			if (path.filename_is_dot_dot())
			{
				request.reply(status_codes::Forbidden);
				return;
			}
		}
		
		if (fs::exists(absolutePath) && fs::is_regular_file(absolutePath))
		{
			auto fstream = concurrency::streams::fstream::open_istream(absolutePath.generic_wstring()).get();
			request.reply(status_codes::OK, fstream);
		}
		else
		{
			request.reply(status_codes::NotFound);
		}
	}

}
