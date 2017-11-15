#include "pch.h"
#include "RestEndpoints.h"
#include "Camera.h"
#include "ErrorDetection.h"
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
		HttpRoute(methods::GET,  L"/object-scan",    &scanObject),
		HttpRoute(methods::GET,  L"/room-scan",      &scanRoom),
		HttpRoute(methods::GET,  L"/mesh-file/all",  &listMeshFiles),
		HttpRoute(methods::GET,  L"/mesh-file",      &downloadMeshFile)
	};


	http_response scanObject(const web::http::http_request& request, const DeviceContext& ctx)
	{
		http_response res;
		ProgressVisualizer::get()->notifyNewScan();

		// Platform rotation is not thread-safe
		static std::mutex scanMutex;
		std::lock_guard<std::mutex> scanLock(scanMutex);

		auto& platform = *ctx.getPlatformControls();
		auto& camera = *ctx.getCamera();

		Eigen::Matrix4f transMat;
		transMat <<
			 0.24645f,  0.01465f,  0.01943f,  0.0f,
			-0.02306f,  0.16352f,  0.16924f,  0.0f,
			-0.00203f, -0.12225f,  0.11809f,  0.0f,
			 0.01083f,  0.11231f, -0.67500f,  1.0f;

		transMat.transposeInPlace();

		auto spTestClip = std::make_shared<ClipVolume>(transMat);
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
		auto spIdealMesh = tryLoadBinaryStlFile("models/cylinder_with_low_spoke.stl");
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
		auto modelPath = vis::ensureModelPath();

		if (!modelPath)
		{
			res.set_status_code(status_codes::InternalError);
			return res;
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
		auto modelPath = ensureModelPath();
		if (!modelPath)
		{
			res.set_status_code(status_codes::InternalError);
			return res;
		}

		// Prevent FS traversal exploits
		auto absolutePath = modelPath.get().append(decodedPath);
		for (auto& path : absolutePath)
		{
			if (path.filename_is_dot_dot())
			{
				res.set_status_code(status_codes::Forbidden);
				return res;
			}
		}
		
		if (fs::exists(absolutePath) && fs::is_regular_file(absolutePath))
		{
			auto fstream = concurrency::streams::fstream::open_istream(absolutePath.generic_wstring()).get();
			res.set_status_code(status_codes::OK);
			res.set_body(fstream);
			return res;
		}
		else
		{
			res.set_status_code(status_codes::NotFound);
			return res;
		}
	}

}
