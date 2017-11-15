#include "pch.h"

#include "ServerCommon.h"
#include "MeshIo.h"
#include "RestEndpoints.h"

using namespace web;
using namespace http;
namespace fs = boost::filesystem;

namespace vis
{

	boost::optional<fs::path> ensureModelPath()
	{
		char documentPath[MAX_PATH];
		SHGetFolderPath(nullptr, CSIDL_PERSONAL, nullptr, SHGFP_TYPE_CURRENT, documentPath);
		fs::path modelPath = fs::path(documentPath).append("VIS/Models");

		if (!fs::is_directory(modelPath))
		{
			boost::system::error_code error;
			if (!fs::create_directories(modelPath, error))
			{
				std::cerr << error.message() << std::endl;
				return boost::optional<fs::path>();
			}
		}

		return modelPath;
	}


	static json::value jsonPoint(const pcl::PointXYZ& point)
	{
		auto& pointArr = json::value::array();
		pointArr[0] = point.x;
		pointArr[1] = point.y;
		pointArr[2] = point.z;

		return pointArr;
	}


	static json::value jsonPoint(const pcl::PointXYZI& point)
	{
		auto& pointArr = json::value::array();
		pointArr[0] = point.x;
		pointArr[1] = point.y;
		pointArr[2] = point.z;
		pointArr[3] = point.intensity;

		return pointArr;
	}


	template <typename TCloud>
	static web::http::http_response createPointCloudResponseT(const web::http::http_request& request, const TCloud& cloud)
	{
		http_response res;
		res.set_status_code(status_codes::OK);

		auto queryParams = uri::split_query(request.request_uri().query());
		if (queryParams.find(L"spc") != queryParams.end() && queryParams[L"spc"] == L"true")
		{
			auto spcBytes = convertCloudToSpc(cloud);
			res.set_body(Concurrency::streams::bytestream::open_istream(spcBytes));
		}
		else
		{
			auto resObj = json::value::object();
			auto& pointCloudArray = resObj[L"points"] = json::value::array();

			for (size_t i = 0; i < cloud.size(); i++)
				pointCloudArray[i] = jsonPoint(cloud[i]);

			res.set_body(resObj);
		}

		return res;
	}


	web::http::http_response createPointCloudResponse(const web::http::http_request& req, const PointCloud& cloud)
	{
		return createPointCloudResponseT(req, cloud);
	}


	web::http::http_response createPointCloudResponse(const web::http::http_request& req, const ErrorPointCloud& cloud)
	{
		return createPointCloudResponseT(req, cloud);
	}


	bool HttpRoute::canHandle(const http::method& method, const uri& url)
	{
		return m_method == method && m_path == url.path();
	}


	web::http::http_response HttpRoute::handle(const http_request& request, const DeviceContext& ctx)
	{
		return m_handler(request, ctx);
	}


	void AppServer::operator()(const http_request& request)
	{
		for (auto& route : s_routes)
		{
			if (route.canHandle(request.method(), request.request_uri()))
			{
				try
				{
					auto response = route.handle(request, *m_pAppContext);
					
					// Enable CORS in case we're running the Frontend on a seperate port
					response.headers()[L"Access-Control-Allow-Origin"] = L"*";
					request.reply(response);
				}
				catch (std::runtime_error& ex)
				{
					std::cerr << "EXCEPTION: " << ex.what() << std::endl;
					request.reply(status_codes::InternalError);
				}

				return;
			}
		}

		request.reply(status_codes::NotFound);
	}

}
