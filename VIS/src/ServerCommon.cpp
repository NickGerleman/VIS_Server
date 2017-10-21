#include "pch.h"
#include "ServerCommon.h"

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


	bool HttpRoute::canHandle(const http::method& method, const uri& url)
	{
		return m_method == method && m_path == url.path();
	}


	void HttpRoute::handle(const http_request& request)
	{
		m_handler(request);
	}


	void RequestRouter::operator()(const http_request& request)
	{
		for (auto& route : m_routes)
		{
			if (route.canHandle(request.method(), request.request_uri()))
			{
				try
				{
					route.handle(request);
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
