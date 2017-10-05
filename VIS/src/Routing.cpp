#include "pch.h"
#include "Routing.h"

using namespace web;
using namespace http;

namespace vis
{

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
				route.handle(request);
				return;
			}
		}

		request.reply(status_codes::NotFound);
	}

}
