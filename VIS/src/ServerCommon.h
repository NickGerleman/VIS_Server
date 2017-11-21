#pragma once

#include "Geometry.h"
#include "PlatformControls.h"

namespace vis
{

	class ProgressVisualizer;


	/// Generate an HTTP Response with a given point cloud
	/// @param req the HTTP request
	/// @param cloud the cloud to write
	web::http::http_response createPointCloudResponse(const web::http::http_request& req, const PointCloud& cloud);


	/// Respond to an HTTP request with a given point cloud
	/// @param req the HTTP request
	/// @param cloud the cloud to write
	web::http::http_response createPointCloudResponse(const web::http::http_request& req, const ErrorPointCloud& cloud);


	/// Context to app-wide resources
	class DeviceContext
	{
	public:

		/// Build the context
		DeviceContext(
			const std::shared_ptr<ICamera>& spCamera,
			const std::shared_ptr<IPlatformControls>& spPlatControls
		)
			: m_spCamera(spCamera)
			, m_spPlatControls(spPlatControls) {}


		/// Get the app-wide camera instance
		ICamera* const getCamera() const { return m_spCamera.get(); }


		/// Get the app-wide visualizer
		IPlatformControls* const getPlatformControls() const { return m_spPlatControls.get(); }

	private:
		std::shared_ptr<ProgressVisualizer> m_spVisualizer;
		std::shared_ptr<ICamera> m_spCamera;
		std::shared_ptr<IPlatformControls> m_spPlatControls;
	};


	/// Binding of a path to action 
	class HttpRoute
	{
	public:
		using Handler = std::function<web::http::http_response(const web::http::http_request&, const DeviceContext&)>;


		/// Create a route binding the path and action
		/// @param method the request's method
		/// @param path the request path
		/// @param handler the handler for the route
		HttpRoute(const web::http::method& method, const wchar_t* path, const Handler& handler)
			: m_method(method)
			, m_path(path)
			, m_handler(handler) {}


		/// Whether this route should handle the URL
		/// @param method the http method
		/// @param url the full url
		bool canHandle(const web::http::method& method, const web::uri& url);


		/// Call the bound action
		/// @param request the associated request
		web::http::http_response handle(const web::http::http_request& request, const DeviceContext& ctx);


	private:
		web::http::method m_method;
		std::wstring m_path;
		Handler m_handler;
	};


	/// HTTP Handler which forwards a request based on path
	class AppServer
	{
	public:
		AppServer(const DeviceContext* pAppContext)
			: m_pAppContext(pAppContext) {}

		void operator()(const web::http::http_request& request);

	private:
		static std::vector<HttpRoute> s_routes;
		const DeviceContext* m_pAppContext;
	};
}
