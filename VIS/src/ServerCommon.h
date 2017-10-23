#pragma once

#include "Geometry.h"
#include "PlatformControls.h"
#include "Visualization.h"

namespace vis
{

	/// Ensure that the model directory exists, creating it if neccesary. Returns
	/// the path to this directory
	boost::optional<boost::filesystem::path> ensureModelPath();


	/// Respond to an HTTP request with a given point cloud
	/// @param res the HTTP response
	/// @param cloud the cloud to write
	void respondWithCloud(const web::http::http_request& req, const PointCloud& cloud);


	/// Respond to an HTTP request with a given point cloud
	/// @param res the HTTP response
	/// @param cloud the cloud to write
	void respondWithCloud(const web::http::http_request& req, const ErrorPointCloud& cloud);


	/// Context to app-wide resources
	class AppContext
	{
	public:

		/// Build the context
		AppContext(
			const std::shared_ptr<ProgressVisualizer>& spVisualizer,
			const std::shared_ptr<ICamera>& spCamera,
			const std::shared_ptr<IPlatformControls>& spPlatControls
		)
			: m_spVisualizer(spVisualizer)
			, m_spCamera(spCamera)
			, m_spPlatControls(spPlatControls) {}


		/// Get the app-wide visualizer
		ProgressVisualizer* const getVisualizer() const { return m_spVisualizer.get(); }


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
		using Handler = std::function<void(const web::http::http_request&, const AppContext&)>;


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
		void handle(const web::http::http_request& request, const AppContext& ctx);


	private:
		web::http::method m_method;
		std::wstring m_path;
		Handler m_handler;
	};


	/// HTTP Handler which forwards a request based on path
	class AppServer
	{
	public:
		AppServer(const AppContext* pAppContext)
			: m_pAppContext(pAppContext) {}

		void operator()(const web::http::http_request& request);

	private:
		static std::vector<HttpRoute> s_routes;
		const AppContext* m_pAppContext;
	};
}
