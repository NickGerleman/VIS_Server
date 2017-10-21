#pragma once

namespace vis
{

	///
	/// Ensure that the model directory exists, creating it if neccesary. Returns
	/// the path to this directory
	///
	boost::optional<boost::filesystem::path> ensureModelPath();


	///
	/// Binding of a path to action 
	///
	class HttpRoute
	{
	public:
		using Handler = std::function<void(const web::http::http_request&)>;


		///
		/// Create a route binding the path and action
		/// @param method the request's method
		/// @param path the request path
		/// @param handler the handler for the route
		///
		HttpRoute(const web::http::method& method, const wchar_t* path, const Handler& handler)
			: m_method(method)
			, m_path(path)
			, m_handler(handler) {}


		///
		/// Whether this route should handle the URL
		/// @param method the http method
		/// @param url the full url
		///
		bool canHandle(const web::http::method& method, const web::uri& url);


		///
		/// Call the bound action
		/// @param request the associated request
		///
		void handle(const web::http::http_request& request);


	private:
		web::http::method m_method;
		std::wstring m_path;
		Handler m_handler;
	};


	///
	/// HTTP Handler which forwards a request based on path
	///
	class RequestRouter
	{
	public:

		///
		/// Build the router from the given routes
		///
		RequestRouter(const std::initializer_list<HttpRoute>& routes)
			: m_routes(routes) {}


		void operator()(const web::http::http_request& request);


	private:
		std::vector<HttpRoute> m_routes;
	};
}
