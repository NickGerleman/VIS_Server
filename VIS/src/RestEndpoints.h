#pragma once

namespace vis
{

	class AppContext;


	/// Scan an object and send it, along with errors, to the client
	/// @param request the incoming request
	/// @param ctx the application context
	void scanObject(const web::http::http_request& request, const AppContext& ctx);


	/// Capture a frame from the camera and send it as a point cloud
	/// @param request the incoming request
	/// @param ctx the application context
	void scanRoom(const web::http::http_request& request, const AppContext& ctx);


	/// List the mesh files stored on the machine
	/// @param request the incoming request
	/// @param ctx the application context
	void listMeshFiles(const web::http::http_request& request, const AppContext& ctx);


	/// Download a mesh file according to the path specified in its query paramter
	/// @param request the incoming request
	/// @param ctx the application context
	void downloadMeshFile(const web::http::http_request& request, const AppContext& ctx);
}
