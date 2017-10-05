#pragma once

namespace vis
{

	///
	/// Scan an object and send it, along with errors, to the client
	/// @param request the incoming request
	///
	void scanObject(const web::http::http_request& request);


	///
	/// List the mesh files stored on the machine
	/// @param request the incoming request
	///
	void listMeshFiles(const web::http::http_request& request);


	///
	/// Download a mesh file according to the path specified in its query paramter
	/// @param request the incoming request
	///
	void downloadMeshFile(const web::http::http_request& request);
}
