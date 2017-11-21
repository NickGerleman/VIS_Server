#pragma once

namespace vis
{

	class DeviceContext;


	/// Scan an object and send it, along with errors, to the client
	/// @param request the incoming request
	/// @param ctx the application context
	web::http::http_response scanObject(const web::http::http_request& request, const DeviceContext& ctx);


	/// Capture a frame from the camera and send it as a point cloud
	/// @param request the incoming request
	/// @param ctx the application context
	web::http::http_response scanRoom(const web::http::http_request& request, const DeviceContext& ctx);


	/// List the mesh files stored on the machine
	/// @param request the incoming request
	/// @param ctx the application context
	web::http::http_response listMeshFiles(const web::http::http_request& request, const DeviceContext& ctx);


	/// Download a mesh file according to the path specified in its query paramter
	/// @param request the incoming request
	/// @param ctx the application context
	web::http::http_response downloadMeshFile(const web::http::http_request& request, const DeviceContext& ctx);


	/// Persist the clip volume to use during object scans
	/// @param request the incoming request
	/// @param ctx the application context
	web::http::http_response setCalibrationVolume(const web::http::http_request& request, const DeviceContext& ctx);
}
