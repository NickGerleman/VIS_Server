## VIS: Visual Inspection System

Repository for the server-side of the Danfoss Visual Inspection System capstone
project. This must be compiled and run on a Windows machine.

### Requirements 

This project should be used with Visual Studio 2015 Update 3. Packages are
managed using vcpkg. The following packages are neccesary to install:

- cpprestsdk
- pcl

It is additionally required that the Kinect SDK 1.8 and Kinect Toolkit 1.8 be seperately
installed. These can be found at https://www.microsoft.com/en-us/download/details.aspx?id=40278
and https://www.microsoft.com/en-us/download/details.aspx?id=40276.

### Endpoints
**Note** The following is a work in progress. More endpoints will be added in the future
to facilitate calibration and likely mesh upload.

#### GET /object-scan
Attempt to scan a physical object and compute the square error of points in its point
cloud. A centered, aligned, four-component point cloud is returned where the fourth
component is square distance to closest physical point. The data is currently mocked.
Error handling or interruption is not currently implemented.

| Parameter | Description |
|-|-|
| mesh-path | A path pointing to a reference mesh file obtained from the /mesh-file/all endpoint |

| Status| Description |
|-|-|
| 200 | Scan was successful 
| 400 | 3D reconstruction has failed |
| 404 | The mesh was not found |

**Sample Response**
```json
{
	"points": [
		[1.0, -1.0, 1.0, 4.5],
		[-1.0, 2.0, 1.0, 8.3]
	]
}
```

#### GET /room-scan
Capture a frame from a camera and send the entire room back as a three-component point
cloud.

| Status| Description |
|-|-|
| 200 | Scan was successful 

**Sample Response**
```json
{
	"points": [
		[1.0, -1.0, 1.0],
		[-1.0, 2.0, 1.0]
	]
}
```

#### GET /mesh-file
Downloads a binary STL file corresponding to a mesh in the VIS mesh folder (in the user's
documents directory). This will currently use the raw, uncentered mesh but should
eventually move to something more controlled and uploaded.

| Parameter | Description |
|-|-|
| path | A path pointing to a reference mesh file obtained from the /mesh-file/all endpoint |

| Status| Description |
|-|-|
| 200 | All Okay
| 403 | The path contained an illegal sequence |
| 404 | The mesh was not found |

#### GET /mesh-file/all
List all possible mesh files that may be downloaded and used as a scan reference. Both of
these currently correspond directly to the files on the filesystem.

**Sample Response**
```json
{
	"files": [
		{"path": "motor/2017/top_section.stl", "filename": "top_section.stl"},
		{"path": "rubber_duck.stl", "filename": "rubber_duck.stl"}
	]
}
```

