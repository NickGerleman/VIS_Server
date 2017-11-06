## VIS: Visual Inspection System

Repository for the server-side of the Danfoss Visual Inspection System capstone
project. This must be compiled and run on a Windows machine. The software is compatibe
with the Occipital structure sensor, or a mock recording.

### Requirements 

This project should be used with Visual Studio 2015 Update 3. Packages are managed using
vcpkg. The following packages are neccesary to install:

- cpprestsdk
- pcl

It is additionally required that the Kinect SDK V2 be seperately installed. This may be
found at https://www.microsoft.com/en-us/download/details.aspx?id=44561.

### Command Line Arguments

| Switch       | Description                                             |
|--------------|---------------------------------------------------------|
| --oni        | Path to a .ONI file to use instead of a physical camera |
| --port       | The port to listen on                                   |
| --visualizer | Display a visualizer while doing scans                  |

### SPC Binary Point Clouds
It is possible to receive any point cloud as a custom binary format called SPC. This is a
simple format consisting of a header and collection of points. The file header has two
unsigned 32-bit fields. The first is the number of components per point, the second is
the number of points. Each point is then stored sequentially where each component is
represented as a 32 bit float.

### Endpoints
**Note** The following is a work in progress. More endpoints will be added in the future
to facilitate calibration and likely mesh upload.

#### GET /object-scan
Attempt to scan a physical object and compute the square error of points in its point
cloud. A centered, aligned, four-component point cloud is returned where the fourth
component is square distance to closest physical point. The data is currently mocked.
Error handling or interruption is not currently implemented.

**Paramters**

| Name      | Type    | Description                                                                        |
|-----------|---------|------------------------------------------------------------------------------------|
| mesh-path | string  | A path pointing to a reference mesh file obtained from the /mesh-file/all endpoint |
| spc       | boolean | (Optional) Whether to use the spc format                                           |

**Return Codes**

| Status | Description                  |
|--------|------------------------------|
| 200    | Scan was successful          |
| 400    | 3D reconstruction has failed |
| 404    | The mesh was not found       |

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

**Paramters**

| Name | Type    | Description                              |
|------|---------|------------------------------------------|
| spc  | boolean | (Optional) Whether to use the spc format |

**Return Codes**

| Status| Description         |
|-------|---------------------|
| 200   | Scan was successful |

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

**Paramters**

| Name | Type   | Description                                                                        |
|------|--------|------------------------------------------------------------------------------------|
| path | string | A path pointing to a reference mesh file obtained from the /mesh-file/all endpoint |

**Return Codes**

| Status | Description                            |
|--------|----------------------------------------|
| 200    | All Okay                               |
| 403    | The path contained an illegal sequence |
| 404    | The mesh was not found                 |

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
