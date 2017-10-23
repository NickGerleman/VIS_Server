#pragma once

#pragma warning(push)

#pragma warning(disable: 4996) // Deprecated
#pragma warning(disable: 4005) // Macro redefinition
#pragma warning(disable: 4267) // size_t to unsigned int

#include <algorithm>
#include <array>
#include <cassert>
#include <csignal>
#include <cmath>
#include <cstdbool>
#include <exception>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

#include <cpprest/asyncrt_utils.h>
#include <cpprest/filestream.h>
#include <cpprest/http_listener.h>
#include <cpprest/json.h>
#include <cpprest/uri.h>

#include <lib/miniball/seb.h>

#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/geometry/triangle_mesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <OpenNI2/OpenNI.h>

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <Shlobj.h>
#include <NuiApi.h>
#include <NuiKinectFusionApi.h>

#pragma warning(pop)
