#include "pch.h"
//#include "Camera.h"
//#include "ErrorDetection.h"
//#include "Geometry.h"
//#include "MeshIO.h"
//#include "Visualization.h"
//
//using namespace std::chrono_literals;
//
//int main(int argc, char** argv)
//{
//	auto spIdealMesh = vis::tryLoadBinaryStlFile("models/cube.stl");
//	auto spCaptureMesh = vis::tryLoadBinaryStlFile("captures/cubert_realsense.stl");
//	auto spCaptureCloud = boost::make_shared<vis::PointCloud>(spCaptureMesh->getVertexDataCloud());
//
//	vis::AlignmentQuality quality;
//	auto spIdealSurface = vis::alignPointCloud(*spIdealMesh, spCaptureCloud, quality);
//	auto spErrorCloud = vis::createErrorCloud(spIdealSurface, *spCaptureCloud);
//
//	auto spCamera = vis::StructureSensorAccess::acquireCamera();
//	auto spCameraViewer = std::make_shared<pcl::visualization::PCLVisualizer>("Camera");
//	
//	spCameraViewer->addPointCloud(spCamera->captureFrame()->generatePointCloud());
//	spCameraViewer->setCameraPosition(0.0, 0.0, 1.0, 0.0, 1.0, 0.0);
//
//	auto spAlignmentViewer = vis::createAlignmentVisualizer(spIdealSurface, spCaptureCloud);
//	auto spErrorViewer = vis::createErrorVisualizer(*spErrorCloud);
//	
//	while (!(spCameraViewer->wasStopped()
//			&& spAlignmentViewer->wasStopped()
//			&& spErrorViewer->wasStopped()))
//	{
//		spCameraViewer->updatePointCloud(spCamera->captureFrame()->generatePointCloud());
//		if (!spCameraViewer->wasStopped())
//			spCameraViewer->spinOnce(100);
//
//		if (!spAlignmentViewer->wasStopped())
//			spAlignmentViewer->spinOnce(100);
//		if (!spErrorViewer->wasStopped())
//			spErrorViewer->spinOnce(100);
//		std::this_thread::sleep_for(1ms);
//	}
//
//	return EXIT_SUCCESS;
//}
