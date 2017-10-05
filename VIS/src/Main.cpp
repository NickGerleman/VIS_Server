#include "pch.h"
#include "ErrorDetection.h"
#include "Geometry.h"
#include "MeshIO.h"
#include "Visualization.h"


int main(int argc, char** argv)
{
	auto spIdealMesh = vis::tryLoadBinaryStlFile("models/cube.stl");
	auto spCaptureMesh = vis::tryLoadBinaryStlFile("captures/cubert_realsense.stl");
	auto spCaptureCloud = boost::make_shared<vis::PointCloud>(spCaptureMesh->getVertexDataCloud());

	vis::AlignmentQuality quality;
	auto spIdealSurface = vis::alignPointCloud(*spIdealMesh, spCaptureCloud, quality);
	auto spErrorCloud = vis::createErrorCloud(spIdealSurface, *spCaptureCloud);

	auto spAlignmentViewer = vis::createAlignmentVisualizer(spIdealSurface, spCaptureCloud);
	auto spErrorViewer = vis::createErrorVisualizer(*spErrorCloud);
	while (!(spAlignmentViewer->wasStopped() && spErrorViewer->wasStopped()))
	{
		if (!spAlignmentViewer->wasStopped())
			spAlignmentViewer->spinOnce(100);
		if (!spErrorViewer->wasStopped())
			spErrorViewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return EXIT_SUCCESS;
}