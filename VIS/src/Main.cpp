#include "pch.h"
#include "error_detection_module/ErrorDetection.h"
#include "Geometry.h"
#include "MeshIO.h"


boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr spCloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr spCloud2)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	viewer->addPointCloud<pcl::PointXYZ>(spCloud1, "c1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "c1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "c1");

	viewer->addPointCloud<pcl::PointXYZ>(spCloud2, "c2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "c2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "c2");

	//viewer->addCoordinateSystem (1.0, "global");
	viewer->initCameraParameters();
	return (viewer);
}


int main(int argc, char** argv)
{
	auto spIdealMesh = vis::tryLoadBinaryStlFile("models/cube.stl");
	
	auto spCaptureMesh = vis::tryLoadBinaryStlFile("captures/cubert_realsense.stl");
	auto spCaptureCloud = boost::make_shared<vis::PointCloud>(spCaptureMesh->getVertexDataCloud());

	vis::AlignmentQuality quality;
	auto spIdealSurface = alignPointCloud(*spIdealMesh, spCaptureCloud, quality);
	pcl::PointCloud<pcl::PointXYZ>::Ptr spCenteredDefects = detectErrors(spIdealSurface, spCaptureCloud);

	// creates the visualization object and adds either our orignial cloud or all of the inliers
	// depending on the command line arguments specified.
	auto spViewer1 = simpleVis(spIdealSurface, spCaptureCloud);
	auto spViewer2 = simpleVis(spIdealSurface, spCenteredDefects);
	while (!spViewer1->wasStopped())
	{
		if (!spViewer1->wasStopped())
			spViewer1->spinOnce(100);
		if (!spViewer2->wasStopped())
			spViewer2->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}