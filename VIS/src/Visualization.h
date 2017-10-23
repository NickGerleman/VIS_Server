#pragma once

#include "Camera.h"
#include "Geometry.h"

namespace vis
{

	/// Fullscreen visualizer that will show various states of the program.
	/// Calls are non-blocking, and generally quick.
	class ProgressVisualizer
	{
	public:

		/// Notify that we have begun a new scan
		void notifyNewScan();
		
		
		/// Notify that we have received a new depth frame from the camera
		/// @param spFrame the frame
		void notifyDepthFrame(const std::shared_ptr<DepthFrame>& spFrame);


		/// Notify the visualizer that progress has been made in reconsturcting the 
		/// @param spCloud the a referece to the current cloud
		void notifySurfaceCloud(const boost::shared_ptr<PointCloud>& spCloud);


		/// Notify that a frame has been clipped
		/// @param spFrame the relevant frame
		void notifyClipFrame(const std::shared_ptr<DepthFrame>& spFrame);


		/// Notify that a bounding sphere has been determined or moved 
		/// @param spShere the bounding sphere
		void notifyBoundingSphere(const boost::shared_ptr<BoundingSphere>& spSphere);


		/// Notify that an error cloud has been calculated
		/// @param spCloud the error cloud
		void notifyErrorCloud(const boost::shared_ptr<ErrorPointCloud>& spCloud);

	private:
		std::thread m_visualizationThread;
		std::mutex m_queueLock;
		std::queue<std::function<void()>> m_RunQueue;
	};


	/// Create a visualizer showing the alignment between two point clouds
	/// @param spReference the reference cloud, shown in white
	/// @param spTarget the target, shown in blue
	std::shared_ptr<pcl::visualization::PCLVisualizer> createAlignmentVisualizer(const boost::shared_ptr<PointCloud>& spReference, const boost::shared_ptr<PointCloud>& spTarget);


	/// Create a visualizer showing the relative error in an error cloud (in red)
	/// @param cloud the error cloud
	std::shared_ptr<pcl::visualization::PCLVisualizer> createErrorVisualizer(const ErrorPointCloud& cloud);

}
