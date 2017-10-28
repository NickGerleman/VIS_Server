#pragma once

#include "Camera.h"
#include "Geometry.h"

namespace vis
{

	///
	struct ProgressRecord
	{

		ProgressRecord(
			const std::shared_ptr<DepthFrame>& spDepthFrame,
			const boost::shared_ptr<PointCloud>& spSurfaceCloud,
			const std::shared_ptr<DepthFrame>& spClipFrame,
			const boost::shared_ptr<PointCloud>& spAlignment,
			const boost::shared_ptr<ErrorPointCloud>& spErrorCloud
		);

		std::shared_ptr<DepthFrame> spDepthFrame;
		boost::shared_ptr<PointCloud> spSurfaceCloud;
		std::shared_ptr<DepthFrame> spClipFrame;
		boost::shared_ptr<PointCloud> spAlignment;
		boost::shared_ptr<ErrorPointCloud> spErrorCloud;
	};


	/// Visualizer that will show various states of the program. Calls are non-blocking, and
	/// generally quick.
	class ProgressVisualizer
	{
		using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;

	public:

		/// Get the singleton instance
		static ProgressVisualizer* get();
		

		/// Start the visualizer thread
		void start();

		
		/// Notify that we have received a new depth frame from the camera
		/// @param spFrame the frame
		void notifyDepthFrame(const std::shared_ptr<DepthFrame>& spFrame);


		/// Notify the visualizer that progress has been made in reconsturcting the 
		/// @param spCloud the a referece to the current cloud
		void notifySurfaceCloud(const boost::shared_ptr<PointCloud>& spCloud);


		/// Notify that a frame has been clipped
		/// @param spFrame the relevant frame
		void notifyClipFrame(const std::shared_ptr<DepthFrame>& spFrame);


		/// Notify that an alignment attempt is in process for the cloud
		/// @param spCloud the relevant frame
		void notifyAlignment(const boost::shared_ptr<PointCloud>& spCloud);


		/// Notify that an error cloud has been calculated
		/// @param spCloud the error cloud
		void notifyErrorCloud(const boost::shared_ptr<ErrorPointCloud>& spCloud);

	private:

		///
		ProgressVisualizer()
			: m_visThreadRecord(nullptr, nullptr, nullptr, nullptr, nullptr) {}


		///
		void pushIfNeeded();


		///
		static void updateDisplay(pcl::visualization::PCLVisualizer& visualizer, const ProgressRecord& record);


		std::thread m_visualizationThread;
		std::mutex m_lastUpdateLock;
		TimePoint m_lastPushTime;

		ProgressRecord m_visThreadRecord;
		std::shared_ptr<DepthFrame> m_spLastDepthFrame;
		boost::shared_ptr<PointCloud> m_spLastSurfaceCloud;
		std::shared_ptr<DepthFrame> m_spLastClipFrame;
		boost::shared_ptr<PointCloud> m_spLastAlignment;
		boost::shared_ptr<ErrorPointCloud> m_spLastErrorCloud;

	};


	/// Create a visualizer showing the alignment between two point clouds
	/// @param spReference the reference cloud, shown in white
	/// @param spTarget the target, shown in blue
	std::shared_ptr<pcl::visualization::PCLVisualizer> createAlignmentVisualizer(const boost::shared_ptr<PointCloud>& spReference, const boost::shared_ptr<PointCloud>& spTarget);


	/// Create a visualizer showing the relative error in an error cloud (in red)
	/// @param cloud the error cloud
	std::shared_ptr<pcl::visualization::PCLVisualizer> createErrorVisualizer(const ErrorPointCloud& cloud);

}
