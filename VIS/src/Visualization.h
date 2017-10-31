#pragma once

#include "Camera.h"
#include "Geometry.h"

namespace vis
{

	/// Logically immutable record of various states of progress
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


	/// Visualizer that will show various states of the program. Calls are non-blocking,
	/// and generally quick. The actual vizualization happens on its own thread, and is
	/// currently horribly inefficient to a noticeable degree. Work should be done on
	/// reducing unneeded sorts and to not recompute colored clouds on each loop
	/// iteration.
	class ProgressVisualizer
	{
		using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;

	public:

		/// Get the singleton instance
		static ProgressVisualizer* get();
		

		/// Start the visualizer thread
		void start();


		/// Notify that a new scan has started and that we should clear our viewports
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


		/// Notify that an alignment attempt is in process for the cloud
		/// @param spCloud the relevant frame
		void notifyAlignment(const boost::shared_ptr<PointCloud>& spCloud);


		/// Notify that an error cloud has been calculated
		/// @param spCloud the error cloud
		void notifyErrorCloud(const boost::shared_ptr<ErrorPointCloud>& spCloud);

	private:

		ProgressVisualizer()
			: m_vizThreadRecord
			(
				nullptr,
				nullptr,
				nullptr,
				nullptr,
				nullptr
			) {}


		/// Create viewports and add pointclouds to them
		void initViewports();


		/// Copy our current progress to a progress record for the visualization thread
		/// to read
		void pushIfNeeded();


		/// Update the displayed point clouds based on the current record
		void updateDisplay();
		void updateDepthFrameView();
		void updateClipFrameView();
		void updateSurfaceCloudView();
		void updateAlignmentView();
		void updateErrorView();
		void updateText();


		/// Place the caemra in a viewport such that the point cloud is centered and
		/// fully visible
		/// @param viewport the viewport number
		/// @param cloud the pointcloud in the viewport
		template <typename TCloud>
		void fitCameraToCloud(int viewport, const TCloud& cloud);

		std::thread m_visualizationThread;
		std::mutex m_lastUpdateLock;
		TimePoint m_lastPushTime;
		std::unique_ptr<pcl::visualization::PCLVisualizer> m_spVisualizer;
		ProgressRecord m_vizThreadRecord;

		boost::shared_ptr<PointCloud> m_spLastSurfaceCloud;
		int m_surfaceCloudViewport;

		std::shared_ptr<DepthFrame> m_spLastDepthFrame;
		int m_depthFrameViewport;

		std::shared_ptr<DepthFrame> m_spLastClipFrame;
		int m_clipFrameViewport;

		// TODO implement reconstruction viewport
		int m_reconstructionViewport;

		boost::shared_ptr<PointCloud> m_spLastAlignment;
		int m_alignmentViewport;

		boost::shared_ptr<ErrorPointCloud> m_spLastErrorCloud;
		int m_errorViewport;
	};

	
	/// Create coloring to show depth on a point cloud
	/// @param cloud the cloud with depth information
	boost::shared_ptr<ColorPointCloud> colorDepthCloud(
		const PointCloud& cloud,
		const Eigen::Vector3f& closeColor = Eigen::Vector3f(0.8f, 0.8f, 0.8f),
		const Eigen::Vector3f& farColor = Eigen::Vector3f(0.2f, 0.2f, 0.2f)
	);


	/// Create a colored point by linearly interpolating between two colors
	/// @param minColor the color corresponding to the minimum value
	/// @param maxColor the color corresponding to the max value
	/// @param minValue the minimum value for the gradient
	/// @param maxValue the maximum value for the gradient
	/// @param point the point whose coordinates will be used
	/// @param the value of the point to use for coloring, will be clamped to be between min
	///        and max
	static pcl::PointXYZRGB colorPoint(
		const Eigen::Vector3f& minColor,
		const Eigen::Vector3f& maxColor,
		float minValue,
		float maxValue,
		pcl::PointXYZ point,
		float pointValue);


	/// Find a point in a pointcloud at a normalized position given sorted order. This is
	/// super inefficient and should be removed eventually.
	/// @param cloud the point cloud
	/// @param samplePlace a number between 0 and 1 corresponding to the place to sample the
	///        cloud
	/// @param compare a comparison functor used for sorting
	template <typename TPoint, typename TComp>
	static TPoint samplePoint(const pcl::PointCloud<TPoint>& cloud, double samplePlace, TComp compare);

}
