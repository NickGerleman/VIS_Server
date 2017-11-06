#pragma once

#include "Camera.h"
#include "Geometry.h"

namespace vis
{

	/// Visualizer that will show various states of the program. Calls will copy any data.
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


		/// Notify that we have received a new depth frame from the camera.
		/// @param spFrame the frame
		void notifyDepthFrame(const std::shared_ptr<DepthFrame>& spFrame);


		/// Notify the visualizer that progress has been made in reconsturcting
		/// the surface of an object. The point cloud should not be modified
		/// after notifying the visualizer.
		/// @param spCloud the a referece to the current cloud
		void notifySurfaceCloud(const boost::shared_ptr<const PointCloud>& spCloud);


		/// Notify that a frame has been clipped
		/// @param spFrame the relevant frame
		/// @param spClip the clipping volume used
		void notifyClipFrame(const std::shared_ptr<DepthFrame>& spFrame, const std::shared_ptr<ClipVolume>& spClip);


		/// Notify that an alignment attempt is in process for the cloud. The
		/// point cloud should not be modified after notifying the visualizer.
		/// @param spCloud the relevant frame
		void notifyAlignment(const boost::shared_ptr<const PointCloud>& spCloud);


		/// Notify that an error cloud has been calculated. after notifying the visualizer.
		/// @param spCloud the error cloud
		void notifyErrorCloud(const boost::shared_ptr<const ErrorPointCloud>& spCloud);


		/// Notify that a frame has been processed in reconstruction
		/// @param spFrame a color frame to show
		void notifyReconstruction(FusionFramePtr spFrame);


	private:


		ProgressVisualizer() = default;


		/// Create viewports and add pointclouds to them
		void initViewports();


		/// Update the displayed point clouds based on the current record
		void updateDisplay();
		void updateDepthFrameView();
		void updateClipFrameView();
		void updateSurfaceCloudView();
		void updateAlignmentView();
		void updateErrorView();
		void updateReconView();
		void updateText();


		/// Place the caemra in a viewport such that the point cloud is centered and
		/// fully visible
		/// @param viewport the viewport number
		/// @param cloud the pointcloud in the viewport
		template <typename TCloud>
		void fitCameraToCloud(int viewport, const TCloud& cloud);

		std::unique_ptr<pcl::visualization::PCLVisualizer> m_spVisualizer;

		boost::shared_ptr<const PointCloud> m_spSurfaceCloud;
		bool m_surfaceCloudDirty;
		std::mutex m_surfaceCloudMutex;
		int m_surfaceCloudViewport;

		std::shared_ptr<DepthFrame> m_spDepthFrame;
		bool m_depthFrameDirty;
		std::mutex m_depthFrameMutex;
		int m_depthFrameViewport;

		std::shared_ptr<DepthFrame> m_spClipFrame;
		std::shared_ptr<ClipVolume> m_spClipVolume;
		bool m_clipFrameDirty;
		std::mutex m_clipFrameMutex;
		int m_clipFrameViewport;

		FusionFramePtr m_spReconFrame;
		bool m_reconFrameDirty;
		std::mutex m_reconFrameMutex;
		int m_reconstructionViewport;

		boost::shared_ptr<const PointCloud> m_spAlignedCloud;
		bool m_alignedCloudDirty;
		std::mutex m_alignedCloudMutex;
		int m_alignedCloudViewport;

		boost::shared_ptr<const ErrorPointCloud> m_spErrorCloud;
		bool m_errorCloudDirty;
		std::mutex m_errorCloudMutex;
		int m_errorCloudViewport;
	};

	
	/// Create coloring to show depth on a point cloud
	/// @param cloud the cloud with depth information
	boost::shared_ptr<ColorPointCloud> colorDepthCloud(
		const PointCloud& cloud,
		const Eigen::Vector3f& closeColor = Eigen::Vector3f(1.0f, 1.0f, 1.0f),
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
