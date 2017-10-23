#pragma once

#include "Geometry.h"

namespace vis
{

	/// Clipping cube of arbitraty translation and rotation used to isolate objects in a
	/// scene
	class ClipCube
	{
	public:
		
		/// Build the clip cube using a transform to a reference 1x1 cube centered at
		/// the origin
		/// @param referenceTransform the transform to achieve the cube position
		ClipCube(const Eigen::Matrix4f& referenceTransform);


		/// Whether the given point is inside of the cube
		/// @param point a given point
		bool intersects(const pcl::PointXYZ& point) const;


		/// Transform a point in world space to the cubes coordinate system
		/// @param point the point in world space
		pcl::PointXYZ relativeCoordinate(const pcl::PointXYZ& point) const;

	private:
		Eigen::Matrix4f m_inverseTransform;

	};


	/// Generic depth frame acquired from the camera
	class DepthFrame
	{
	public:
		using FusionFrameDeleter = void(*)(NUI_FUSION_IMAGE_FRAME*);

		virtual ~DepthFrame() = default;


		/// Build a point cloud from the depth frame (with 1M units)
		virtual boost::shared_ptr<PointCloud> generatePointCloud() = 0;


		/// Width of the frame in pixels
		int width();


		/// Height of the frame in pixels
		int height();


		/// Clip the depth frame such that it only contains points within the cube
		/// @param clipCube the cube to clip to
		virtual std::shared_ptr<DepthFrame> clipFrame(const ClipCube& clipCube) = 0;
		

		/// Convert the frame into a KinectFusion depth float frame (in 1M units).
		/// Release must be called on the underlying texture after use.
		virtual std::unique_ptr<NUI_FUSION_IMAGE_FRAME, FusionFrameDeleter> convertToFusionFrame() = 0;


	protected:
		DepthFrame(int width, int height)
			: m_width(width)
			, m_height(height) {}


	private:
		int m_width;
		int m_height;
	};


	/// Depth frame for OpenNI cameras
	class OpenNIFrame : public DepthFrame
	{
	public:

		/// Create the frame from an OpenNI native frame
		/// @param depthFrame the frame containing pixel data
		/// @param depthDtream the stream from the camera where the frame was obtained
		/// @param pCameraParams camera parameters to use in converstion to a fustion frame
		OpenNIFrame(const openni::VideoFrameRef& depthFrame, std::shared_ptr<openni::VideoStream> spDepthStream, const NUI_FUSION_CAMERA_PARAMETERS* pCameraParams);
		
		OpenNIFrame(const OpenNIFrame& otherFrame) = default;

		boost::shared_ptr<PointCloud> generatePointCloud() override;
		std::shared_ptr<DepthFrame> clipFrame(const ClipCube& clipCube) override;
		std::unique_ptr<NUI_FUSION_IMAGE_FRAME, FusionFrameDeleter> convertToFusionFrame() override;

	private:
		
		/// Returns the world coordiates, if any, for the pixel at the given
		/// position
		/// @param x the x coordiate in pixel space
		/// @param y the y coordiate in pixel space
		boost::optional<pcl::PointXYZ> worldPtFromPixel(int x, int y);

		std::vector<uint16_t> m_depthPixels;
		std::shared_ptr<openni::VideoStream> m_spDepthStream;
		const NUI_FUSION_CAMERA_PARAMETERS* m_pCameraParams;
	};


	/// Generic interface for a camera handle
	class ICamera
	{

	public:

		/// Capture the latest frame from the device
		virtual std::shared_ptr<DepthFrame> captureFrame() = 0;
	};


	/// Handle to OpenNI camera
	class OpenNICamera : public ICamera
	{
		friend class StructureSensor;

	public:

		~OpenNICamera();
		std::shared_ptr<DepthFrame> captureFrame() override;

	protected:

		/// Build a handle to the camera stream
		/// @param spDevice the OpenNI native device handle
		/// @param spDepthDtream the OpenNI native video stream
		/// @param threadLock a mutex held by the camera handle
		/// @pram pCameraParams camera intrinsics ised for reconstruction
		OpenNICamera(
			std::unique_ptr<openni::Device> spDevice,
			std::shared_ptr<openni::VideoStream> spDepthStream,
			std::unique_lock<std::recursive_mutex>&& threadLock,
			const NUI_FUSION_CAMERA_PARAMETERS* pCameraParams
		)
			: m_spDevice(std::move(spDevice))
			, m_spDepthStream(std::move(spDepthStream))
			, m_threadLock(std::move(threadLock))
			, m_pCameraParams(pCameraParams) {}

		std::shared_ptr<openni::VideoStream> m_spDepthStream;

	private:

		std::unique_ptr<openni::Device> m_spDevice;
		std::unique_lock<std::recursive_mutex> m_threadLock;
		const NUI_FUSION_CAMERA_PARAMETERS* m_pCameraParams;

	};


	/// Camera that plays back data according to an ONI file. The camera will repeatedly
	/// loop the first frame until told to playback the entire stream (for
	/// platform rotation)
	class MockCamera : public OpenNICamera
	{
	public:

		/// Build the camera
		/// @param filepath a path to an ONI file
		/// @param pCameraParams intrinsics for the camera used
		static std::shared_ptr<MockCamera> make(const std::string& filepath, const NUI_FUSION_CAMERA_PARAMETERS* pCameraParams);


		/// Start showing the entirety of the footage, resuming looping at the first
		/// frame after it is done.
		void startFullLoop();


		/// Whether the camera is looping the first frame only
		bool isStatic();


		std::shared_ptr<DepthFrame> captureFrame() override;

	private:

		MockCamera(
			std::unique_ptr<openni::Device> spDevice,
			std::shared_ptr<openni::VideoStream> spDepthStream,
			std::unique_lock<std::recursive_mutex>&& threadLock,
			const NUI_FUSION_CAMERA_PARAMETERS* pCameraParams,
			openni::PlaybackControl* pControls
		)
			: OpenNICamera(std::move(spDevice), spDepthStream, std::move(threadLock), pCameraParams)
			, m_pControls(pControls)
			, m_static(true)
			, m_curFrame(0) {}


		openni::PlaybackControl* m_pControls;
		bool m_static;
		int m_curFrame;
	};


	/// Access point to acquire a handle to the structure sensor set up for VGA
	/// resolution. This will fail if multiple OpenNI devices are present at once.
	/// Multiple users of the camera can be present at once so long as they are
	/// on the same thread.
	class StructureSensor
	{
	public:

		/// Get a camera handle for the structure sensor. This will block if another
		/// thread has a handle.
		static std::unique_ptr<OpenNICamera> acquireCamera();


		/// Retrive instrinsics for the structure sensor
		static const NUI_FUSION_CAMERA_PARAMETERS* cameraIntrinsics();
	};

}
