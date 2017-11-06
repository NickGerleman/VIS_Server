#include "pch.h"
#include "Visualization.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace pcl;
using namespace visualization;

namespace vis
{
	static const int NUM_ROWS = 2;
	static const int NUM_COLS = 3;

	/// Used to determine how many points are sampled from a frame to generate a point cloud
	static const int FRAME_SAMPLE_STEP = 2;
	static const int POINT_SIZE = 2;

	/// Tags used to idemtify each point cloud
	static const char* DEPTH_FRAME_TAG     = "DEPTH_FRAME_TAG";
	static const char* CLIP_FRAME_TAG      = "CLIP_FRAME_TAG";
	static const char* RECONSTRUCTION_TAG  = "RECONSTRUCTION_TAG";
	static const char* SURFACE_CLOUD_TAG   = "SURFACE_CLOUD_TAG";
	static const char* ALIGNMENT_TAG       = "ALIGNMENT_TAG";
	static const char* ERROR_CLOUD_TAG     = "ERROR_CLOUD_TAG";

	/// Text for each viewport
	static const char* DEPTH_FRAME_TEXT    = "World Depth";
	static const char* CLIP_FRAME_TEXT     = "Clipped Depth";
	static const char* RECONSTRUCTION_TEXT = "3D Reconstruction";
	static const char* SURFACE_CLOUD_TEXT  = "Surface Construction";
	static const char* ALIGNMENT_TEXT      = "Alignment";
	static const char* ERROR_CLOUD_TEXT    = "Error Levels";


	/*static*/ ProgressVisualizer* ProgressVisualizer::get()
	{
		static ProgressVisualizer* pViz = nullptr;
		if (pViz == nullptr)
			pViz = new ProgressVisualizer;
		return pViz;
	}


	void ProgressVisualizer::start()
	{
		static bool isInitialized = false;
		if (isInitialized)
			return;
		isInitialized = true;

		std::thread([this]()
		{
			m_spVisualizer = std::make_unique<::PCLVisualizer>();
			initViewports();

			m_spVisualizer->setWindowName("Visualizer");
			m_spVisualizer->setBackgroundColor(0.1, 0.1, 0.1);
			m_spVisualizer->setShowFPS(false);

			while (true)
			{
				updateDisplay();
				m_spVisualizer->spinOnce();
				std::this_thread::sleep_for(1ms);
			}

		}).detach();
	}


	void ProgressVisualizer::notifyNewScan()
	{
		std::lock_guard<std::mutex> depthLock(m_depthFrameMutex);
		std::lock_guard<std::mutex> surfaceLock(m_surfaceCloudMutex);
		std::lock_guard<std::mutex> clipLock(m_clipFrameMutex);
		std::lock_guard<std::mutex> alignLock(m_alignedCloudMutex);
		std::lock_guard<std::mutex> errorLock(m_errorCloudMutex);
		std::lock_guard<std::mutex> reconLock(m_reconFrameMutex);

		m_spDepthFrame = nullptr;
		m_spSurfaceCloud = nullptr;
		m_spClipFrame = nullptr;
		m_spAlignedCloud = nullptr;
		m_spErrorCloud = nullptr;
		m_spReconFrame = nullptr;

		m_depthFrameDirty
			= m_clipFrameDirty
			= m_surfaceCloudDirty
			= m_alignedCloudDirty
			= m_errorCloudDirty
			= m_reconFrameDirty
			= true;
	}


	void ProgressVisualizer::notifyDepthFrame(const std::shared_ptr<DepthFrame>& spFrame)
	{
		std::lock_guard<std::mutex> depthLock(m_depthFrameMutex);
		m_spDepthFrame = spFrame;
		m_depthFrameDirty = true;
	}


	void ProgressVisualizer::notifySurfaceCloud(const boost::shared_ptr<const PointCloud>& spCloud)
	{
		std::lock_guard<std::mutex> surfaceLock(m_surfaceCloudMutex);
		m_spSurfaceCloud = spCloud;
		m_surfaceCloudDirty = true;
	}


	void ProgressVisualizer::notifyClipFrame(const std::shared_ptr<DepthFrame>& spFrame, const std::shared_ptr<ClipVolume>& spClip)
	{
		std::lock_guard<std::mutex> clipLock(m_clipFrameMutex);
		m_spClipFrame = spFrame;
		m_spClipVolume = spClip;
		m_clipFrameDirty = true;
	}


	void ProgressVisualizer::notifyAlignment(const boost::shared_ptr<const PointCloud>& spCloud)
	{
		std::lock_guard<std::mutex> alignLock(m_alignedCloudMutex);
		m_spAlignedCloud = spCloud;
		m_alignedCloudDirty = true;
	}


	void ProgressVisualizer::notifyErrorCloud(const boost::shared_ptr<const ErrorPointCloud>& spCloud)
	{
		std::lock_guard<std::mutex> errorLock(m_errorCloudMutex);
		m_spErrorCloud = spCloud;
		m_errorCloudDirty = true;
	}


	void ProgressVisualizer::notifyReconstruction(FusionFramePtr spFrame)
	{
		std::lock_guard<std::mutex> reconLock(m_reconFrameMutex);
		m_spReconFrame = std::move(spFrame);
		m_reconFrameDirty = true;
	}


	static void initViewport(PCLVisualizer& visualizer, int row, int col, const char* tag, const char* text, int& viewportId)
	{
		double minX = col * (1.0 / NUM_COLS);
		double maxX = minX + (1.0 / NUM_COLS);

		double maxY = 1 - (row * 1.0 / NUM_ROWS);
		double minY = maxY - (1.0 / NUM_ROWS);

		visualizer.createViewPort(minX, minY, maxX, maxY, viewportId);
		visualizer.addText(text, 20, 20, text, viewportId);
		visualizer.addPointCloud(boost::make_shared<ColorPointCloud>(), tag, viewportId);
		visualizer.setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, POINT_SIZE, tag);
		visualizer.createViewPortCamera(viewportId);
	}


	void ProgressVisualizer::initViewports()
	{
		initViewport(*m_spVisualizer, 0, 0, DEPTH_FRAME_TAG,    DEPTH_FRAME_TEXT,    m_depthFrameViewport);
		initViewport(*m_spVisualizer, 0, 1, CLIP_FRAME_TAG,     CLIP_FRAME_TEXT,     m_clipFrameViewport);
		initViewport(*m_spVisualizer, 0, 2, RECONSTRUCTION_TAG, RECONSTRUCTION_TEXT, m_reconstructionViewport);
		initViewport(*m_spVisualizer, 1, 0, SURFACE_CLOUD_TAG,  SURFACE_CLOUD_TEXT,  m_surfaceCloudViewport);
		initViewport(*m_spVisualizer, 1, 1, ALIGNMENT_TAG,      ALIGNMENT_TEXT,      m_alignedCloudViewport);
		initViewport(*m_spVisualizer, 1, 2, ERROR_CLOUD_TAG,    ERROR_CLOUD_TEXT,    m_errorCloudViewport);
	}


	void ProgressVisualizer::updateDisplay()
	{
		if (m_depthFrameDirty)
			updateDepthFrameView();

		if (m_clipFrameDirty)
			updateClipFrameView();

		if (m_surfaceCloudDirty)
			updateSurfaceCloudView();

		if (m_alignedCloudDirty)
			updateAlignmentView();

		if (m_errorCloudDirty)
			updateErrorView();

		if (m_reconFrameDirty)
			updateReconView();

		m_depthFrameDirty
			= m_clipFrameDirty
			= m_surfaceCloudDirty
			= m_alignedCloudDirty
			= m_errorCloudDirty
			= m_reconFrameDirty
			= false;
	}


	void ProgressVisualizer::updateDepthFrameView()
	{
		std::lock_guard<std::mutex> depthLock(m_depthFrameMutex);
		if (!m_spDepthFrame)
		{
			m_spVisualizer->updatePointCloud(boost::make_shared<ColorPointCloud>(), DEPTH_FRAME_TAG);
			return;
		}

		auto spDepthCloud = m_spDepthFrame->generatePointCloud(FRAME_SAMPLE_STEP);
		if (spDepthCloud->empty())
			return;
		auto spColorDepthCloud = colorDepthCloud(*spDepthCloud);
		
		m_spVisualizer->updatePointCloud(spColorDepthCloud, DEPTH_FRAME_TAG);
		fitCameraToCloud(m_depthFrameViewport, *spColorDepthCloud);
	}


	void ProgressVisualizer::updateClipFrameView()
	{
		std::lock_guard<std::mutex> clipLock(m_clipFrameMutex);
		if (!m_spClipFrame)
		{
			m_spVisualizer->updatePointCloud(boost::make_shared<ColorPointCloud>(), CLIP_FRAME_TAG);
			return;
		}

		auto spClipCloud = m_spClipFrame->generatePointCloud(FRAME_SAMPLE_STEP);
		for (auto& pt : *spClipCloud)
			pt = m_spClipVolume->relativeCoordinate(pt);

		if (spClipCloud->empty())
			return;

		auto spColorDepthCloud = colorDepthCloud(*spClipCloud);
		
		m_spVisualizer->updatePointCloud(spColorDepthCloud, CLIP_FRAME_TAG);
		fitCameraToCloud(m_clipFrameViewport, *spColorDepthCloud);
	}


	void ProgressVisualizer::updateSurfaceCloudView()
	{
		std::lock_guard<std::mutex> surfaceLock(m_surfaceCloudMutex);
		if (!m_spSurfaceCloud)
		{
			m_spVisualizer->updatePointCloud(boost::make_shared<ColorPointCloud>(), SURFACE_CLOUD_TAG);
			return;
		}

		auto spColorDepthCloud = colorDepthCloud(*m_spSurfaceCloud);
		m_spVisualizer->updatePointCloud(spColorDepthCloud, SURFACE_CLOUD_TAG);
		fitCameraToCloud(m_surfaceCloudViewport, *spColorDepthCloud);
	}


	void ProgressVisualizer::updateAlignmentView()
	{
		std::lock_guard<std::mutex> alignLock(m_alignedCloudMutex);
		if (!m_spAlignedCloud)
		{
			m_spVisualizer->updatePointCloud(boost::make_shared<ColorPointCloud>(), ALIGNMENT_TAG);
			return;
		}

		auto spComposite = colorDepthCloud(*m_spSurfaceCloud);
		auto spCapture = colorDepthCloud(*m_spAlignedCloud, {0.8f, 0.2f, 0.2f}, {0.2f, 0.2f, 0.2f});
		spComposite->insert(spComposite->end(), spCapture->begin(), spCapture->end());

		m_spVisualizer->updatePointCloud(spComposite, ALIGNMENT_TAG);
		fitCameraToCloud(m_alignedCloudViewport, *spComposite);
	}


	void ProgressVisualizer::updateErrorView()
	{
		std::lock_guard<std::mutex> errorLock(m_errorCloudMutex);
		if (!m_spErrorCloud)
		{
			m_spVisualizer->updatePointCloud(boost::make_shared<ColorPointCloud>(), ERROR_CLOUD_TAG);
			return;
		}
		const auto& spErrorCloud = m_spErrorCloud;

		auto compareError = [](auto& p1, auto& p2)
		{
			return p1.intensity < p2.intensity;
		};

		// Start gradient at error outliers
		auto quartile1 = samplePoint(*spErrorCloud, 0.25, compareError).intensity;
		auto quartile3 = samplePoint(*spErrorCloud, 0.75, compareError).intensity;
		auto iqr = quartile3 - quartile1;
		auto gradientMin = quartile3 + (1.5f * iqr);
		auto gradientMax = (*std::max_element(spErrorCloud->begin(), spErrorCloud->end(), compareError)).intensity;
		
		auto spColorCloud = boost::make_shared<ColorPointCloud>();
		for (const auto& pt : *spErrorCloud)
		{
			PointXYZ xyzPoint(pt.x, pt.y, pt.z);
			spColorCloud->push_back(colorPoint({0.8f, 0.8f, 0.8f}, { 0.8f, 0.0f, 0.0f }, gradientMin, gradientMax, xyzPoint, pt.intensity));
		}
		m_spVisualizer->updatePointCloud(spColorCloud, ERROR_CLOUD_TAG);
		fitCameraToCloud(m_errorCloudViewport, *spColorCloud);
	}


	void ProgressVisualizer::updateReconView()
	{
		std::lock_guard<std::mutex> reconLock(m_reconFrameMutex);
		if (!m_spReconFrame)
		{
			m_spVisualizer->updatePointCloud(boost::make_shared<ColorPointCloud>(), RECONSTRUCTION_TAG);
			return;
		}

		// Treat the frame like a point cloud along one plane
		auto spCloud = boost::make_shared<ColorPointCloud>();
		uint32_t* pPixelsArgb = reinterpret_cast<uint32_t*>(m_spReconFrame->pFrameBuffer->pBits);
		uint32_t pxStride = m_spReconFrame->pFrameBuffer->Pitch / sizeof(uint32_t);

		for (UINT y = 0; y < m_spReconFrame->height; y++)
		{
			for (UINT x = 0; x < m_spReconFrame->width; x++)
			{
				uint32_t pixel = pPixelsArgb[(y * pxStride) + x];
				uint8_t a = static_cast<uint8_t>(pixel >> 24);
				uint8_t r = static_cast<uint8_t>(pixel >> 16);
				uint8_t g = static_cast<uint8_t>(pixel >> 8);
				uint8_t b = static_cast<uint8_t>(pixel);

				PointXYZRGB pt(r, g, b);
				pt.x = static_cast<float>(x);
				pt.y = -static_cast<float>(y);
				pt.z = -100.0f;
				spCloud->push_back(pt);
			}

		}

		m_spVisualizer->updatePointCloud(spCloud, RECONSTRUCTION_TAG);
		fitCameraToCloud(m_reconstructionViewport, *spCloud);
	}


	template <typename TCloud>
	void ProgressVisualizer::fitCameraToCloud(int viewport, const TCloud& cloud)
	{
		std::vector<Camera> viewportCameras;
		m_spVisualizer->getCameras(viewportCameras);
		auto& ourCamera = viewportCameras[viewport];
		
		// Set up-vector
		ourCamera.view[0] = 0.0;
		ourCamera.view[1] = 1.0;
		ourCamera.view[2] = 0.0;

		// Find bounds of current point cloud
		float minX, maxX, minY, maxY, minZ, maxZ;
		minX = minY = minZ = FLT_MAX;
		maxX = maxY = maxZ = FLT_MIN;
		for (auto& pt : cloud)
		{
			minX = std::fmin(minX, pt.x);
			maxX = std::fmax(maxX, pt.x);
			minY = std::fmin(minY, pt.y);
			maxY = std::fmax(maxY, pt.y);
			minZ = std::fmax(maxZ, pt.z);
			maxZ = std::fmax(maxZ, pt.z);
		}
		
		// Center the camera on X and Y
		float dX = maxX - minX;
		float dY = maxY - minY;
		ourCamera.pos[0] = minX + (dX / 2);
		ourCamera.pos[1] = minY + (dY / 2);

		ourCamera.focal[0] = ourCamera.pos[0];
		ourCamera.focal[1] = ourCamera.pos[1];
		ourCamera.focal[2] = maxZ;

		// Use FOV to calculate Z pos (with padding)
		const double PADDING_FACTOR = 1.2;
		auto aspectRatio = ((ourCamera.window_size[0] / NUM_COLS) / (ourCamera.window_size[1] / NUM_ROWS));
		auto fovX = 2.0 * atan(tan(ourCamera.fovy / 2.0) * aspectRatio);
		
		auto thetaX = fovX / 2;
		auto zFitX = (1 / tan(thetaX)) * (dX * PADDING_FACTOR / 2);
		auto thetaY = ourCamera.fovy / 2;
		auto zFitY = (1 / tan(thetaY)) * (dY * PADDING_FACTOR / 2);

		ourCamera.pos[2] = maxZ + std::fmax(zFitX, zFitY);

		// Choose some reasonable clip planes based on Z pos
		ourCamera.clip[0] = ourCamera.pos[2];
		ourCamera.clip[1] = minZ;

		// Finally set
		m_spVisualizer->setCameraParameters(ourCamera, viewport);
	}


	boost::shared_ptr<ColorPointCloud> colorDepthCloud(const PointCloud& cloud, const Eigen::Vector3f& closeColor, const Eigen::Vector3f& farColor)
	{
		auto compareDepth = [](const auto& pt1, const auto& pt2)
		{
			return pt1.z < pt2.z;
		};

		auto closePt = samplePoint(cloud, 0.95, compareDepth);
		auto farPt = samplePoint(cloud, 0.05, compareDepth);

		auto spColorCloud = boost::make_shared<ColorPointCloud>();
		for (const auto& pt : cloud)
			spColorCloud->push_back(colorPoint(closeColor, farColor, closePt.z, farPt.z, pt, pt.z));

		return spColorCloud;
	}


	static PointXYZRGB colorPoint(const Eigen::Vector3f& minColor, const Eigen::Vector3f& maxColor, float minValue, float maxValue, PointXYZ point, float pointValue)
	{
		auto scale = maxValue - minValue;
		auto normalizedVal = scale == 0 ? 0 : (pointValue - minValue) / scale;
		normalizedVal = std::fmin(1.0f, std::fmax(0.0f, normalizedVal));

		auto rDiff = maxColor[0] - minColor[0];
		auto gDiff = maxColor[1] - minColor[1];
		auto bDiff = maxColor[2] - minColor[2];

		PointXYZRGB retPoint
		(
			static_cast<uint8_t>((minColor[0] + rDiff * normalizedVal) * 255),
			static_cast<uint8_t>((minColor[1] + gDiff * normalizedVal) * 255),
			static_cast<uint8_t>((minColor[2] + bDiff * normalizedVal) * 255)
		);

		retPoint.x = point.x;
		retPoint.y = point.y;
		retPoint.z = point.z;
		return retPoint;
	}


	template <typename TPoint, typename TComp>
	static TPoint samplePoint(const pcl::PointCloud<TPoint>& cloud, double samplePlace, TComp compare)
	{
		std::vector<TPoint> orderedPoints(cloud.begin(), cloud.end());
		std::sort(orderedPoints.begin(), orderedPoints.end(), compare);
		return orderedPoints[static_cast<size_t>(samplePlace * orderedPoints.size())];
	}

}
