#include "pch.h"
#include "Visualization.h"

using namespace std::chrono;
using namespace std::chrono_literals;

namespace vis
{
	static const int NUM_ROWS = 2;
	static const int NUM_COLS = 3;

	static const milliseconds PUSH_INTERVAL = 10ms;

	static const char* DEPTH_FRAME_TAG     = "DEPTH_FRAME_TAG";
	static const char* CLIP_FRAME_TAG      = "CLIP_FRAME_TAG";
	static const char* RECONSTRUCTION_TAG  = "RECONSTRUCTION_TAG";
	static const char* SURFACE_CLOUD_TAG   = "SURFACE_CLOUD_TAG";
	static const char* ALIGNMENT_TAG       = "ALIGNMENT_TAG";
	static const char* ERROR_CLOUD_TAG     = "ERROR_CLOUD_TAG";

	static const char* DEPTH_FRAME_TEXT    = "World Depth";
	static const char* CLIP_FRAME_TEXT     = "Clipped Depth";
	static const char* RECONSTRUCTION_TEXT = "3D Reconstruction";
	static const char* SURFACE_CLOUD_TEXT  = "Surface Reconstruction";
	static const char* ALIGNMENT_TEXT      = "Alignment";
	static const char* ERROR_CLOUD_TEXT    = "Error Levels";

	ProgressRecord::ProgressRecord(
		const std::shared_ptr<DepthFrame>& spDepthFrame,
		const boost::shared_ptr<PointCloud>& spSurfaceCloud,
		const std::shared_ptr<DepthFrame>& spClipFrame,
		const boost::shared_ptr<PointCloud>& spAlignment,
		const boost::shared_ptr<ErrorPointCloud>& spErrorCloud
	)
	{
		if (spDepthFrame)
			this->spDepthFrame = spDepthFrame->clone();

		if (spSurfaceCloud)
			this->spSurfaceCloud = boost::make_shared<PointCloud>(*spSurfaceCloud);

		if (spClipFrame)
			this->spClipFrame = spClipFrame->clone();

		if (spAlignment)
			this->spAlignment = boost::make_shared<PointCloud>(*spAlignment);

		if (spErrorCloud)
			this->spErrorCloud = boost::make_shared<ErrorPointCloud>(*spErrorCloud);
	}


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

		m_visualizationThread = std::thread([this]()
		{
			m_spVisualizer = std::make_unique<pcl::visualization::PCLVisualizer>();
			initViewports();

			m_spVisualizer->setWindowName("Vizualizer");
			m_spVisualizer->setBackgroundColor(0.15, 0.15, 0.15);
			m_spVisualizer->setShowFPS(false);

			while (true)
			{
				{
					std::lock_guard<std::mutex> lock(m_lastUpdateLock);
					updateDisplay();
				}

				m_spVisualizer->spinOnce();
				std::this_thread::sleep_for(PUSH_INTERVAL / 2);
			}
		});

		m_visualizationThread.detach();
	}


	void ProgressVisualizer::notifyNewScan()
	{
		m_spLastDepthFrame = nullptr;
		m_spLastSurfaceCloud = nullptr;
		m_spLastClipFrame = nullptr;
		m_spLastAlignment = nullptr;
		m_spLastErrorCloud = nullptr;
	}


	void ProgressVisualizer::notifyDepthFrame(const std::shared_ptr<DepthFrame>& spFrame)
	{
		m_spLastDepthFrame = spFrame;
		pushIfNeeded();
	}


	void ProgressVisualizer::notifySurfaceCloud(const boost::shared_ptr<PointCloud>& spCloud)
	{
		m_spLastSurfaceCloud = spCloud;
		pushIfNeeded();
	}


	void ProgressVisualizer::notifyClipFrame(const std::shared_ptr<DepthFrame>& spFrame)
	{
		m_spLastClipFrame = spFrame;
		pushIfNeeded();
	}


	void ProgressVisualizer::notifyAlignment(const boost::shared_ptr<PointCloud>& spCloud)
	{
		m_spLastAlignment = spCloud;
		pushIfNeeded();
	}


	void ProgressVisualizer::notifyErrorCloud(const boost::shared_ptr<ErrorPointCloud>& spCloud)
	{
		m_spLastErrorCloud = spCloud;
		pushIfNeeded();
	}


	static void initViewport(pcl::visualization::PCLVisualizer& vizualizer, int row, int col, const char* tag, const char* text, int& viewportId)
	{
		double minX = col * (1.0 / NUM_COLS);
		double maxX = minX + (1.0 / NUM_COLS);

		double maxY = 1 - (row * 1.0 / NUM_ROWS);
		double minY = maxY - (1.0 / NUM_ROWS);

		vizualizer.createViewPort(minX, minY, maxX, maxY, viewportId);
		vizualizer.addText(text, 0, 0, text, viewportId);
		vizualizer.addPointCloud(boost::make_shared<ColorPointCloud>(), tag, viewportId);
		vizualizer.createViewPortCamera(viewportId);
	}


	void ProgressVisualizer::initViewports()
	{
		initViewport(*m_spVisualizer, 0, 0, DEPTH_FRAME_TAG,    DEPTH_FRAME_TEXT,    m_depthFrameViewport);
		initViewport(*m_spVisualizer, 0, 1, CLIP_FRAME_TAG,     CLIP_FRAME_TEXT,     m_clipFrameViewport);
		initViewport(*m_spVisualizer, 0, 2, RECONSTRUCTION_TAG, RECONSTRUCTION_TEXT, m_reconstructionViewport);
		initViewport(*m_spVisualizer, 1, 0, SURFACE_CLOUD_TAG,  SURFACE_CLOUD_TEXT, m_surfaceCloudViewport);
		initViewport(*m_spVisualizer, 1, 1, ALIGNMENT_TAG,      ALIGNMENT_TEXT,      m_alignmentViewport);
		initViewport(*m_spVisualizer, 1, 2, ERROR_CLOUD_TAG,    ERROR_CLOUD_TEXT,    m_errorViewport);
	}


	void ProgressVisualizer::pushIfNeeded()
	{
		auto curTime = high_resolution_clock::now();
		if (curTime - m_lastPushTime > PUSH_INTERVAL)
		{
			std::lock_guard<std::mutex> lock(m_lastUpdateLock);
			m_vizThreadRecord = ProgressRecord
			(
				m_spLastDepthFrame,
				m_spLastSurfaceCloud,
				m_spLastClipFrame,
				m_spLastAlignment,
				m_spLastErrorCloud
			);
			m_lastPushTime = curTime;
		}

	}


	void ProgressVisualizer::updateDisplay()
	{
		updateText();
		updateDepthFrameView();
		updateClipFrameView();
		updateSurfaceCloudView();
		updateAlignmentView();
		updateErrorView();
	}


	void ProgressVisualizer::updateDepthFrameView()
	{
		if (!m_vizThreadRecord.spDepthFrame)
		{
			m_spVisualizer->updatePointCloud(boost::make_shared<ColorPointCloud>(), DEPTH_FRAME_TAG);
			return;
		}

		auto spColorDepthCloud = colorDepthCloud(*m_vizThreadRecord.spDepthFrame->generatePointCloud());
		m_spVisualizer->updatePointCloud(spColorDepthCloud, DEPTH_FRAME_TAG);
		fitCameraToCloud(m_depthFrameViewport, *spColorDepthCloud);
	}


	void ProgressVisualizer::updateClipFrameView()
	{
		if (!m_vizThreadRecord.spClipFrame)
		{
			m_spVisualizer->updatePointCloud(boost::make_shared<ColorPointCloud>(), CLIP_FRAME_TAG);
			return;
		}

		auto spColorDepthCloud = colorDepthCloud(*m_vizThreadRecord.spClipFrame->generatePointCloud());
		m_spVisualizer->updatePointCloud(spColorDepthCloud, CLIP_FRAME_TAG);
		fitCameraToCloud(m_clipFrameViewport, *spColorDepthCloud);
	}


	void ProgressVisualizer::updateSurfaceCloudView()
	{
		if (!m_vizThreadRecord.spSurfaceCloud)
		{
			m_spVisualizer->updatePointCloud(boost::make_shared<ColorPointCloud>(), SURFACE_CLOUD_TAG);
			return;
		}

		auto spColorDepthCloud = colorDepthCloud(*m_vizThreadRecord.spSurfaceCloud);
		m_spVisualizer->updatePointCloud(spColorDepthCloud, SURFACE_CLOUD_TAG);
		fitCameraToCloud(m_surfaceCloudViewport, *spColorDepthCloud);
	}


	void ProgressVisualizer::updateAlignmentView()
	{
		if (!m_vizThreadRecord.spAlignment)
		{
			m_spVisualizer->updatePointCloud(boost::make_shared<ColorPointCloud>(), ALIGNMENT_TAG);
			return;
		}

		auto spComposite = colorDepthCloud(*m_vizThreadRecord.spSurfaceCloud);
		auto spCapture = colorDepthCloud(*m_vizThreadRecord.spAlignment, {0.8f, 0.2f, 0.2f}, {0.2f, 0.2f, 0.2f});
		spComposite->insert(spComposite->end(), spCapture->begin(), spCapture->end());

		m_spVisualizer->updatePointCloud(spComposite, ALIGNMENT_TAG);
		fitCameraToCloud(m_alignmentViewport, *spComposite);
	}


	void ProgressVisualizer::updateErrorView()
	{
		if (!m_vizThreadRecord.spErrorCloud)
		{
			m_spVisualizer->updatePointCloud(boost::make_shared<ColorPointCloud>(), ERROR_CLOUD_TAG);
			return;
		}
		const auto& spErrorCloud = m_vizThreadRecord.spErrorCloud;

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
			pcl::PointXYZ xyzPoint(pt.x, pt.y, pt.z);
			spColorCloud->push_back(colorPoint({ 0.8f, 0.8f, 0.8f }, { 0.8f, 0.0f, 0.0f }, gradientMin, gradientMax, xyzPoint, pt.intensity));
		}
		m_spVisualizer->updatePointCloud(spColorCloud, ERROR_CLOUD_TAG);
		fitCameraToCloud(m_errorViewport, *spColorCloud);
	}


	void ProgressVisualizer::updateText()
	{
		const double PADDING_X = 0.06;
		const double PADDING_Y = 0.04;

		std::vector<pcl::visualization::Camera> viewportCameras;
		m_spVisualizer->getCameras(viewportCameras);
		double viewportWidth = viewportCameras[0].window_size[0] / NUM_COLS;
		double viewportHeight = viewportCameras[0].window_size[0] / NUM_ROWS;

		int offsetX = static_cast<int>(PADDING_X * viewportWidth);
		int offsetY = static_cast<int>(PADDING_Y * viewportHeight);

		m_spVisualizer->updateText(SURFACE_CLOUD_TEXT,  offsetX, offsetY, SURFACE_CLOUD_TEXT);
		m_spVisualizer->updateText(DEPTH_FRAME_TEXT,    offsetX, offsetY, DEPTH_FRAME_TEXT);
		m_spVisualizer->updateText(CLIP_FRAME_TEXT,     offsetX, offsetY, CLIP_FRAME_TEXT);
		m_spVisualizer->updateText(RECONSTRUCTION_TEXT, offsetX, offsetY, RECONSTRUCTION_TEXT);
		m_spVisualizer->updateText(ALIGNMENT_TEXT,      offsetX, offsetY, ALIGNMENT_TEXT);
		m_spVisualizer->updateText(ERROR_CLOUD_TEXT,    offsetX, offsetY, ERROR_CLOUD_TEXT);
	}


	template <typename TCloud>
	void ProgressVisualizer::fitCameraToCloud(int viewport, const TCloud& cloud)
	{
		std::vector<pcl::visualization::Camera> viewportCameras;
		m_spVisualizer->getCameras(viewportCameras);
		auto& ourCamera = viewportCameras[viewport];
		
		// Set up-vector and look-vector
		ourCamera.view[0] = 0.0;
		ourCamera.view[1] = 1.0;
		ourCamera.view[2] = 0.0;

		ourCamera.focal[0] = 0.0;
		ourCamera.focal[1] = 0.0;
		ourCamera.focal[2] = -1.0;

		// Find bounds of current point cloud
		float minX, maxX, minY, maxY, maxZ;
		minX = minY = FLT_MAX;
		maxX = maxY = maxZ = FLT_MIN;
		for (auto& pt : cloud)
		{
			minX = std::fmin(minX, pt.x);
			maxX = std::fmax(maxX, pt.x);
			minY = std::fmin(minY, pt.y);
			maxY = std::fmax(maxY, pt.y);
			maxZ = std::fmax(maxZ, pt.z);
		}
		
		// Center the camera on X and Y
		float dX = maxX - minX;
		float dY = maxY - minY;
		ourCamera.pos[0] = minX + (dX / 2);
		ourCamera.pos[1] = minY + (dY / 2);

		// Use FOV to calculate Z pos (with padding)
		const double PADDING_FACTOR = 1.3;
		auto aspectRatio = ((ourCamera.window_size[0] / NUM_COLS) / (ourCamera.window_size[1] / NUM_ROWS));
		auto fovX = 2.0 * atan(tan(ourCamera.fovy / aspectRatio));
		
		auto thetaX = fovX / 2;
		auto zFitX = (1 / tan(thetaX)) * (dX * PADDING_FACTOR / 2);
		auto thetaY = ourCamera.fovy / 2;
		auto zFitY = (1 / tan(thetaY)) * (dY * PADDING_FACTOR / 2);

		ourCamera.pos[2] = maxZ + (std::fmax(zFitX, zFitY));

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


	static pcl::PointXYZRGB colorPoint(const Eigen::Vector3f& minColor, const Eigen::Vector3f& maxColor, float minValue, float maxValue, pcl::PointXYZ point, float pointValue)
	{
		auto scale = maxValue - minValue;
		auto normalizedVal = scale == 0 ? 0 : (pointValue - minValue) / scale;
		normalizedVal = std::fmin(1.0f, std::fmax(0.0f, normalizedVal));

		auto rDiff = maxColor[0] - minColor[0];
		auto gDiff = maxColor[1] - minColor[1];
		auto bDiff = maxColor[2] - minColor[2];

		pcl::PointXYZRGB retPoint
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
