#include "pch.h"
#include "Visualization.h"

using namespace std::chrono;
using namespace std::chrono_literals;

namespace vis
{

	static const milliseconds PUSH_INTERVAL = 4ms;


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
			pcl::visualization::PCLVisualizer viz;
			viz.setWindowName("Visualizer");
			viz.setShowFPS(false);

			while (true)
			{
				viz.spinOnce();

				{
					std::lock_guard<std::mutex> lock(m_lastUpdateLock);
					updateDisplay(viz, m_visThreadRecord);
				}

				std::this_thread::sleep_for(PUSH_INTERVAL);
			}
		});

		m_visualizationThread.detach();
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


	void ProgressVisualizer::pushIfNeeded()
	{
		auto curTime = high_resolution_clock::now();
		if (curTime - m_lastPushTime > PUSH_INTERVAL)
		{
			std::lock_guard<std::mutex> lock(m_lastUpdateLock);
			m_visThreadRecord = ProgressRecord
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


	/*static*/ void ProgressVisualizer::updateDisplay(pcl::visualization::PCLVisualizer& visualizer, const ProgressRecord& record)
	{
		if (record.spDepthFrame && !visualizer.updatePointCloud(record.spDepthFrame->generatePointCloud()))
			visualizer.addPointCloud(record.spDepthFrame->generatePointCloud());
	}


	std::shared_ptr<pcl::visualization::PCLVisualizer> createAlignmentVisualizer(const boost::shared_ptr<PointCloud>& spReference, const boost::shared_ptr<PointCloud>& spTarget)
	{
		auto spVisualizer = std::make_shared<pcl::visualization::PCLVisualizer>("Alignment");
		spVisualizer->setBackgroundColor(0, 0, 0);

		spVisualizer->addPointCloud<pcl::PointXYZ>(spReference, "ref");
		spVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ref");
		spVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "ref");

		spVisualizer->addPointCloud<pcl::PointXYZ>(spTarget, "target");
		spVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target");
		spVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "target");

		spVisualizer->initCameraParameters();
		return spVisualizer;
	}


	std::shared_ptr<pcl::visualization::PCLVisualizer> createErrorVisualizer(const ErrorPointCloud& cloud)
	{
		// Use the first statistical outlier as the beginning of the gradient
		std::vector<float> orderedErrors;
		for (const auto& point : cloud)
			orderedErrors.push_back(point.intensity);
		std::sort(orderedErrors.begin(), orderedErrors.end());

		auto quartile1 = orderedErrors[orderedErrors.size() / 4];
		auto quartile3 = orderedErrors[(orderedErrors.size() / 4) * 3];
		auto iqr = quartile3 - quartile1;
		
		auto gradientMin = quartile3 + (1.5f * iqr);
		auto gradientMax = *orderedErrors.rbegin();
		auto scale = (gradientMin == 0.0) ? gradientMax : (gradientMax / gradientMin);

		// Add to colored point cloud
		auto spColorCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
		for (const auto& point : cloud)
		{
			auto normalizedError = (point.intensity - gradientMin) / scale;
			normalizedError = std::fmin(1.0f, std::fmax(0.0f, normalizedError));
			auto greenBlueComponent = static_cast<uint8_t>((1.0 - normalizedError) * 255);

			pcl::PointXYZRGB colorPoint(255, greenBlueComponent, greenBlueComponent);
			colorPoint.x = point.x;
			colorPoint.y = point.y;
			colorPoint.z = point.z;
			spColorCloud->push_back(colorPoint);
		}

		// Visualize
		auto spVisualizer = std::make_shared<pcl::visualization::PCLVisualizer>("Error");
		spVisualizer->setBackgroundColor(0, 0, 0);
		spVisualizer->addPointCloud<pcl::PointXYZRGB>(spColorCloud);
		spVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);

		return spVisualizer;
	}

}
