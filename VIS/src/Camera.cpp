#include "pch.h"
#include "Camera.h"
#include "Visualization.h"

using namespace openni;


namespace vis
{

	static void assertNI(openni::Status status)
	{
		if (status != STATUS_OK)
			throw std::runtime_error(OpenNI::getExtendedError());
	}

	ClipCube::ClipCube(const Eigen::Matrix4f& referenceTransform)
		: m_inverseTransform(referenceTransform.inverse()) {}


	bool ClipCube::intersects(const pcl::PointXYZ& point) const
	{
		auto relativeCoords = relativeCoordinate(point);
		return relativeCoords.x >= -0.5
			&& relativeCoords.x <= 0.5
			&& relativeCoords.y >= -0.5
			&& relativeCoords.y <= 0.5
			&& relativeCoords.z >= -0.5
			&& relativeCoords.z <= 0.5;
	}


	pcl::PointXYZ ClipCube::relativeCoordinate(const pcl::PointXYZ& point) const
	{
		Eigen::Vector4f transformed = m_inverseTransform * Eigen::Vector4f(point.x, point.y, point.z, 1.0);
		return { transformed.x(), transformed.y(), transformed.z() };
	}


	int DepthFrame::width()
	{
		return m_width;
	}


	int DepthFrame::height()
	{
		return m_height;
	}


	OpenNIFrame::OpenNIFrame(const VideoFrameRef& depthFrame, std::shared_ptr<VideoStream> spDepthStream, const NUI_FUSION_CAMERA_PARAMETERS* pCameraParams)
		: DepthFrame(depthFrame.getWidth(), depthFrame.getHeight())
		, m_spDepthStream(spDepthStream)
		, m_pCameraParams(pCameraParams)
	{
		const uint16_t* depthData = static_cast<const uint16_t*>(depthFrame.getData());
		size_t numPixels = this->width() * this->height();
		m_depthPixels.insert(m_depthPixels.begin(), depthData, depthData + numPixels);
	}


	std::shared_ptr<DepthFrame> OpenNIFrame::clone()
	{
		return std::make_shared<OpenNIFrame>(*this);
	}


	boost::shared_ptr<PointCloud> OpenNIFrame::generatePointCloud()
	{
		auto spCloud = boost::make_shared<PointCloud>();
		
		for (int y = 0; y < this->height(); y++)
		{
			for (int x = 0; x < this->width(); x++)
			{
				auto worldPt = worldPtFromPixel(x, y);
				if (worldPt)
					spCloud->push_back(worldPt.get());
			}
		}

		return spCloud;
	}


	std::shared_ptr<DepthFrame> OpenNIFrame::clipFrame(const ClipCube& clipCube)
	{
		auto spClippedFrame = std::make_shared<OpenNIFrame>(*this);

		for (int y = 0; y < this->height(); y++)
		{
			for (int x = 0; x < this->width(); x++)
			{
				auto worldPt = worldPtFromPixel(x, y);
				if (!(worldPt && clipCube.intersects(worldPt.get())))
					(*spClippedFrame).m_depthPixels[(y * height()) + width()] = 0;
			}
		}

		return spClippedFrame;
	}


	std::unique_ptr<NUI_FUSION_IMAGE_FRAME, DepthFrame::FusionFrameDeleter> OpenNIFrame::convertToFusionFrame()
	{
		NUI_FUSION_IMAGE_FRAME* pFrame;
		
		HRESULT hr;
		
		hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, width(), height(), m_pCameraParams, &pFrame);
		if (!SUCCEEDED(hr))
			throw std::runtime_error("NuiFusionCreateImageFrame failed");

		// Convert to Kinect depth pixel format
		std::vector<NUI_DEPTH_IMAGE_PIXEL> kinectDepthPixels(m_depthPixels.size());
		for (uint16_t depth : m_depthPixels)
			kinectDepthPixels.push_back({0, depth});

		hr = NuiFusionDepthToDepthFloatFrame(
			kinectDepthPixels.data(),
			width(),
			height(),
			pFrame,
			NUI_FUSION_DEFAULT_MINIMUM_DEPTH,
			NUI_FUSION_DEFAULT_MAXIMUM_DEPTH,
			false /*mirrorDepth*/);

		if (!SUCCEEDED(hr))
			throw std::runtime_error("NuiFusionDepthToDepthFloatFrame failed");

		auto freeFrame = [](NUI_FUSION_IMAGE_FRAME* pFrame)
		{
			if (!SUCCEEDED(NuiFusionReleaseImageFrame(pFrame)))
				throw std::runtime_error("Failed to free image data");
		};

		return std::unique_ptr<NUI_FUSION_IMAGE_FRAME, FusionFrameDeleter>(pFrame, freeFrame);
	}


	boost::optional<pcl::PointXYZ> OpenNIFrame::worldPtFromPixel(int x, int y)
	{
		int idx = (y * this->width()) + x;
		uint16_t depth = m_depthPixels[idx];

		if (depth == 0)
			return boost::none;

		pcl::PointXYZ worldPt;
		if (CoordinateConverter::convertDepthToWorld(*m_spDepthStream, x, y, depth, &worldPt.x, &worldPt.y, &worldPt.z) != Status::STATUS_OK)
			throw std::runtime_error(OpenNI::getExtendedError());

		// Convert to meters in right-hand coordinate system
		worldPt.x /= 1000;
		worldPt.y /= 1000;
		worldPt.z = -(worldPt.z / 1000);

		return worldPt;
	}


	OpenNICamera::~OpenNICamera()
	{
		m_spDepthStream->stop();
		m_spDevice->close();
	}


	std::shared_ptr<DepthFrame> OpenNICamera::captureFrame()
	{
		VideoFrameRef niFrame;
		if (m_spDepthStream->readFrame(&niFrame) != Status::STATUS_OK)
			throw std::runtime_error(OpenNI::getExtendedError());

		auto spFrame = std::make_shared<OpenNIFrame>(niFrame, m_spDepthStream, m_pCameraParams);
		ProgressVisualizer::get()->notifyDepthFrame(spFrame);

		return spFrame;
	}


	/*static*/ std::unique_ptr<OpenNICamera> StructureSensor::acquireCamera()
	{
		auto spDevice = std::make_unique<openni::Device>();
		assertNI(spDevice->open(ANY_DEVICE));
		auto spDepthStream = std::make_shared<VideoStream>();
		
		assertNI(spDepthStream->create(*spDevice, SENSOR_DEPTH));
		spDepthStream->setMirroringEnabled(false);

		VideoMode vgaMode;
		vgaMode.setFps(30);
		vgaMode.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
		vgaMode.setResolution(640, 480);
		spDepthStream->setVideoMode(vgaMode);

		assertNI(spDepthStream->start());

		static std::recursive_mutex s_camMutex;
		return std::unique_ptr<OpenNICamera>(new OpenNICamera(
				std::move(spDevice),
				spDepthStream,
				std::unique_lock<std::recursive_mutex>(s_camMutex),
				cameraIntrinsics()
		));
	}


	/*static*/ std::shared_ptr<MockCamera> MockCamera::make(const std::string& filepath, const NUI_FUSION_CAMERA_PARAMETERS* pCameraParams)
	{
		auto spDevice = std::make_unique<openni::Device>();
		assertNI(spDevice->open(filepath.c_str()));

		auto spDepthStream = std::make_shared<openni::VideoStream>();
		assertNI(spDepthStream->create(*spDevice, SENSOR_DEPTH));
		assertNI(spDepthStream->start());

		// We're not having multiple threads access the same physical device,
		// so we don't actually need the handle to have any sort of real lock
		static std::recursive_mutex s_unusedMutex;

		auto* pPlaybackControls = spDevice->getPlaybackControl();
		auto spMockCamera = std::shared_ptr<MockCamera>(new MockCamera(
			std::move(spDevice),
			spDepthStream,
			std::unique_lock<std::recursive_mutex>(s_unusedMutex),
			pCameraParams,
			pPlaybackControls));

		return spMockCamera;
	}


	void MockCamera::startFullLoop()
	{
		m_static = false;
		m_curFrame = 0;
	}


	bool MockCamera::isStatic()
	{
		return m_static;
	}


	std::shared_ptr<DepthFrame> MockCamera::captureFrame()
	{
		auto spFrame = OpenNICamera::captureFrame();

		if (m_static)
		{
			m_pControls->seek(*m_spDepthStream, 0);
		}
		else
		{
			int lenStream = m_pControls->getNumberOfFrames(*m_spDepthStream);
			m_curFrame = (m_curFrame + 1) % lenStream;
			if (m_curFrame == 0)
			{
				m_pControls->seek(*m_spDepthStream, 0);
				m_static = true;
			}
		}

		return spFrame;
	}


	const NUI_FUSION_CAMERA_PARAMETERS* StructureSensor::cameraIntrinsics()
	{
		static NUI_FUSION_CAMERA_PARAMETERS* pParams = nullptr;

		if (pParams == nullptr)
		{
			// Taken from https://forums.structure.io/t/intrinsics-for-ios-devices-and-the-structure-sensor-depth-point-cloud-info/4640
			pParams = new NUI_FUSION_CAMERA_PARAMETERS;
			pParams->focalLengthX = 0.909375f;
			pParams->focalLengthY = 1.2125f;
			pParams->principalPointX = 0.5;
			pParams->principalPointY = 0.5;
		}

		return pParams;
	}

}
