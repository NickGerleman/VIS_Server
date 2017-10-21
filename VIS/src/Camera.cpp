#include "pch.h"
#include "Camera.h"

using namespace openni;

namespace vis
{

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


	void OpenNIFrame::clipFrame(const ClipCube& clipCube)
	{
		for (int y = 0; y < this->height(); y++)
		{
			for (int x = 0; x < this->width(); x++)
			{
				auto worldPt = worldPtFromPixel(x, y);
				if (!(worldPt && clipCube.intersects(worldPt.get())))
					m_depthPixels[(y * height()) + width()] = 0;
			}
		}
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


	std::unique_ptr<DepthFrame> OpenNICamera::captureFrame()
	{
		VideoFrameRef niFrame;
		if (m_spDepthStream->readFrame(&niFrame) != Status::STATUS_OK)
			throw std::runtime_error(OpenNI::getExtendedError());

		return std::make_unique<OpenNIFrame>(niFrame, m_spDepthStream, m_pCameraParams);
	}


	/*static*/ std::unique_ptr<OpenNICamera> StructureSensorAccess::acquireCamera()
	{
		static std::recursive_mutex s_camMutex;
		std::unique_lock<std::recursive_mutex> threadLock(s_camMutex);

		// Initialize if this is the first call
		static NUI_FUSION_CAMERA_PARAMETERS* s_pCameraParams = nullptr;
		if (s_pCameraParams == nullptr)
		{
			if (OpenNI::initialize() != STATUS_OK)
				throw std::runtime_error(OpenNI::getExtendedError());
		}

		// Set up device, configuring stream
		auto spDevice = std::make_unique<openni::Device>();
		if (spDevice->open(ANY_DEVICE) != STATUS_OK)
			throw std::runtime_error(OpenNI::getExtendedError());

		auto spDepthStream = std::make_shared<VideoStream>();
		if (spDepthStream->create(*spDevice, SENSOR_DEPTH) != STATUS_OK)
			throw std::runtime_error(OpenNI::getExtendedError());

		VideoMode vgaMode;
		vgaMode.setFps(30);
		vgaMode.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
		vgaMode.setResolution(640, 480);
		
		spDepthStream->setVideoMode(vgaMode);
		spDepthStream->setMirroringEnabled(false);

		if (spDepthStream->start() != Status::STATUS_OK)
			throw std::runtime_error(OpenNI::getExtendedError());

		// Camera instrinsics taken from https://forums.structure.io/t/intrinsics-for-ios-devices-and-the-structure-sensor-depth-point-cloud-info/4640
		if (s_pCameraParams == nullptr)
		{
			static NUI_FUSION_CAMERA_PARAMETERS params;
			params.focalLengthX = 0.909375f;
			params.focalLengthY = 1.2125f;
			params.principalPointX = 0.5;
			params.principalPointY = 0.5;
			s_pCameraParams = &params;
		}

		return std::unique_ptr<OpenNICamera>(new OpenNICamera(
				std::move(spDevice),
				spDepthStream,
				std::move(threadLock),
				s_pCameraParams
		));
	}

}
