#include "pch.h"
#include "Reconstruction.h"
#include "Visualization.h"

namespace vis
{
	// TODO experiment with these
	const static USHORT MAX_INTEGRATION_WEIGHT = NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT;
	const static USHORT MAX_ALIGN_ITERS = NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT;

	const static int MAX_VOXELS = 640 * 640 * 160;
	const static float MAX_VOXELS_PER_METER = 512;
	const static UINT VOXELS_PER_DIMENSION = 512;

	const static int IMAGE_WIDTH = 640;
	const static int IMAGE_HEIGHT = 480;

	const static Matrix4 IDENTITY_MATRIX
	{
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
	};

	Reconstructor::Reconstructor(const std::shared_ptr<ClipVolume>& spclipVolume)
		: m_finished(false)
		, m_spClipVolume(spclipVolume)
	{
		NUI_FUSION_RECONSTRUCTION_PARAMETERS reconParams;
		reconParams.voxelsPerMeter = MAX_VOXELS_PER_METER;
		reconParams.voxelCountX = VOXELS_PER_DIMENSION;
		reconParams.voxelCountY = VOXELS_PER_DIMENSION;
		reconParams.voxelCountZ = VOXELS_PER_DIMENSION;

		// Use whatever GPU on the device is reccomended
		HRESULT hr = NuiFusionCreateReconstruction(
			&reconParams,
			NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP,
			-1,
			&IDENTITY_MATRIX,
			&m_pFusionRecon
		);
		if (!SUCCEEDED(hr))
			throw std::runtime_error("KinectFusion Failed to Start");

		// Begin thread
		m_reconThread = std::thread(std::bind(&Reconstructor::loopUntilDone, this));
	}


	Reconstructor::~Reconstructor()
	{
		m_pFusionRecon->Release();
	}


	void Reconstructor::submitFrame(const std::shared_ptr<DepthFrame>& spFrame)
	{
		if (!m_reconThread.joinable())
			throw std::runtime_error("Cannot submit frame to finished Reconstructor");

		std::unique_lock<std::mutex> queueLock(m_queueMutex);
		m_framesToProcess.push(spFrame);
		m_queueReadyOrFinish.notify_one();
	}


	std::shared_ptr<Mesh> Reconstructor::waitForResult()
	{
		m_finished = true;
		m_queueReadyOrFinish.notify_one();
		m_reconThread.join();

		INuiFusionMesh* pFusionMesh;
		HRESULT hr = m_pFusionRecon->CalculateMesh(1/*voxelStep*/, &pFusionMesh);
		if (!SUCCEEDED(hr))
			throw std::runtime_error("Cannot calculate mesh for reconstruction");

		auto spVisMesh = fusionToPclMesh(pFusionMesh, *m_spClipVolume);
		pFusionMesh->Release();

		return spVisMesh;
	}


	std::shared_ptr<vis::DepthFrame> Reconstructor::blockForFrame()
	{
		std::unique_lock<std::mutex> queueLock(m_queueMutex);

		if (m_framesToProcess.empty() && m_finished)
			return nullptr;

		if (m_framesToProcess.empty())
			m_queueReadyOrFinish.wait(queueLock);

		if (m_framesToProcess.empty() && m_finished)
			return nullptr;

		auto spNextFrame = m_framesToProcess.front();
		m_framesToProcess.pop();
		return spNextFrame;
	}


	void Reconstructor::loopUntilDone()
	{
		Matrix4 worldToCam = IDENTITY_MATRIX;

		while (auto spFrame = blockForFrame())
		{
			// Grab the latest frame to integrate and clip it down to size
			auto spClippedFrame = spFrame->clipFrame(m_spClipVolume);
			auto spFusionFrame = spClippedFrame->convertToFusionFrame();

			// Align frame to current volume and integrate it
			FLOAT alignmentEnergy;
			HRESULT hr = m_pFusionRecon->ProcessFrame(
				spFusionFrame.get(),
				MAX_ALIGN_ITERS,
				MAX_INTEGRATION_WEIGHT,
				&alignmentEnergy, &worldToCam
			);

			// TODO actually get this to the right thread
			if (!SUCCEEDED(hr))
				throw std::runtime_error("Could not process frame during reconstruction");

			m_pFusionRecon->GetCurrentWorldToCameraTransform(&worldToCam);
			ProgressVisualizer::get()->notifyReconstruction(generateImage(IMAGE_WIDTH, IMAGE_HEIGHT, worldToCam, spFusionFrame->pCameraParameters));
		}
	}


	FusionFramePtr Reconstructor::generateImage(int width, int height, const Matrix4& worldToCam, const NUI_FUSION_CAMERA_PARAMETERS* pCameraParams)
	{
		NUI_FUSION_IMAGE_FRAME* pCloudFrame;
		NUI_FUSION_IMAGE_FRAME* pColorFrame;
		NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, width, height, pCameraParams, &pCloudFrame);
		NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, width, height, pCameraParams, &pColorFrame);

		if (!SUCCEEDED(m_pFusionRecon->CalculatePointCloud(pCloudFrame, &worldToCam)))
			throw std::runtime_error("Could not calculate fusion point cloud");

		if (!SUCCEEDED(NuiFusionShadePointCloud(pCloudFrame, &worldToCam, nullptr, pColorFrame, nullptr)))
			throw std::runtime_error("Could not shade fusion point cloud");

		NuiFusionReleaseImageFrame(pCloudFrame);
		return FusionFramePtr(pColorFrame);
	}


	/*static*/ std::shared_ptr<Mesh> Reconstructor::fusionToPclMesh(INuiFusionMesh* pFusionMesh, const ClipVolume& clipVolume)
	{
		const Vector3* pVerts;
		pFusionMesh->GetVertices(&pVerts);
		const int* pTriangles;
		pFusionMesh->GetTriangleIndices(&pTriangles);

		auto spPclMesh = std::make_shared<Mesh>();

		for (UINT i = 0; i < pFusionMesh->VertexCount(); i++)
		{
			// KinectFusion exports meshes into a wacky coordinate space, correct that here
			pcl::PointXYZ worldSpacePt(pVerts[i].x, -pVerts[i].y, -pVerts[i].z);
			auto clipSpacePt = clipVolume.relativeCoordinate(worldSpacePt);
			spPclMesh->addVertex(clipSpacePt);
		}

		for (UINT i = 0; i < pFusionMesh->TriangleVertexIndexCount(); i += 3)
		{
			spPclMesh->addFace(
				pcl::geometry::VertexIndex(pTriangles[i]),
				pcl::geometry::VertexIndex(pTriangles[i + 1]),
				pcl::geometry::VertexIndex(pTriangles[i + 2])
			);
		}

		return spPclMesh;
	}


}
