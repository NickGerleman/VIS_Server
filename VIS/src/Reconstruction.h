#pragma once

#include "Camera.h"
#include "Geometry.h"

namespace vis
{

	/// Performs 3D reconstruction on a series of depth images. Reconstruction is GPU
	/// accelerated and operates on its own thread.
	class Reconstructor
	{
	public:

		/// Build the reconstructor using the given clip volume
		/// @param spClipVolume the clip volume to use
		Reconstructor(const std::shared_ptr<ClipVolume>& spclipVolume);
		~Reconstructor();

		/// Submit a frame to be eventually processed
		/// @param spFrame the unclipped depth frame
		void submitFrame(const std::shared_ptr<DepthFrame>& spFrame);

		/// Notify the Reconstructor that all frames have been submitted and block until
		/// reconstruction is complete
		std::shared_ptr<Mesh> waitForResult();
	
	private:

		/// Block until a frame is submitted or the client calls waitForResult
		std::shared_ptr<vis::DepthFrame> blockForFrame();


		/// Main loop of Reconstructor. Repeatedly block for a frame then process it.
		void loopUntilDone();

		/// Generate an image of the current reconstruction volume
		/// @param width width in pixels of image
		/// @param height height in pixels of image
		/// @param worldToCam transform to place the camera in world space
		/// @param pCameraParams camera intrinsics to embed in the fusion frame
		FusionFramePtr generateImage(int width, int height, const Matrix4& worldToCam, const NUI_FUSION_CAMERA_PARAMETERS* pCameraParams);
		
		/// Convert a fusion mesh into a PCL mesh in the coordinate space of the clip volume
		/// @param pFusionMesh the KinectFusion reconstructed mesh
		/// @param clipVolume the clip volume to use
		static std::shared_ptr<Mesh> fusionToPclMesh(INuiFusionMesh* pFusionMesh, const ClipVolume& clipVolume);

		std::atomic<bool> m_finished;
		std::condition_variable m_queueReadyOrFinish;
		std::mutex m_queueMutex;
		std::thread m_reconThread;

		std::shared_ptr<ClipVolume> m_spClipVolume;
		std::queue<std::shared_ptr<DepthFrame>> m_framesToProcess;
		INuiFusionReconstruction* m_pFusionRecon;
	};

}
