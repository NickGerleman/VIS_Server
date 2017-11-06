#pragma once

#include "Camera.h"
#include "Geometry.h"

namespace vis
{

	class Reconstructor
	{
	public:
		Reconstructor(const std::shared_ptr<ClipVolume>& spclipVolume);
		~Reconstructor();
		void submitFrame(const std::shared_ptr<DepthFrame>& spFrame);
		std::shared_ptr<Mesh> waitForResult();
	
	private:
		std::shared_ptr<vis::DepthFrame> blockForFrame();
		void loopUntilDone();

		FusionFramePtr generateImage(int width, int height, const Matrix4& worldToCam, const NUI_FUSION_CAMERA_PARAMETERS* pCameraParams);
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
