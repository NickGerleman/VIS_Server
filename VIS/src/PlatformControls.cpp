#include "pch.h"
#include "PlatformControls.h"

namespace vis
{
	void MockPlatformControls::startRotation()
	{
		m_spMockCamera->startFullLoop();
	}


	bool MockPlatformControls::isRotating()
	{
		return !m_spMockCamera->isStatic();
	}
}
