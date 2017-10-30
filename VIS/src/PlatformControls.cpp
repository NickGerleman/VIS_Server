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

	void ServoPlatformControls::startRotation()
	{
		if (m_SP->isConnected())
		{
			m_SP->writeData("r", 1);
		}
	}

	bool ServoPlatformControls::isRotating()
	{
		if (m_SP->isConnected()) 
		{
			char incomingData[256] = "";
			m_SP->readData(incomingData, 255);
			if (strcmp(incomingData, "done") == 0)
			{
				return false;
			}
			else
			{
				return true;
			}
		}
		return false;
	}

	bool ServoPlatformControls::isConnected()
	{
		return m_SP->isConnected();
	}
}
