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
		if (m_pserialPort->isConnected())
		{
			if (!m_pserialPort->writeData("r", 1))
			{
				throw std::runtime_error("Failed to send rotation command through Serial Port");
			}
		}
	}

	bool ServoPlatformControls::isRotating()
	{
		if (m_pserialPort->isConnected())
		{
			char incomingData[256] = "";
			m_pserialPort->readData(incomingData, 255);
			return (strcmp(incomingData, "done") != 0);
		}
		return false;
	}
}
