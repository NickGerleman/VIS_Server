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
		if (m_serialPort.isConnected())
		{
			if (!m_serialPort.writeData("r", 1))
				throw std::runtime_error("Failed to send rotation command through Serial Port");
		}
	}


	bool ServoPlatformControls::isRotating()
	{
		if (m_serialPort.isConnected())
		{
			char incomingData[256] = "";
			m_serialPort.readData(incomingData, 255);
			return (strcmp(incomingData, "done") != 0);
		}
		return false;
	}
}
