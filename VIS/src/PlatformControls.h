#pragma once

#include "Camera.h"
#include "Serial.h"

namespace vis
{

	/// Controls to rotate the platform and check its state
	class IPlatformControls
	{
	public:


		virtual ~IPlatformControls() = default;


		/// Start rotating the platform, stopping after a full rotation
		virtual void startRotation() = 0;


		/// Whether the platform is currently rotating
		virtual bool isRotating() = 0;

	};


	/// Mock platform controls used to manipulate footage used by the mock camera
	class MockPlatformControls : public IPlatformControls
	{
	public:

		/// Build controls that will manipulate the footage of the given camera
		/// @param spMockCamera the camera to manipulate;
		MockPlatformControls(const std::shared_ptr<MockCamera>& spMockCamera)
			: m_spMockCamera(spMockCamera) {}


		void startRotation() override;
		bool isRotating() override;

	private:
		std::shared_ptr<MockCamera> m_spMockCamera;
	};

	/// Servo platform controls used to rotate the platform through a serial port
	class ServoPlatformControls : public IPlatformControls
	{
	public:
		ServoPlatformControls(const std::string& portName)
			: m_serialPort(portName.c_str()) {}

		void startRotation() override;
		bool isRotating() override;
	private:
			Serial m_serialPort;
	};

}
