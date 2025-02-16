#include "pch.h"
#include "Serial.h"

namespace vis
{

	Serial::Serial(const char *portName)
	{
		// We're not yet connected
		m_connected = false;

		// Try to connect to the given port throuh CreateFile
		m_serialHandler = CreateFileA(portName,
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			FILE_ATTRIBUTE_NORMAL,
			NULL);

		// Check if the connection was successfull
		if (m_serialHandler == INVALID_HANDLE_VALUE)
		{
			// If not success full display an Error
			if (GetLastError() == ERROR_FILE_NOT_FOUND) {
				throw std::runtime_error("Handle was not attached. Reason: " + std::string(portName) + " not available.\n" );
			}
			else
				throw std::runtime_error("Unknown Error");
		}
		else
		{
			// If connected we try to set the comm parameters
			DCB dcbSerialParams = { 0 };

			// Try to get the current
			if (!GetCommState(m_serialHandler, &dcbSerialParams))
			{
				// If impossible, show an error
				throw std::runtime_error("Failed to get current serial parameters!");
			}
			else
			{
				// Define serial connection parameters for the arduino board
				dcbSerialParams.BaudRate = CBR_9600;
				dcbSerialParams.ByteSize = 8;
				dcbSerialParams.StopBits = ONESTOPBIT;
				dcbSerialParams.Parity = NOPARITY;
				// Setting the DTR to Control_Enable ensures that the Arduino is properly
				// reset upon establishing a connection
				dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;

				// Set the parameters and check for their proper application
				if (!SetCommState(m_serialHandler, &dcbSerialParams))
				{
					throw std::runtime_error("Could not set Serial Port parameters");
				}
				else
				{
					// If everything went fine we're connected
					m_connected = true;
					// Flush any remaining characters in the buffers 
					PurgeComm(m_serialHandler, PURGE_RXCLEAR | PURGE_TXCLEAR);
				}
			}
		}

	}

	Serial::~Serial()
	{
		// Check if we are connected before trying to disconnect
		if (m_connected)
		{
			// We're no longer connected
			m_connected = false;
			// Close the serial handler
			CloseHandle(m_serialHandler);
		}
	}

	int Serial::readData(char *buffer, unsigned int nbChar)
	{
		// Number of bytes we'll have read
		DWORD bytesRead;
		// Number of bytes we'll really ask to read
		unsigned int toRead;

		// Use the ClearCommError function to get status info on the Serial port
		ClearCommError((m_serialHandler), &(m_errors), &(m_status));

		// Check if there is something to read
		if (m_status.cbInQue > 0)
		{
			// If there is we check if there is enough data to read the required number
			// of characters, if not we'll read only the available characters to prevent
			// locking of the application.
			if (m_status.cbInQue > nbChar)
				toRead = nbChar;
			else
				toRead = m_status.cbInQue;

			// Try to read the require number of chars, and return the number of read bytes on success
			if (ReadFile(m_serialHandler, buffer, toRead, &bytesRead, NULL))
				return bytesRead;

		}

		// If nothing has been read, or that an error was detected return 0
		return 0;

	}


	bool Serial::writeData(const char *buffer, unsigned int nbChar)
	{
		DWORD bytesSend;

		// Try to write the buffer on the Serial port
		if (!WriteFile(m_serialHandler, (void *)buffer, nbChar, &bytesSend, 0))
		{
			// In case it don't work get comm error and return false
			ClearCommError(m_serialHandler, &m_errors, &m_status);

			return false;
		}
		else
			return true;
	}

	bool Serial::isConnected()
	{
		// Simply return the connection status
		return m_connected;
	}

}
