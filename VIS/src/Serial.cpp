#include "pch.h"
#include "Serial.h"

namespace vis
{

	Serial::Serial(const char *portName)
	{
		// We're not yet connected
		this->m_connected = false;

		try 
		{

		}
		catch (std::exception &e)
		{

		}
		// Try to connect to the given port throuh CreateFile
		this->m_serialHandler = CreateFileA(portName,
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			FILE_ATTRIBUTE_NORMAL,
			NULL);

		// Check if the connection was successfull
		if (this->m_serialHandler == INVALID_HANDLE_VALUE)
		{
			// If not success full display an Error
			if (GetLastError() == ERROR_FILE_NOT_FOUND) {

				throw std::runtime_error("ERROR: Handle was not attached. Reason: " + std::string(portName) + " not available.\n" );

			}
			else
			{
				printf("ERROR!!!");
			}
		}
		else
		{
			// If connected we try to set the comm parameters
			DCB dcbSerialParams = { 0 };

			// Try to get the current
			if (!GetCommState(this->m_serialHandler, &dcbSerialParams))
			{
				// If impossible, show an error
				printf("failed to get current serial parameters!");
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
					throw std::runtime_error("ALERT: Could not set Serial Port parameters");
				}
				else
				{
					// If everything went fine we're connected
					this->m_connected = true;
					// Flush any remaining characters in the buffers 
					PurgeComm(this->m_serialHandler, PURGE_RXCLEAR | PURGE_TXCLEAR);
				}
			}
		}

	}

	Serial::~Serial()
	{
		// Check if we are connected before trying to disconnect
		if (this->m_connected)
		{
			// We're no longer connected
			this->m_connected = false;
			// Close the serial handler
			CloseHandle(this->m_serialHandler);
		}
	}

	int Serial::readData(char *buffer, unsigned int nbChar)
	{
		// Number of bytes we'll have read
		DWORD bytesRead;
		// Number of bytes we'll really ask to read
		unsigned int toRead;

		// Use the ClearCommError function to get status info on the Serial port
		ClearCommError((this->m_serialHandler), &(this->m_errors), &(this->m_status));

		// Check if there is something to read
		if (this->m_status.cbInQue > 0)
		{
			// If there is we check if there is enough data to read the required number
			// of characters, if not we'll read only the available characters to prevent
			// locking of the application.
			if (this->m_status.cbInQue > nbChar)
			{
				toRead = nbChar;
			}
			else
			{
				toRead = this->m_status.cbInQue;
			}

			// Try to read the require number of chars, and return the number of read bytes on success
			if (ReadFile(this->m_serialHandler, buffer, toRead, &bytesRead, NULL))
			{
				return bytesRead;
			}

		}

		// If nothing has been read, or that an error was detected return 0
		return 0;

	}


	bool Serial::writeData(const char *buffer, unsigned int nbChar)
	{
		DWORD bytesSend;

		// Try to write the buffer on the Serial port
		if (!WriteFile(this->m_serialHandler, (void *)buffer, nbChar, &bytesSend, 0))
		{
			// In case it don't work get comm error and return false
			ClearCommError(this->m_serialHandler, &this->m_errors, &this->m_status);

			return false;
		}
		else
			return true;
	}

	bool Serial::isConnected()
	{
		// Simply return the connection status
		return this->m_connected;
	}

}