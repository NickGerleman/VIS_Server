#pragma once
namespace vis
{

	class Serial
	{
	public:

		/// Initialize Serial communication with the given port name
		/// @param name of the Serial communication port
		Serial(const char *portName);

		/// Close the connection
		~Serial();

		/// Read data in a buffer
		/// @param buffer block of memory to write incoming data
		/// @param nbChar number of bytes to read
		int readData(char *buffer, unsigned int nbChar);

		/// Writes data from a buffer through the Serial connection
		/// @param buffer block of memory to send outgoing data
		/// @param nbChar number of bytes to write
		bool writeData(const char *buffer, unsigned int nbChar);

		/// Check if we are actually connected
		bool isConnected();

	private:
		/// Serial comm handler
		HANDLE m_serialHandler;
		/// Connection status
		bool m_connected;
		/// Get various information about the connection
		COMSTAT m_status;
		/// Keep track of last error
		DWORD m_errors;

	};

}