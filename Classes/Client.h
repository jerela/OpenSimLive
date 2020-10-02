// A C++ socket client class
//
// Keith Vertanen 11/98, updated 12/08

#ifndef _CLIENT_H__
#define _CLIENT_H__

// Adds in the send/recv acks after each message.
#define DEBUG_ACK

static const int CLIENT_BUFF_SIZE = 64000;

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>

#ifdef _WIN32
#include <winsock2.h>
#else
#include <netinet/in.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

// Duplicated from winsock2.h
#define SD_RECEIVE      0x00
#define SD_SEND         0x01
#define SD_BOTH         0x02

#endif

class Client
{
	public:
		Client(int iPort, int iPortDatagram, const char* pStrHost, bool bReverse, bool* pResult);
		~Client();

		bool				Close();										// Close the socket

		bool				SendString(char* pStr);							// Send a string to socket
		bool				SendInts(int* pVals, int iLen);					// Send some integers
		bool				SendBytes(char* pVals, int iLen);				// Send some bytes
		bool				SendFloats(float* pVals, int iLen);				// Send some floats
		bool				SendDoubles(double* pVals, int iLen);			// Send some doubles
		
		int					RecvString(char* pStr, int iMax, char chTerm);	// Receive a string
		int					RecvInts(int* pVals, int iLen);  				// Receive some ints
		int					RecvFloats(float* pVals, int iLen);  			// Receive some floats
		int					RecvDoubles(double* pVals, int iLen);  			// Receive some doubles
		int					RecvBytes(char* pVals, int iLen);  				// Receive some bytes
				
		// NOTE: these are not currently implemented!
		bool				SendDatagram(char *val, int len);
		int					RecvDatagram(char* pVals, int iLen);  			// Receive a datagram

	protected:		
		int					m_iPort;							// Port I'm listening on
		int					m_iPortDatagram;					// Datagram port I'm listening on
		int					m_iSock;							// Socket connection
		struct sockaddr_in	m_addrRemote;						// Connector's address information
		double*				m_pBuffer;							// Reuse the same memory for buffer
		double*				m_pBuffer2;

	private:
		bool				RecvAck();
		bool				SendAck();
};

#endif
