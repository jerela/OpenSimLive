
#define VERBOSE 1      // turn on or off debugging output

#include "Client.h"

Client::Client(int iPort, int iPortDatagram, const char* pStrHost, bool bReverse, bool* pResult)
{
	struct hostent*	he = NULL;

	m_iPort = iPort;
	m_iPortDatagram = iPortDatagram;
	m_iSock = -1;
	m_pBuffer = NULL;
	m_pBuffer2 = NULL;

	if (pResult)
		*pResult = false;

	// Allocate our temporary buffers that are used to convert data types
	m_pBuffer = (double *) malloc(sizeof(double) * CLIENT_BUFF_SIZE);
	if (!m_pBuffer)
	{
		perror("Server::Server, failed to malloc buffer!");
		return;
	}

	m_pBuffer2 = (double *) malloc(sizeof(double) * CLIENT_BUFF_SIZE);
	if (!m_pBuffer2)
	{
		free(m_pBuffer);
		m_pBuffer = NULL;
		perror("Server::Server, failed to malloc buffer2!");
		return;
	}

#ifdef _WIN32
	// For Windows, we need to fire up the Winsock DLL before we can do socket stuff.
	WORD wVersionRequested;
    WSADATA wsaData;
    int err;

	// Use the MAKEWORD(lowbyte, highbyte) macro declared in Windef.h 
    wVersionRequested = MAKEWORD(2, 2);

    err = WSAStartup(wVersionRequested, &wsaData);
    if (err != 0) 
	{
        // Tell the user that we could not find a usable Winsock DLL
        perror("Client::Client, WSAStartup failed with error");
        return;
    }
#endif
	
	if (VERBOSE) 
		printf("Client: opening socket to %s on port = %d\n", pStrHost, m_iPort);

	if ((he = gethostbyname(pStrHost)) == NULL) 
	{
		perror("Client::Client, gethostbyname");
		return;
	}

	if ((m_iSock = socket(AF_INET, SOCK_STREAM, 0)) == -1) 
	{
		perror("Client::Client, socket");
		return;
	}

	m_addrRemote.sin_family		= AF_INET;        
	m_addrRemote.sin_port		= htons(m_iPort);      
	m_addrRemote.sin_addr		= *((struct in_addr *) he->h_addr); 
	memset(&(m_addrRemote.sin_zero), 0, 8);

	if (connect(m_iSock, (struct sockaddr *) &m_addrRemote, sizeof(struct sockaddr)) == -1) 
	{
		perror("Client::Client, connect m_iSock");
		return;
	}

	// See if we are suppose to open a port for datagram messages
	if (m_iPortDatagram != -1)
	{
		// TBD
	}

	// Send out request for reversed bits or not
	/*char temp[1];
	if (bReverse)
	{
		temp[0] = 1;
		if (send(m_iSock, temp, 1, 0) == -1)
		{
			perror("Client::Client, send 1");
			return;
		}
	}
	else
	{
		temp[0] = 0;
		if (send(m_iSock, temp, 1, 0) == -1)
		{
			perror("Client::Client, send 2");
			return;
		}
	}*/
	
	if (pResult)
		*pResult = true;
}

Client::~Client()
{
#ifdef _WIN32
	// Windows specific socket shutdown code
	WSACleanup();
#endif

	if (m_pBuffer)
	{
		free(m_pBuffer);
		m_pBuffer = NULL;
	}
	if (m_pBuffer2)
	{
		free(m_pBuffer2);
		m_pBuffer2 = NULL;
	}

}

// Send a string to the socket
bool Client::SendString(char* pStr)
{
	if (send(m_iSock, (char *) pStr, strlen(pStr), 0) == -1)
	{
		perror("Client::SendString, send");
		return false;
	}

	if (VERBOSE) 
		printf("Client: sending string '%s'\n", pStr);                       


	if (pStr != "\r\n\r\n")
		return true;


	printf("Awaiting reply...\n");


	int		iLastRead = 0;

	// Set the temp buffer to our already allocated spot
	char* pTemp = (char*)m_pBuffer;


	iLastRead = recv(m_iSock, pTemp, CLIENT_BUFF_SIZE, 0);
	pStr = pTemp;

	if (VERBOSE)
		printf("Client: received '%s'\n", pStr);






	return true;
}

// Receive some bytes, returns number of bytes received.
int Client::RecvBytes(char* pVals, int iLen)
{
	int			i				= 0;
	int			j				= 0;
	char*		pTemp			= NULL;
	int			iTotalBytes		= 0;
	int			iNumBytes		= 0;
	bool		bEnd			= false;

	pTemp = (char *) m_pBuffer;

	// We receiving the incoming ints one byte at a time.
	while (!bEnd)
	{
		if ((iNumBytes = recv(m_iSock, pTemp, CLIENT_BUFF_SIZE, 0)) == -1)
		{
			perror("Client::RecvBytes, recv");
			return -1;
		}
		//if (iNumBytes != 6400)
		//	return -1;
		for (i = 0; i < iNumBytes; i++)
		{
			pVals[j] = pTemp[i];
			j++;
			if (j >= iLen) {
				bEnd = true;
				break;
			}
		}
		//bEnd = true;
		iTotalBytes += iNumBytes;
		if (iTotalBytes == iLen)
			bEnd = true;
		//printf("iNumBytes: %i, ", iNumBytes);
	}


	if (VERBOSE)
	{
		//printf("Client: received %d bytes - ", iTotalBytes);
		//printf("\n");
	}

	return iTotalBytes;
}


// Shut down the socket.
bool Client::Close()
{
	if (shutdown(m_iSock, SD_BOTH) == -1)
		return false;
#ifndef _WIN32
	close(m_iSock);
#endif
	return true;
}

// Recieve a short ack from the server.
bool Client::RecvAck()
{
	char temp[1];
	int iTotal = 0;
	int iResult = 0;

	if (VERBOSE)
		printf("Waiting for ack...\n");

	while ((iTotal < 1) && (iResult != -1))
	{
		iResult = recv(m_iSock, temp, 1, 0);	
		iTotal += iResult;	
	}
	if (iResult == -1)
	{
		perror("Client::RecvAck, recv");
		return false;
	}

	if (VERBOSE)
		printf("Ack recieved.\n");

	return true;
}

// Send a short ack to the server.
bool Client::SendAck()
{
	char temp[1];
	temp[0] = 42;

	if (VERBOSE)
		printf("Sending ack...\n");

	if (send(m_iSock, temp, 1, 0) == -1)
	{
		perror("Client::SendAck, send");
		return false;
	}

	return true;
}

