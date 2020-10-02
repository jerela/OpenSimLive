
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

// Send some integers over the wire
bool Client::SendInts(int* pVals, int iLen)
{
	int	i = 0;

	if (send(m_iSock, (char *) pVals, sizeof(int) * iLen, 0) == -1)
	{
		perror("Client::SendInts, send ints");              
		return false;
	}

	if (VERBOSE)
	{ 
		printf("Client: sending %d ints - ", iLen);                       
		for (i = 0; i < iLen; i++)
			printf("%d ", pVals[i]);
		printf("\n");
	}

#ifdef DEBUG_ACK
	if (!RecvAck())
		return false;
	if (!SendAck())
		return false;	
#endif

	return true;
}

// Send some floats over the wire
bool Client::SendFloats(float* pVals, int iLen)
{
	int i = 0;

	if (send(m_iSock, (char *) pVals, sizeof(float) * iLen, 0) == -1)
	{
		perror("Client::SendFloats, send floats");              
		return false;
	}

	if (VERBOSE)
	{ 
		printf("Client: sending %d floats - ", iLen);                       
		for (i = 0; i < iLen; i++)
			printf("%0.3f ", pVals[i]);
		printf("\n");
	}

#ifdef DEBUG_ACK
	if (!RecvAck())
		return false;
	if (!SendAck())
		return false;	
#endif

	return true;
}

// Send some doubles over the wire
bool Client::SendDoubles(double* pVals, int iLen)
{
	int i = 0;

	if (send(m_iSock, (char *) pVals, sizeof(double) * iLen, 0) == -1)
	{
		perror("Client::SendDoubles, send doubles");              
		return false;
	}

	if (VERBOSE)
	{ 
		printf("Client: sending %d doubles - ", iLen);                       
		for (i = 0; i < iLen; i++)
			printf("%0.3f ", pVals[i]);
		printf("\n");
	}

#ifdef DEBUG_ACK
	if (!RecvAck())
		return false;
	if (!SendAck())
		return false;	
#endif

	return true;
}

// Send some bytes over the wire
bool Client::SendBytes(char* pVals, int iLen)
{
	int i = 0;

	if (send(m_iSock, (char *) pVals, iLen, 0) == -1)
	{
		perror("Client::SendBytes, send bytes");
		return false;
	}

	if (VERBOSE)
	{
		printf("Client: sending %d bytes - ", iLen);

		for (i = 0; i < iLen; i++)
			printf("%d ", pVals[i]);
		printf("\n");
	}

#ifdef DEBUG_ACK
	if (!RecvAck())
		return false;
	if (!SendAck())
		return false;	
#endif

	return true;
}

// receive a string, returns num of bytes received
int Client::RecvString(char* pStr, int iMax, char chTerm)
{
	int		iNumBytes	= 0;
	bool	bEnd		= false;
	int		i			= 0;
	int		j			= 0;
	int		iLastRead	= 0;

	// Set the temp buffer to our already allocated spot
	char* pTemp = (char *) m_pBuffer;

	// This is annoying, but the java end is sending a char
	// at a time, so we recv some chars (probably 1), append
	// it to our str string, then carry on until we see
	// the terminal character.
	/*while (!bEnd)
	{
		if ((iLastRead = recv(m_iSock, pTemp, CLIENT_BUFF_SIZE, 0)) == -1)
		{
			perror("Client::RecvString, recv");
			return -1;
		}
		for (i = 0; i < iLastRead; i++)
		{
			pStr[j] = pTemp[i];	
			j++;
		}
		if ((pTemp[i - 1] == chTerm) || (j == (iMax - 1)))
			bEnd = true;

		iNumBytes += iLastRead;
	}*/

	iLastRead = recv(m_iSock, pTemp, CLIENT_BUFF_SIZE, 0);
	pStr = pTemp;


	//pStr[j] = '\0';

	if (VERBOSE) 
		printf("Client: received '%s'\n", pStr);                       	

/*#ifdef DEBUG_ACK
	if (!SendAck())
		return -1;
	if (!RecvAck())
		return -1;
#endif*/

	return iNumBytes;
}


// Receive some ints, returns num of ints received.
int Client::RecvInts(int* pVals, int iLen)
{
	int			i				= 0;
	int			j				= 0;
	char*		pTemp			= NULL;
	char*		pResult			= NULL;
	int			iTotalBytes		= 0;
	int			iNumBytes		= 0;
	bool		bEnd			= false;

	pTemp	= (char *) m_pBuffer;
	pResult	= (char *) m_pBuffer2;

	// We receiving the incoming ints one byte at a time.
	while (!bEnd)
	{
		if ((iNumBytes = recv(m_iSock, pTemp, CLIENT_BUFF_SIZE, 0)) == -1)
		{
			perror("Client::RecvInts, recv");
			return -1;
		}
		for (i = 0; i < iNumBytes; i++)
		{
			pResult[j] = pTemp[i];	
			j++;
		}

		iTotalBytes += iNumBytes;
		if (iTotalBytes == (iLen * sizeof(int)))
			bEnd = true;
	}

	// Now we need to put the array of bytes into the array of ints
	char*	ptr		= (char *) pVals;
	int		iNum	= j / sizeof(int);

	for (i = 0; i < j; i++)
	{
		ptr[i] = pResult[i];
	}

	if (VERBOSE) 
	{
		printf("Client: received %d ints - ", iNum);             	
		for (i = 0; i < iNum; i++)
			printf("%d ", pVals[i]);
		printf("\n");
	}

#ifdef DEBUG_ACK
	if (!SendAck())
		return -1;
	if (!RecvAck())
		return -1;
#endif

	return iNum;
}

// Receive some floats, returns number of floats received.
int Client::RecvFloats(float* pVals, int iLen)
{
	int			i				= 0;
	int			j				= 0;
	char*		pTemp			= NULL;
	char*		pResult			= NULL;
	int			iTotalBytes		= 0;
	int			iNumBytes		= 0;
	bool		bEnd			= false;

	pTemp	= (char *) m_pBuffer;
	pResult	= (char *) m_pBuffer2;

	// We receiving the incoming floats one byte at a time.
	while (!bEnd)
	{
		// iNumBytes gets assigned the number of bytes received by recv, the values are stored in pTemp
		if ((iNumBytes = recv(m_iSock, pTemp, CLIENT_BUFF_SIZE, 0)) == -1)
		{
			perror("Client::RecvFloats, recv");
			return -1;
		}
		printf("CHK1, iNumBytes=%i\n",iNumBytes);
		// put all of pTemp into pResult
		for (i = 0; i < iNumBytes; i++)
		{
			pResult[j] = pTemp[i];
			j++;
		}

		bEnd = true;
		//iTotalBytes += iNumBytes;
		//if (iTotalBytes == (iLen * sizeof(float)))
		//	bEnd = true;
	}

	/*int streak = 0;
	int nstreak = 0;
	bool last = 0;
	for (i = 0; i < iNumBytes; ++i)
	{
		if (pResult[i] != 0)
		{
			if (last == 0)
			{
				if (nstreak < 20)
					asd;
				if (nstreak == 20)
					asd;
				if (nstreak > 20)
					asd;
			}
			++streak;
			last = 1;
		}
		else
		{
			++nstreak;
			last = 0;
		}

	}*/



	// Now we need to put the array of bytes into the array of floats
	char*	ptr		= NULL;
	int		iNum	= j / sizeof(float);

	printf("CHK3\n");

	/*for (i = 0; i < 16; i++)
		printf("%f ", pVals[i]);
	printf("\n");*/

	ptr = (char *) pVals;

	printf("CHK4, j=%i \n",j);

	for (i = 0; i < iLen; i++)
	{
		//printf("i: %i \n", i);
		ptr[i] = pResult[i];
		//if (i < 16)
		//	printf("Writing %i to ptr[i]\n", pResult[i]);
	}

	printf("CHK5\n");

	if (VERBOSE) 
	{
		printf("Client: received %i floats - ", iNum);
		//int max = 16;
		//if (iNum < max)
		//	max = iNum;
		//for (i = 0; i < max; i++)
		//	printf("%f ", pVals[i]);
		printf("\n");
	}

/*#ifdef DEBUG_ACK
	if (!SendAck())
		return -1;
	if (!RecvAck())
		return -1;
#endif*/

	return iNum;

}

// Receive some doubles, returns number of doubles received.
int Client::RecvDoubles(double* pVals, int iLen)
{
	int			i				= 0;
	int			j				= 0;
	char*		pTemp			= NULL;
	char*		pResult			= NULL;
	int			iTotalBytes		= 0;
	int			iNumBytes		= 0;
	bool		bEnd			= false;

	pTemp	= (char *) m_pBuffer;
	pResult	= (char *) m_pBuffer2;

	// We receiving the incoming doubles one byte at a time.
	while (!bEnd)
	{
		if ((iNumBytes = recv(m_iSock, pTemp, CLIENT_BUFF_SIZE, 0)) == -1)
		{
			perror("Client::RecvDoubles, recv");
			return -1;
		}
		for (i = 0; i < iNumBytes; i++)
		{
			pResult[j] = pTemp[i];	
			j++;
		}

		iTotalBytes += iNumBytes;
		if (iTotalBytes == (iLen * sizeof(double)))
			bEnd = true;
	}

	// Now we need to put the array of bytes into the array of doubles
	char*	ptr;
	int		iNum = j / sizeof(double);

	ptr = (char *) pVals;

	for (i = 0; i < j; i++)
	{
		ptr[i] = pResult[i];
	}

	if (VERBOSE) 
	{
		printf("Client: received %d doubles - ", iNum);             	
		for (i = 0; i < iNum; i++)
			printf("%e ", pVals[i]);
		printf("\n");
	}

#ifdef DEBUG_ACK
	if (!SendAck())
		return -1;
	if (!RecvAck())
		return -1;
#endif

	return iNum;
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
		printf("Client: received %d bytes - ", iTotalBytes);
		//for (i = 0; i < iLen; i++)
		//	printf("%d ", pVals[i]);
		printf("\n");
	}

/*#ifdef DEBUG_ACK
	if (!SendAck())
		return -1;
	if (!RecvAck())
		return -1;
#endif*/

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

// Send a packet of bytes using a datagram
bool Client::SendDatagram(char* pVals, int iLen)
{
	// TBD
	return false;
}

// Receive a datagram
int Client::RecvDatagram(char* pVals, int iLen)
{
	// TBD
	return 0;
}