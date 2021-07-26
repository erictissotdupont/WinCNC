/*
 * sockect.c
 *   Listener and talker socket
*/
#include "CNC.h"
#include "Ws2tcpip.h"
#include "Mstcpip.h"

#include "status.h"
#include "socket.h"

// This is the port onto which the CNC advertizes its presence. Those
// messages include the current state and position. Those are sent every
// 1 sec when the CNC is idle. Can go slower when the machine is idle for
// a long time.
#define BROADCAST_PORT			50042

// This is the port the host uses for sending commands. The CNC acknowleges
// on the same port.
#define DATA_PORT				50043

// This is the maximum time a message will wait in the outbound 
// buffer before being sent
#define MSG_WAIT_PERIOD_MS		30

// This is the buffer for accumulating outbound commands. Note this does not
// include the header. So the actual message will be a few bytes long.
#define OUT_MSG_BUF_SIZE		1024

// The CNC will nack immediately after reception and then every 100ms. 
// This timeout is designed to allow for two consecutive missed NACK messages
// not causing a retry.
#define COMMAND_TIMEOUT_MS		150

// This is how long a lack of communication will result in a disconnected state
#define DISCONNECT_TIMEOUT_MS	30000

// Rather than infinite number of retries, 
#define MAX_COMMAND_RETRY		((DISCONNECT_TIMEOUT_MS / COMMAND_TIMEOUT_MS) + 1)


#define IPSTRSIZE				80
#define MAX_RESPONSE			80

#define BUFFER_MUTEX_TIMEOUT	1000

#define MAX_CALLBACK			10



void(*g_pEventCallback[CND_MAX_EVENT][MAX_CALLBACK])( PVOID );
int g_pCallbg_ACKcount[CND_MAX_EVENT];
#define NOTIFY_CALLBACK(event,param) for(int j=0;j<g_pCallbg_ACKcount[event];j++) g_pEventCallback[event][j](param);

char response[MAX_RESPONSE];
int rspCnt = 0;
HANDLE hResponseReceived = NULL;
int bConnected = 0;
int bRun = 1;

bool g_bRun = true;

HANDLE g_hConnected;
HANDLE g_hDisconnected;
HANDLE g_hBufferFull;
HANDLE g_hBufferEmpty;
HANDLE g_hAckReceived;
HANDLE g_hNackReceived;

SOCKET g_CNCSocket;
char g_szCNCIP[IPSTRSIZE];
struct sockaddr_in g_CncAddr;

#define STATUS_LITLE_ENDIAN   0x80000000
unsigned long g_Status;

unsigned long g_TXcount = 0;
unsigned long g_RXcount = 0;
unsigned long g_RetCount = 0;
int g_msgInQueue = 0;

long errCount = 0;
long repeatCount = 0;

int g_outCharCount;
int g_outCmdCount;
char g_outBuffer[OUT_MSG_BUF_SIZE];
HANDLE g_outBufferMutex = NULL;
unsigned long g_msgSeq;

HANDLE hDebug = INVALID_HANDLE_VALUE;

int getAckPendingCount()
{
	return 0;
}

int isCncConnected()
{
	return bConnected;
}

long getCncErrorCount()
{
	return errCount;
}

static const unsigned char crc8_table[256] = {
	0x00, 0xF7, 0xB9, 0x4E, 0x25, 0xD2, 0x9C, 0x6B,
	0x4A, 0xBD, 0xF3, 0x04, 0x6F, 0x98, 0xD6, 0x21,
	0x94, 0x63, 0x2D, 0xDA, 0xB1, 0x46, 0x08, 0xFF,
	0xDE, 0x29, 0x67, 0x90, 0xFB, 0x0C, 0x42, 0xB5,
	0x7F, 0x88, 0xC6, 0x31, 0x5A, 0xAD, 0xE3, 0x14,
	0x35, 0xC2, 0x8C, 0x7B, 0x10, 0xE7, 0xA9, 0x5E,
	0xEB, 0x1C, 0x52, 0xA5, 0xCE, 0x39, 0x77, 0x80,
	0xA1, 0x56, 0x18, 0xEF, 0x84, 0x73, 0x3D, 0xCA,
	0xFE, 0x09, 0x47, 0xB0, 0xDB, 0x2C, 0x62, 0x95,
	0xB4, 0x43, 0x0D, 0xFA, 0x91, 0x66, 0x28, 0xDF,
	0x6A, 0x9D, 0xD3, 0x24, 0x4F, 0xB8, 0xF6, 0x01,
	0x20, 0xD7, 0x99, 0x6E, 0x05, 0xF2, 0xBC, 0x4B,
	0x81, 0x76, 0x38, 0xCF, 0xA4, 0x53, 0x1D, 0xEA,
	0xCB, 0x3C, 0x72, 0x85, 0xEE, 0x19, 0x57, 0xA0,
	0x15, 0xE2, 0xAC, 0x5B, 0x30, 0xC7, 0x89, 0x7E,
	0x5F, 0xA8, 0xE6, 0x11, 0x7A, 0x8D, 0xC3, 0x34,
	0xAB, 0x5C, 0x12, 0xE5, 0x8E, 0x79, 0x37, 0xC0,
	0xE1, 0x16, 0x58, 0xAF, 0xC4, 0x33, 0x7D, 0x8A,
	0x3F, 0xC8, 0x86, 0x71, 0x1A, 0xED, 0xA3, 0x54,
	0x75, 0x82, 0xCC, 0x3B, 0x50, 0xA7, 0xE9, 0x1E,
	0xD4, 0x23, 0x6D, 0x9A, 0xF1, 0x06, 0x48, 0xBF,
	0x9E, 0x69, 0x27, 0xD0, 0xBB, 0x4C, 0x02, 0xF5,
	0x40, 0xB7, 0xF9, 0x0E, 0x65, 0x92, 0xDC, 0x2B,
	0x0A, 0xFD, 0xB3, 0x44, 0x2F, 0xD8, 0x96, 0x61,
	0x55, 0xA2, 0xEC, 0x1B, 0x70, 0x87, 0xC9, 0x3E,
	0x1F, 0xE8, 0xA6, 0x51, 0x3A, 0xCD, 0x83, 0x74,
	0xC1, 0x36, 0x78, 0x8F, 0xE4, 0x13, 0x5D, 0xAA,
	0x8B, 0x7C, 0x32, 0xC5, 0xAE, 0x59, 0x17, 0xE0,
	0x2A, 0xDD, 0x93, 0x64, 0x0F, 0xF8, 0xB6, 0x41,
	0x60, 0x97, 0xD9, 0x2E, 0x45, 0xB2, 0xFC, 0x0B,
	0xBE, 0x49, 0x07, 0xF0, 0x9B, 0x6C, 0x22, 0xD5,
	0xF4, 0x03, 0x4D, 0xBA, 0xD1, 0x26, 0x68, 0x9F
};

unsigned char crc8(unsigned char* pt, unsigned int nbytes, unsigned char crc)
{
	while (nbytes-- > 0)
	{
		crc = crc8_table[(crc ^ *pt++) & 0xff];
	}
	return crc;
}

// Returns the 8bit CRC of the 3 x 32bit long integrers in an array taking into
// account the endianness of the CNC remote processor.
//
unsigned char GetPosCRC(long x, long y, long z)
{
	long posForCRC[3];
	if (g_Status & STATUS_LITLE_ENDIAN)
	{
		posForCRC[0] = x;
		posForCRC[1] = y;
		posForCRC[2] = z;
	}
	else
	{
		posForCRC[0] = htonl(x);
		posForCRC[1] = htonl(y);
		posForCRC[2] = htonl(z);
	}
	return crc8((unsigned char*)posForCRC, sizeof(posForCRC), 0xFF);
}


unsigned long getDurationOfCommandsInPipe();


void getSocketStatusString(char* szBuffer, unsigned int cbBuffer)
{
	sprintf_s(szBuffer, cbBuffer, "%s - Tx:%lu - Rx:%lu - Ret:%lu - %d %%",
		g_szCNCIP,
		g_TXcount,
		g_RXcount,
		g_RetCount,
		(int)((100 * g_msgInQueue ) / 256 ));
}


tStatus waitForStatus(unsigned long timeout)
{
	tStatus ret = retUnknownErr;

	// Allow for one more second than the max duration of commands pending for
	// the answer to come back.
	switch (WaitForSingleObject(hResponseReceived, timeout))
	{
	case WAIT_OBJECT_0:
		ret = retSuccess;
		break;
	case WAIT_TIMEOUT:
		ret = retCncStatusTimeout;
		break;
	default:
		ret = retUnknownErr;
	}
	return ret;
}



tStatus postCommand(char* cmd)
{
	HANDLE hEvent[2];
	tStatus status = retSuccess;
	int cl = strlen(cmd);

	if (!bConnected) return retCncNotConnected;
	if (cmd == NULL) return retInvalidParam;
	if (cl > OUT_MSG_BUF_SIZE) return retInvalidParam;

	// Wait for the output buffer to be available or the stop event
	hEvent[0] = g_outBufferMutex;
	hEvent[1] = g_hDisconnected;
	switch( WaitForMultipleObjects( 2, hEvent, FALSE, INFINITE ))
	{
	default:
		status = retInternalError;
		break;

	case WAIT_OBJECT_0 + 1 :
		status = retStopRequested;
		break;

	case WAIT_OBJECT_0 :

		// First, check if the outbout buffer is full
		if (cl + g_outCharCount + 2 >= sizeof(g_outBuffer))
		{
			// Signal the sender thread to flush the outbound buffer
			SetEvent(g_hBufferFull);
			ResetEvent(g_hBufferEmpty);

			// Release access to the buffer
			ReleaseMutex(g_outBufferMutex);

			hEvent[0] = g_hBufferEmpty;
			hEvent[1] = g_hDisconnected;
			switch( WaitForMultipleObjects(2, hEvent, FALSE, INFINITE))
			{
			case WAIT_OBJECT_0 : // Buffer is now empty

				// Re-acquire the mutex. This should be instantaneous since
				// the buffer is now empty unless a "sendCommand" came in the
				// middle.
				if (WaitForSingleObject(g_outBufferMutex, 1000) != WAIT_OBJECT_0)
				{
					status = retBufferMutexTimeout;
				}
				break;
			case WAIT_OBJECT_0 + 1 :
				status = retStopRequested;
				break;
			default :
				status = retInternalError;
				break;
			}
		}

		if (status == retSuccess)
		{
			strcpy_s(&g_outBuffer[g_outCharCount], sizeof(g_outBuffer) - g_outCharCount, cmd);
			g_outCharCount += cl;
			g_outBuffer[g_outCharCount++] = '|';
			g_outBuffer[g_outCharCount] = '\0';
			g_outCmdCount++;

			ReleaseMutex(g_outBufferMutex);
		}
		break;
	}
	return status;
}

bool sendAndWaitForAck(char* msg, size_t cbMsg)
{
	int iResult;
	bool bSuccess = false;
	int retry = 0;

	while ( !bSuccess && retry++ < MAX_COMMAND_RETRY )
	{
		if (sendto(g_CNCSocket, msg, strlen(msg) + 1, 0, (SOCKADDR*)&g_CncAddr, sizeof(g_CncAddr)) <= 0)
		{
			iResult = WSAGetLastError();
		}
		else
		{
			bool bTimeout = false;
			HANDLE hEvent[3];

			g_TXcount++;

			hEvent[0] = g_hAckReceived;
			hEvent[1] = g_hNackReceived;
			hEvent[2] = g_hDisconnected;

			do
			{
				// The CNC will NACK every 100ms. Retry if we didn't get any
				// communication for 350 which is 2 missed messages in a row.
				//
				switch (WaitForMultipleObjects(3, hEvent, FALSE, 650 ))
				{
				case WAIT_TIMEOUT:
					g_RetCount++;
					bTimeout = true;
					{
						unsigned long seq;
						char str[100];
						sscanf_s(msg + 4, "%lu", &seq);
						sprintf_s( str, 100, "Timeout waiting for %lu", seq);
						OutputDebugStringA(str);						
					}				
					break;

				case WAIT_OBJECT_0: // Ack
				case WAIT_OBJECT_0 + 2: // Stop
					bSuccess = true;
					break;

				case WAIT_OBJECT_0 + 1: // Nack
					// The CNC command pipe is full. It's asking us to stall.
					// Will stay in this loop for as long as the CNC is telling
					// us to wait...
					break;
				}
			} while (!bTimeout && !bSuccess);
		}
	}
	return bSuccess;
}

tStatus sendCommand(char* cmd, char* rsp, size_t cbRsp )
{
	int cbMsg;
	int cl = strlen(cmd);
	char msg[OUT_MSG_BUF_SIZE + 64]; // Extra space is of the header
	tStatus status;
	HANDLE hEvent[2];

	if (!bConnected) return retCncNotConnected;
	if (cmd == NULL) return retInvalidParam;
	if (cl > OUT_MSG_BUF_SIZE) return retInvalidParam;

	// Wait for the output buffer to be available or the stop event
	hEvent[0] = g_outBufferMutex;
	hEvent[1] = g_hDisconnected;
	switch (WaitForMultipleObjects(2, hEvent, FALSE, INFINITE))
	{
	default:
		status = retInternalError;
		break;

	case WAIT_OBJECT_0 + 1:
		status = retStopRequested;
		break;

	case WAIT_OBJECT_0:
		cbMsg = sprintf_s(msg, sizeof(msg), "CMD,%lu,1|%s|", g_msgSeq, cmd) + 1;
		if (sendAndWaitForAck(msg, cbMsg))
		{
			status = retSuccess;
		}
		else
		{
			status = retCncCommunicationError;
		}
		ReleaseMutex(g_outBufferMutex);
		break;
	}
	return status;
}


bool transmit( SOCKET s )
{
	HANDLE hEvent[2];
	char msg[OUT_MSG_BUF_SIZE + 64]; // Extra space is of the header

	hEvent[0] = g_hBufferFull;
	hEvent[1] = g_hDisconnected;

	while( WaitForMultipleObjects(2, hEvent, FALSE, 100) != (WAIT_OBJECT_0 + 1))
	{
		if (WaitForSingleObject(g_outBufferMutex, 1000) == WAIT_OBJECT_0)
		{
			if (g_outCmdCount == 0)
			{
				// Nothing to send... idle
			}
			else
			{
				int cbHeader = sprintf_s(msg, sizeof(msg), "CMD,%lu,%d|", g_msgSeq, g_outCmdCount);
				memcpy( msg + cbHeader, g_outBuffer, g_outCharCount + 1 );

				sendAndWaitForAck( msg, cbHeader + g_outCharCount + 1 );

				g_outCmdCount = 0;
				g_outCharCount = 0;
				SetEvent(g_hBufferEmpty);
			}

			ReleaseMutex(g_outBufferMutex);
		}
		else
		{
			// TODO : Deal with mutext timeout
		}
	}
	return true;
}

bool listen( SOCKET s )
{
	int cnt;
	char msg[OUT_MSG_BUF_SIZE];
	long x, y, z;
	static char statusStr[80];
	unsigned long seq;
	int status;
	
	while( (cnt = recv( s, msg, sizeof(msg), 0)) > 0)
	{
		if (msg[cnt - 1] != 0)
		{
			OutputDebugStringA("Not zero terminated message.");
			msg[cnt] = 0;
		}

		if (strncmp(msg, "ACK,", 4) == 0)
		{
			g_RXcount++;

			if (sscanf_s(msg + 4, "%lu,%ld,%ld,%ld,%d,%d", 
				&seq, 
				&x,&y,&z,
				&g_msgInQueue,
				&status) != 6 )
			{
				OutputDebugStringA("ACK format error.");
			}
			else if (seq != g_msgSeq)
			{
				// OutputDebugStringA("Out of sequence ACK.");
			}
			else
			{
				// TODO : what if the status is not zero ???
				g_msgSeq++;
				SetEvent(g_hAckReceived);

				sprintf_s(statusStr, sizeof(statusStr), "X%ldY%ldZ%ld", x, y, z );
				NOTIFY_CALLBACK(CNC_RESPONSE, statusStr)

				//char str[100];
				//sprintf_s(str, 100, "Got ACK %d\r\n", seq);
				// OutputDebugStringA(str);
			}
		}
		else if( strncmp( msg, "NAK,", 4) == 0)
		{
			g_RXcount++;

			if (sscanf_s(msg + 4, "%lu,%ld,%ld,%ld,%d", &seq, &x, &y, &z, &g_msgInQueue) != 5 )
			{
				OutputDebugStringA("NAK format error.");
			}
			else if (seq != g_msgSeq)
			{
				OutputDebugStringA("Out of sequence NAK.");
			}
			else
			{
				SetEvent(g_hNackReceived);

				sprintf_s(statusStr, sizeof(statusStr), "X%ldY%ldZ%ld", x, y, z);
				NOTIFY_CALLBACK(CNC_RESPONSE, statusStr)
			}
		}
	}

	closesocket(s);
	return true;
}

DWORD senderThread(PVOID pParam)
{
	while (g_bRun)
	{
		HANDLE hEvent[2];
		hEvent[0] = g_hConnected;
		hEvent[1] = g_hDisconnected;
		switch (WaitForMultipleObjects(2, hEvent, FALSE, INFINITE))
		{
		case WAIT_OBJECT_0: // Connected	
			transmit(g_CNCSocket);
			break;
		}
	}
	return 0;
}

DWORD receiverThread(PVOID pParam)
{
	while (g_bRun)
	{
		HANDLE hEvent[2];
		hEvent[0] = g_hConnected;
		hEvent[1] = g_hDisconnected;

		switch (WaitForMultipleObjects(2, hEvent, FALSE, INFINITE))
		{
		case WAIT_OBJECT_0: // Connected	
			listen(g_CNCSocket);
			break;
		}
	}
	return 0;
}


DWORD listenerThread(PVOID pParam)
{
	int iResult, cnt;
	struct sockaddr_in Addr;
	struct sockaddr_in CncAddr;
	char msg[OUT_MSG_BUF_SIZE];
	int idleCount = 0;

	Sleep(30);
	NOTIFY_CALLBACK(CNC_CONNECTED, NULL)

	SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (s == INVALID_SOCKET) 
	{
		iResult = WSAGetLastError();
		return 1;
	}

	memset(&Addr, 0x00, sizeof(Addr));
	Addr.sin_addr.s_addr = htonl(INADDR_ANY);
	Addr.sin_family = AF_INET;
	Addr.sin_port = htons(BROADCAST_PORT);

	if( bind(s, (SOCKADDR*)&Addr, sizeof(Addr)) != 0)
	{
		iResult = WSAGetLastError();
		return 1;
	}

	INT err;
	INT bAllow = 1;
	err = setsockopt(s, SOL_SOCKET, SO_BROADCAST, (char *)&bAllow, sizeof(bAllow));

	while (g_bRun)
	{
		int CncAddrSize = sizeof(CncAddr);
		memset(&CncAddr, 0x00, CncAddrSize);

		if ((cnt = recvfrom(s, msg, sizeof(msg), 0, (SOCKADDR*)&CncAddr, &CncAddrSize)) <= 0)
		{
			iResult = WSAGetLastError();
			return 1;
		}

		if (strncmp(msg, "CNC,", 4 ) == 0)
		{
			unsigned long seq;
			long x, y, z;

			if (sscanf_s(msg + 4, "%lu,%ld,%ld,%ld,%lx",
				&seq,
				&x, &y, &z,
				&g_Status ) != 5)
			{
				// Malformed message
			}
			else
			{
				if (bConnected)
				{
					// TODO : refresh status

				}
				else
				{
					memcpy(&g_CncAddr, &CncAddr, sizeof(g_CncAddr));
					g_CncAddr.sin_port = htons(DATA_PORT);
					RtlIpv4AddressToStringA(&CncAddr.sin_addr, g_szCNCIP);

					g_CNCSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
					if (g_CNCSocket == INVALID_SOCKET)
					{
						return false;
					}

					Addr.sin_port = htons(DATA_PORT);

					iResult = bind(g_CNCSocket, (SOCKADDR*)&Addr, sizeof(Addr));
					if (iResult == SOCKET_ERROR)
					{
						iResult = WSAGetLastError();
						return false;
					}

					OutputDebugStringA("Synchronized seq with CNC");
					g_msgSeq = seq;
					bConnected = true;
					SetEvent(g_hConnected);
				}

				g_RXcount++;
				NOTIFY_CALLBACK(CNC_CONNECTED, NULL)
			}
		}
	}

	iResult = closesocket(s);

#if 0

	SOCKET cnc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	if (cnc == INVALID_SOCKET) {
		return 1;
	}

	/* Just change the port and connect. */
	CncAddr.sin_port = htons(DATA_PORT);

	

	strcpy_s(msg, sizeof(msg), "RST");

	if (sendto(cnc, msg, strlen(msg)+1, 0, (SOCKADDR*)&CncAddr, CncAddrSize ) <= 0) {
		iResult = WSAGetLastError();
		return 1;
	}

	if (bind(cnc, (SOCKADDR*)&CncAddr, CncAddrSize) == 0)
	{
		iResult = WSAGetLastError();
		return 1;
	}

	if ((cnt = recv(cnc, msg, sizeof(msg), 0)) <= 0) {
		iResult = WSAGetLastError();
		return 1;
	}

	msg[cnt] = 0;
	if (strcmp(msg, "RSP,1,0") != 0) {
		Sleep(100);
		return 1;
	}
/*
	strcpy_s(msg, sizeof(msg), "CMD,1,2|1,2,3,4,0|-2,-4,-6,5,0");
	if (sendto(cnc, msg, strlen(msg) + 1, 0, (SOCKADDR*)&CncAddr, CncAddrSize) <= 0) {
		iResult = WSAGetLastError();
		return 1;
	}

	if ((cnt = recv(cnc, msg, sizeof(msg), 0)) <= 0) {
		iResult = WSAGetLastError();
		return 1;
	}
*/

	outSize = 0;
	memset(outBuffer, 0, sizeof(outBuffer));

	printf("Connected.\n");

	DWORD threadId;
	CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)receiverThread, (PVOID)cnc, 0, &threadId);

	while (bRun) {

		if (WaitForSingleObject(mutexBuffer, BUFFER_MUTEX_TIMEOUT) == WAIT_TIMEOUT)
		{
			bRun = FALSE;
			continue;
		}

		if (outBuffer[0] != 0)
		{
			if (outSize != strlen(outBuffer))
			{
				printf("ERROR : buffer size mismatch. Expected %d, got %d.\n", outSize, strlen(outBuffer));
				bRun = 0;
			}
			else
			{
				if (send(cnc, outBuffer, outSize,0) != outSize)
				{
					printf("Write failed.\n");
					bRun = 0;
				}
				else
				{
					if (hDebug != INVALID_HANDLE_VALUE)
					{
						WriteFile(hDebug, "OUT:", 4, NULL, NULL);
						WriteFile(hDebug, outBuffer, outSize, NULL, NULL);
						WriteFile(hDebug, "\r\n", 2, NULL, NULL);
					}
				}
				outSize = 0;
				outBuffer[0] = 0;
			}

			idleCount = 0;
		}
		ReleaseMutex(mutexBuffer);
		// if (pthread_mutex_unlock(&mutexBuffer) < 0) { perror("pthread_mutex_unlock"); }
		Sleep(MSG_WAIT_PERIOD_MS);
	}

	iResult = closesocket(cnc);
	if (iResult == SOCKET_ERROR) {
		iResult = WSAGetLastError();
		WSACleanup();
		return 1;
	}

	WSACleanup();
	return 0;
#endif

	return 0;
}

void registerSocketCallback(CNC_SOCKET_EVENT event, void(*pCallback)(PVOID))
{
	if (g_pCallbg_ACKcount[event] < MAX_CALLBACK)
	{
		g_pEventCallback[event][g_pCallbg_ACKcount[event]++] = pCallback;
	}

	if (event == CNC_CONNECTED && bConnected)
	{
		NOTIFY_CALLBACK(CNC_CONNECTED, NULL)
	}

}

int initSocketCom( )
{
  DWORD threadId;
  int iResult;
  WSADATA wsaData;

  memset(&wsaData, 0x00, sizeof(wsaData));
  if ((iResult = WSAStartup(0x0202, &wsaData)) != NO_ERROR) 
  {
	  return 1;
  }

  

  memset(g_pCallbg_ACKcount, 0x00, sizeof(g_pCallbg_ACKcount));
  memset(g_pEventCallback, 0x00, sizeof(g_pEventCallback));

  //hDebug = CreateFile( L"C:\\Windows\\Temp\\Cncdebug.txt", GENERIC_WRITE, FILE_SHARE_READ, NULL, CREATE_ALWAYS, 0, NULL);

  g_outCmdCount = 0;
  g_outCharCount = 0;
  g_outBufferMutex = CreateMutex(NULL, FALSE, NULL);
  hResponseReceived = CreateEvent(NULL, FALSE, FALSE, NULL);

  strcpy_s(g_szCNCIP, sizeof(g_szCNCIP), "Disconnected");

  g_hConnected = CreateEvent(NULL, TRUE, FALSE, NULL);

  g_hDisconnected          = CreateEvent(NULL, FALSE, FALSE, NULL);
  g_hBufferFull    = CreateEvent(NULL, FALSE, FALSE, NULL);
  g_hBufferEmpty   = CreateEvent(NULL, FALSE, FALSE, NULL);
  g_hAckReceived   = CreateEvent(NULL, FALSE, FALSE, NULL);
  g_hNackReceived  = CreateEvent(NULL, FALSE, FALSE, NULL);

  CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)senderThread, NULL, 0, &threadId);
  CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)receiverThread, NULL, 0, &threadId);
  CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)listenerThread, NULL, 0, &threadId);

  WaitForSingleObject(g_hConnected, 3000);
  sendCommand("RST", NULL, 0 );
  
  return 0;
}

