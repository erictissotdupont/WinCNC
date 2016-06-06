/*
 * sockect.c
 *   Listener and talker socket
*/
#include "CNC.h"
#include "Ws2tcpip.h"
#include "Mstcpip.h"

#include "status.h"
#include "socket.h"

#define BROADCAST_PORT    50042
#define DATA_PORT         50043

#define MSGPERIOD         30 // 30ms
#define MSGBUFSIZE        200
#define IPSTRSIZE         80
#define MAX_RESPONSE	  80

void(*g_pOnResponse)( PVOID ) = NULL;
void(*g_pOnConnected)( PVOID ) = NULL;
void(*g_pOnAcknowledge)( PVOID ) = NULL;

char cncIP[IPSTRSIZE];
char response[MAX_RESPONSE];
int rspCnt = 0;
HANDLE hResponseReceived = NULL;
int bConnected = 0;
int bRun = 1;
int g_errorLevel = 0;

long ackCount = 0;
long cmdCount = 0;
long errCount = 0;
long repeatCount = 0;

int outSize= 0;
char outBuffer[MSGBUFSIZE];

HANDLE mutexBuffer = NULL;
HANDLE hDebug;

int getAckPendingCount()
{
	return cmdCount - ackCount;
}

int isCncConnected()
{
	return bConnected;
}

long getCncErrorCount()
{
	return errCount;
}

void getStatusString(char* szBuffer, unsigned int cbBuffer)
{
	sprintf_s(szBuffer, cbBuffer, "%s - %5.2fs (%3d,%3d) - Tx:%d - Ack:%d - Err:%d",
		cncIP,
		1.0, //timeInPipe / 1000.0f,
		1, // getCountOfCommandsInPipe( ),
		getAckPendingCount( ),
		cmdCount,
		ackCount,
		errCount );
}

tStatus sendCommand( char* cmd )
{
  int timeout;
  int cl = strlen( cmd );
  int bl;

  if (!bConnected ) return retCncNotConnected;
  if( cmd == NULL ) return retInvalidParam;
  if( cl > MSGBUFSIZE ) return retInvalidParam;
  if (errCount && *cmd == '@') return retCncError;

  if (*cmd == 'S') ResetEvent(hResponseReceived);

  // If the command can't fit in the current buffer
  if(( strlen( outBuffer ) + cl + 1 ) >= MSGBUFSIZE )
  {
    timeout = 5000 / MSGPERIOD;
    // Wait for the buffer to be empty
    // printf( "outBuffer is full...\n" );
	while (outBuffer[0] != 0 && timeout-- > 0 )
	{
		Sleep(MSGPERIOD);
	}
    if( timeout <= 0 )
    {
      printf( "Timeout on TX\n" );
      return retBufferBusyTimeout;
    }
  }

  WaitForSingleObject(mutexBuffer, INFINITE);

  bl = strlen( outBuffer );

  // Add the new command to the buffer
  strcpy_s( outBuffer + bl, sizeof(outBuffer) - bl, cmd );
  cmdCount++;
  outSize += cl;
  
  ReleaseMutex(mutexBuffer);

  return retSuccess;
}


void* receiverThread( void* arg )
{
	int nbytes;
	char msgbuf[MSGBUFSIZE];
	SOCKET cnc = (SOCKET)arg;
	bool gotStatus = false;

	bConnected = true;
	rspCnt = 0;
	if (g_pOnConnected) g_pOnConnected( cncIP );

	while( bRun )
	{
		memset(msgbuf, 0, sizeof(msgbuf));
		if ((nbytes = recv(cnc, msgbuf, sizeof(msgbuf),0)) < 0) 
		{
			perror("read");
			bRun = false;
		}

		WaitForSingleObject(mutexBuffer, INFINITE);
		for (int i = 0; i < nbytes;i++)
		{
			switch( msgbuf[i] )
			{
			case 'O' : 
				ackCount++;
				if (g_pOnAcknowledge) g_pOnAcknowledge((PVOID)TRUE);
				rspCnt = 0;
				gotStatus = false;
				break;
			case 'E' : 
				errCount++;
				if (g_pOnAcknowledge) g_pOnAcknowledge((PVOID)FALSE);
				rspCnt = 0;
				gotStatus = false;
				break;
			default:
				if (rspCnt < MAX_RESPONSE)
				{
					response[rspCnt++] = msgbuf[i];
					if (msgbuf[i] == 'S') gotStatus = true;
					if (msgbuf[i] == '\n')
					{
						ackCount++;
						response[rspCnt++] = 0;
						if (g_pOnResponse) g_pOnResponse(response);
						if (g_pOnAcknowledge) g_pOnAcknowledge((PVOID)TRUE);
						if (gotStatus)
						{
							char* pt = strchr(response, 'S');
							if (pt)
							{
								if (sscanf_s(pt + 1, "%d", &g_errorLevel) != 1) g_errorLevel = -1;
							}
							SetEvent(hResponseReceived);
						}
						rspCnt = 0;
						gotStatus = false;
					}
				}
			}
		}
		ReleaseMutex(mutexBuffer);
	}
	bConnected = false;
	return NULL;
}

tStatus waitForStatus( unsigned long timeout )
{
	tStatus ret = retUnknownErr;

	// Allow for one more second than the max duration of commands pending for
	// the answer to come back.
	switch (WaitForSingleObject( hResponseReceived, timeout ))
	{
	case WAIT_OBJECT_0 :
		if (g_errorLevel == 0) ret = retSuccess; else ret = retCncError;
		break;
	case WAIT_TIMEOUT :
		ret = retCncStatusTimeout;
		break;
	default:
		ret = retUnknownErr;
	}
	return ret;
}

wchar_t sockErrStr[MAX_PATH];

DWORD senderThread(PVOID pParam)
{
	int iResult, cnt;
	struct sockaddr_in Addr;
	struct sockaddr_in CncAddr;
	int CncAddrSize = sizeof(CncAddr);
	char msg[512];
	WSADATA wsaData;
	int idleCount = 0;

	memset(&wsaData, 0x00, sizeof(wsaData));
	if((iResult = WSAStartup(0x0202, &wsaData)) != NO_ERROR) {
		wsprintf(sockErrStr, L"Error at WSAStartup(). Code:%d(%d) .", iResult, WSAGetLastError( ));
		MessageBox(NULL, sockErrStr, L"Error", MB_OKCANCEL );
		return 1;
	}

	memset(&Addr, 0x00, sizeof(Addr));
	Addr.sin_addr.s_addr = htonl(INADDR_ANY);
	Addr.sin_family = AF_INET;
	Addr.sin_port = htons(BROADCAST_PORT);

	SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (s == INVALID_SOCKET) {
		wprintf(L"socket function failed with error: %u\n", WSAGetLastError());
		WSACleanup();
		return 1;
	}

	if ((iResult = bind(s, (SOCKADDR*)&Addr, sizeof(Addr))) != 0) {
		iResult = WSAGetLastError();
		WSACleanup();
		return iResult;
	}

	CncAddrSize = sizeof(CncAddr);
	if ((cnt = recvfrom(s, msg, sizeof(msg), 0, (SOCKADDR*)&CncAddr, &CncAddrSize)) <= 0) {
		iResult = WSAGetLastError();
		WSACleanup();
		return 1;
	}

	iResult = closesocket(s);
	if (iResult == SOCKET_ERROR) {
		iResult = WSAGetLastError();
		WSACleanup();
		return 1;
	}

	SOCKET cnc = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (cnc == INVALID_SOCKET) {
		return 1;
	}

	/* Just change the port and connect. */
	CncAddr.sin_port = htons(DATA_PORT);

	if (connect(cnc, (SOCKADDR*)&CncAddr, CncAddrSize) != 0) {
		return 1;
	}

	RtlIpv4AddressToStringA(&CncAddr.sin_addr, cncIP);
	if (g_pOnConnected) g_pOnConnected(NULL);

	strcpy_s(msg, sizeof(msg), "RST\n");

	if (send(cnc, msg, strlen(msg), 0) <= 0) {
		iResult = WSAGetLastError();
		return 1;
	}

	if ((cnt = recv(cnc, msg, sizeof(msg), 0)) <= 0) {
		iResult = WSAGetLastError();
		return 1;
	}

	msg[cnt] = 0;
	if (strcmp(msg, "HLO\n") != 0) {
		return 1;
	}

	outSize = 0;
	memset(outBuffer, 0, sizeof(outBuffer));

	printf("Connected.\n");

	DWORD threadId;
	CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)receiverThread, (PVOID)cnc, 0, &threadId);

	while (bRun) {

		WaitForSingleObject(mutexBuffer, INFINITE);

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
					DWORD dwWritten;
					WriteFile(hDebug, outBuffer, outSize, &dwWritten, NULL);
					WriteFile(hDebug, "-------\n", 8, &dwWritten, NULL);
				}
				outSize = 0;
				outBuffer[0] = 0;
			}

			idleCount = 0;
		}
		ReleaseMutex(mutexBuffer);
		// if (pthread_mutex_unlock(&mutexBuffer) < 0) { perror("pthread_mutex_unlock"); }
		Sleep(MSGPERIOD);
	}

	iResult = closesocket(cnc);
	if (iResult == SOCKET_ERROR) {
		iResult = WSAGetLastError();
		WSACleanup();
		return 1;
	}

	WSACleanup();
	return 0;

}

void registerSocketCallback(CNC_SOCKET_EVENT event, void(*pCallback)(PVOID))
{
	switch (event)
	{
	case CNC_CONNECTED :
		g_pOnConnected = pCallback;
		break;
	case CNC_ACKNOWLEDGE :
		g_pOnAcknowledge = pCallback;
		break;
	case CNC_RESPONSE :
		g_pOnResponse = pCallback;
		break;
	}
}

int initSocketCom( )
{
  DWORD threadId;

  outSize = 0;
  memset( outBuffer, 0, sizeof( outBuffer ));

  hDebug = CreateFile( L"C:\\Windows\\Temp\\Cncdebug.txt", GENERIC_WRITE, FILE_SHARE_READ, NULL, CREATE_ALWAYS, 0, NULL);

  mutexBuffer = CreateMutex(NULL, FALSE, NULL);
  hResponseReceived = CreateEvent(NULL, FALSE, FALSE, NULL);

  strcpy_s(cncIP, sizeof(cncIP), "Disconnected");
  if (g_pOnConnected) g_pOnConnected(NULL);

  CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)senderThread, NULL, 0, &threadId);

  return 0;
}

