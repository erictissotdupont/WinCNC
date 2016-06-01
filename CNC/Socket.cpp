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
#define MAX_CMD_IN_PIPE	  3000 // 3 seconds
#define TIMEPIPESIZE	  (MAX_CMD_IN_PIPE/8)

#define IPSTRSIZE         80
#define MAX_RESPONSE	  80

void(*g_pNotifCallback)(CNC_SOCKET_EVENT, PVOID) = NULL;
char cncIP[IPSTRSIZE];
char response[MAX_RESPONSE];
int rspCnt = 0;
HANDLE hResponseReceived = NULL;
int bConnected = 0;
int bRun = 1;

long ackCount = 0;
long cmdCount = 0;
long errCount = 0;
long repeatCount = 0;

int outSize= 0;
char outBuffer[MSGBUFSIZE];

typedef struct
{
	long x;
	long y;
	long z;
	unsigned long duration;
} tCommandInPipe;

int timePipeInIdx, timePipeOutIdx;
tCommandInPipe inPipe[ TIMEPIPESIZE ];
unsigned long timeInPipe;
long xInPipe;
long yInPipe;
long zInPipe;

HANDLE mutexBuffer = NULL;
//HANDLE hDebug;

int getCountOfCommandsInPipe()
{
	int c = timePipeInIdx - timePipeOutIdx;
	if (c < 0) c = c + TIMEPIPESIZE;
	return c;
}

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
		timeInPipe / 1000.0f,
		getCountOfCommandsInPipe( ),
		getAckPendingCount( ),
		cmdCount,
		ackCount,
		errCount );
}

unsigned long getDurationOfCommandsInPipe( )
{
  return timeInPipe;
}

void getDistanceInPipe(long* x, long* y, long* z)
{
	*x = xInPipe;
	*y = yInPipe;
	*z = zInPipe;
}

bool isPipeAvailable(int cmdLen)
{
	if( getDurationOfCommandsInPipe() > MAX_CMD_IN_PIPE) return false;
	if ((strlen(outBuffer) + cmdLen + 1) >= MSGBUFSIZE) return false;
	return true;
}

tStatus sendCommand( char* cmd, long x, long y, long z, unsigned long d )
{
  int timeout;
  int cl = strlen( cmd );
  int bl;

  if (!bConnected ) return retCncNotConnected;
  if( cmd == NULL ) return retInvalidParam;
  if( cl > MSGBUFSIZE ) return retInvalidParam;
  if (errCount && *cmd == '@') return retCncError;

  // Don't push more than 5 seconds worth of commands in the pipe
  // Make sure the # of commands tracked doesn't overflow the buffer
  timeout = 5000 / MSGPERIOD;
  while(((getDurationOfCommandsInPipe() > MAX_CMD_IN_PIPE) ||
	     (getCountOfCommandsInPipe() >= TIMEPIPESIZE )) && timeout-- )
  {
	  Sleep(MSGPERIOD);
  }  
  if (timeout <= 0)
  {
	  printf("Timeout on TX\n");
	  return retBufferBusyTimeout;
  }

  // If the command can't fit in the current buffer
  if(( strlen( outBuffer ) + cl + 1 ) >= MSGBUFSIZE )
  {
    timeout = ( getDurationOfCommandsInPipe( ) + 1000 ) / MSGPERIOD;
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

  tCommandInPipe *pt = &inPipe[timePipeInIdx++];
  if (timePipeInIdx >= TIMEPIPESIZE) timePipeInIdx = 0;

  pt->duration = d;
  pt->x = x;
  pt->y = y;
  pt->z = z;

  timeInPipe += d;
  xInPipe += x;
  yInPipe += y;
  zInPipe += z;

  bl = strlen( outBuffer );

  // Add the new command to the buffer
  strcpy_s( outBuffer + bl, sizeof(outBuffer) - bl, cmd );
  cmdCount++;
  outSize += cl;
  
  ReleaseMutex(mutexBuffer);

  return retSuccess;
}

void RemoveFromPipe()
{
	tCommandInPipe *pt = &inPipe[timePipeOutIdx++];
	if (timePipeOutIdx >= TIMEPIPESIZE) timePipeOutIdx = 0;
	timeInPipe -= pt->duration;
	xInPipe -= pt->x;
	yInPipe -= pt->y;
	zInPipe -= pt->z;
}

void* receiverThread( void* arg )
{
	int nbytes;
	char msgbuf[MSGBUFSIZE];
	SOCKET cnc = (SOCKET)arg;

	bConnected = true;
	rspCnt = 0;
	if (g_pNotifCallback) g_pNotifCallback(CNC_CONNECTED, cncIP);

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
				RemoveFromPipe();
				rspCnt = 0;
				break;
			case 'E' : 
				errCount++;
				RemoveFromPipe();
				rspCnt = 0;
				break;
			default:
				if (rspCnt < MAX_RESPONSE)
				{
					response[rspCnt++] = msgbuf[i];
					if (msgbuf[i] == '\n')
					{
						ackCount++;
						RemoveFromPipe();
						response[rspCnt++] = 0;
						if (g_pNotifCallback) g_pNotifCallback(CNC_RESPONSE, response );
						SetEvent(hResponseReceived);
						rspCnt = 0;
					}
				}
			}
		}
		ReleaseMutex(mutexBuffer);
	}
	bConnected = false;
	return NULL;
}

tStatus waitForStatus( )
{
	tStatus ret = retCncError;
	char* pt;

	// Allow for one more second than the max duration of commands pending for
	// the answer to come back.
	switch (WaitForSingleObject( hResponseReceived, MAX_CMD_IN_PIPE + 1000 ))
	{
	case WAIT_OBJECT_0 :
		pt = strchr( response, 'S');
		if (pt)
		{
			int errorLevel;
			if (sscanf_s(pt + 1, "%d", &errorLevel) == 1)
			{
				if (errorLevel == 0) ret = retSuccess;
			}
		}
		break;
	case WAIT_TIMEOUT :
		ret = retCncStatusTimeout;
		break;
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
	if (g_pNotifCallback) g_pNotifCallback(CNC_DISCONNECTED, 0);

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
				/*
				else
				{
					DWORD dwWritten;
					WriteFile(hDebug, outBuffer, outSize, &dwWritten, NULL);
					WriteFile(hDebug, "-------\n", 8, &dwWritten, NULL);
				}
				*/
				outSize = 0;
				outBuffer[0] = 0;
			}

			idleCount = 0;
		}
		else
		{
			int inPipe;
			// Each time we start to be idle, check the duration of the commands
			// currently pending acknowledge and set the timeout when it's been
			// too long for receiving an acknowledge. Give a 5sec grace period.
			if (idleCount == 0)
			{
				inPipe = ((getDurationOfCommandsInPipe() + 50)) / MSGPERIOD;
				// printf( "%.2f sec of commands. Will wait for %ld loops.\n", getDurationOfCommandsInPipe( ) / 1000.0, inPipe );
			}

			// If we've been idle for one second longer than the amount of
			// commands still pending to be acknowledged, check that the count
			// of sent matches the # of ACKs
			if (idleCount++ > inPipe)
			{
				if (cmdCount != ackCount)
				{
					printf("Commands send:%ld. ACKs:%ld. ETA:%.1f sec (%ld).\n",
					cmdCount,
					ackCount,
					getDurationOfCommandsInPipe() / 1000.0,
					inPipe);
				}
			}
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

int initSocketCom(void(*callback)(CNC_SOCKET_EVENT, PVOID))
{
  DWORD threadId;

  timePipeInIdx = 0;
  timePipeOutIdx = 0;
  timeInPipe = 0;
  xInPipe = 0;
  yInPipe = 0;
  zInPipe = 0;
  memset( inPipe, 0, sizeof( inPipe ));

  outSize = 0;
  memset( outBuffer, 0, sizeof( outBuffer ));

  //hDebug = CreateFile( L"C:\\Windows\\Temp\\Cncdebug.txt", GENERIC_WRITE, FILE_SHARE_READ, NULL, CREATE_ALWAYS, 0, NULL);

  mutexBuffer = CreateMutex(NULL, FALSE, NULL);
  hResponseReceived = CreateEvent(NULL, FALSE, FALSE, NULL);

  g_pNotifCallback = callback;

  strcpy_s(cncIP, sizeof(cncIP), "Disconnected");
  if (g_pNotifCallback) g_pNotifCallback(CNC_DISCONNECTED, 0);

  CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)senderThread, NULL, 0, &threadId);

  return 0;
}

