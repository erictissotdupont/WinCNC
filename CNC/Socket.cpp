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
#define MSGBUFSIZE        300
#define MAXINPIPE         (3*MSGBUFSIZE/10)
#define TIMEPIPESIZE	  MAXINPIPE + 10

#define IPSTRSIZE         80

void(*g_pNotifCallback)(CNC_SOCKET_EVENT, PVOID) = NULL;
char cncIP[IPSTRSIZE];
int bConnected = 0;
int bRun = 1;

long ackCount = 0;
long cmdCount = 0;
long errCount = 0;
long repeatCount = 0;

int outSize= 0;
char outBuffer[MSGBUFSIZE];

int timePipeInIdx, timePipeOutIdx;
unsigned long timePipe[ TIMEPIPESIZE ];
unsigned long totalInPipe;

HANDLE mutexBuffer = NULL;

unsigned long getDurationOfCommandsInPipe( )
{
  /*
  unsigned long t = 0;
  int i = timePipeOutIdx;
  while( i != timePipeInIdx )
  {
    t += timePipe[i++];
    if( i>=TIMEPIPESIZE ) i=0;
  }
  return t;
  */
  return totalInPipe;
}

tStatus sendCommand( char* cmd, unsigned long d )
{
  int timeout;
  int cl = strlen( cmd );
  int bl;

  if (!bConnected ) return retCncNotConnected;
  if( errCount ) return retCncError;
  if( cmd == NULL ) return retInvalidParam;
  if( cl > MSGBUFSIZE ) return retInvalidParam;

  // Don't push more than 5 seconds worth of commands in the pipe
  timeout = (60 * 1000) / MSGPERIOD;
  while(( getDurationOfCommandsInPipe( ) > 5000 ) && timeout-- ) Sleep( MSGPERIOD );

  // If the command can't fit in the current buffer
  if(( strlen( outBuffer ) + cl + 1 ) >= MSGBUFSIZE )
  {
    timeout = (60 * 1000) / MSGPERIOD;
    // Wait for the buffer to be empty (1 min timeout)
    // printf( "outBuffer is full...\n" );
    while( outBuffer[0] != 0 && timeout-- > 0 ) Sleep( MSGPERIOD );
    if( timeout <= 0 )
    {
      printf( "Timeout on TX\n" );
      return retCncCommError;
    }
  }

  WaitForSingleObject(mutexBuffer, INFINITE);
  //pthread_mutex_lock(&mutexBuffer);

  timePipe[ timePipeInIdx++ ] = d;
  totalInPipe += d;
  if( timePipeInIdx >= TIMEPIPESIZE ) timePipeInIdx = 0;

  bl = strlen( outBuffer );
  // Add the new command to the buffer
  strcpy_s( outBuffer + bl, sizeof(outBuffer) - bl, cmd );
  cmdCount++;
  outSize += cl;
  
  ReleaseMutex(mutexBuffer);
  // pthread_mutex_unlock(&mutexBuffer);
  
  //printf( "Send:%s", cmd );

  return retSuccess;
}


void* receiverThread( void* arg )
{
  int nbytes;
  char msgbuf[MSGBUFSIZE];
  unsigned long tmp;
  SOCKET cnc = (SOCKET)arg;

  bConnected = true;

  if (g_pNotifCallback) g_pNotifCallback(CNC_CONNECTED, cncIP);

  while( bRun )
  {
	if ((nbytes = recv(cnc, msgbuf, sizeof(msgbuf),0)) < 0) 
	{
      perror("read");
	  bRun = false;
    }
    tmp = 0;
    while( nbytes>0 )
    {
      switch( msgbuf[--nbytes] )
      {
      case 'O' : ackCount++;
         tmp += timePipe[ timePipeOutIdx++ ];  
         if( timePipeOutIdx >= TIMEPIPESIZE ) timePipeOutIdx = 0;
        break;
      case 'E' : errCount++; break;
      case 'T' : errCount++; break;
      }
    }
	WaitForSingleObject(mutexBuffer,INFINITE);
    totalInPipe -= tmp;
	ReleaseMutex(mutexBuffer);
  }

  bConnected = false;

  return NULL;
}

wchar_t sockErrStr[260];

DWORD senderThread(PVOID pParam)
{
	int iResult, cnt;
	struct sockaddr_in Addr;
	struct sockaddr_in CncAddr;
	int CncAddrSize = sizeof(CncAddr);
	char msg[80];
	WSADATA wsaData;
	int idleCount = 0;

	if (WSAStartup(0x0202, &wsaData) != NO_ERROR) {
		iResult = WSAGetLastError();
		wsprintf(sockErrStr, L"Error at WSAStartup(). Code:%d .", iResult, iResult);
		return 1;
	}

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
		return iResult;
	}

	if ((cnt = recvfrom(s, msg, sizeof(msg), 0, (SOCKADDR*)&CncAddr, &CncAddrSize)) <= 0) {
		iResult = WSAGetLastError();
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

	strcpy_s(msg, sizeof(msg), "RST");

	if (send(cnc, msg, strlen(msg), 0) <= 0) {
		iResult = WSAGetLastError();
		return 1;
	}

	if ((cnt = recv(cnc, msg, sizeof(msg), 0)) <= 0) {
		iResult = WSAGetLastError();
		return 1;
	}

	msg[cnt] = 0;
	if (strcmp(msg, "HLO") != 0) {
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
			if (cmdCount - MAXINPIPE < ackCount)
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

					outSize = 0;
					outBuffer[0] = 0;
				}
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
  totalInPipe = 0;
  memset( timePipe, 0, sizeof( timePipe ));

  outSize = 0;
  memset( outBuffer, 0, sizeof( outBuffer ));

  mutexBuffer = CreateMutex(NULL, FALSE, NULL);

  g_pNotifCallback = callback;

  CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)senderThread, NULL, 0, &threadId);

  return 0;
}

