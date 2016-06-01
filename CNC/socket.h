

typedef enum
{
	CNC_CONNECTED = 1,
	CNC_DISCONNECTED,
	CNC_RESPONSE,
} CNC_SOCKET_EVENT;

int isCncConnected( );
long getCncErrorCount( );
void getDistanceInPipe(long* x, long* y, long* z);
int initSocketCom(void(*callback)(CNC_SOCKET_EVENT, PVOID));
tStatus sendCommand( char* cmd, long x, long y, long z, unsigned long d );
tStatus waitForStatus( );
void getStatusString(char* szBuffer, unsigned int cbBuffer);

