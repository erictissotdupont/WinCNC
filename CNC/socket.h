
typedef enum
{
	CNC_CONNECTED = 0,
	CNC_RESPONSE,
	CNC_ACKNOWLEDGE,
	CND_MAX_EVENT
} CNC_SOCKET_EVENT;

int isCncConnected( );
int getLastErrorStatus( );
long getCncErrorCount( );

unsigned char GetPosCRC(long x, long y, long z);
void registerSocketCallback(CNC_SOCKET_EVENT event, void(*pCallback)(PVOID));
int initSocketCom( );
tStatus postCommand( char* cmd );
tStatus sendCommand( char* cmd, char* rsp, size_t cbRsp );
tStatus waitForStatus( unsigned long timeout );
void getSocketStatusString(char* szBuffer, unsigned int cbBuffer);

