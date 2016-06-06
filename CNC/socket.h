

typedef enum
{
	CNC_CONNECTED = 1,
	CNC_RESPONSE,
	CNC_ACKNOWLEDGE,
} CNC_SOCKET_EVENT;

int isCncConnected( );
long getCncErrorCount( );

void registerSocketCallback(CNC_SOCKET_EVENT event, void(*pCallback)(PVOID));
int initSocketCom( );
tStatus sendCommand( char* cmd );
tStatus waitForStatus( unsigned long timeout );
void getStatusString(char* szBuffer, unsigned int cbBuffer);

