
typedef enum
{
	CNC_CONNECTED = 0,
	CNC_RESPONSE,
	CNC_ACKNOWLEDGE,
	CND_MAX_EVENT
} CNC_SOCKET_EVENT;


typedef enum {
	cncStatus_Success = 0,
	cncStatus_HeaderDecodingError = -1,
	cncStatus_MessageIsTooShort = -2,
	cncStatus_CommandDecodingError = -3,
	cncStatus_PositionCRCmismatch = -4,
	cncStatus_UnknownCommand = -5,
	cncStatus_SequenceError = -6,
} tCnCCmdStatus;

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

