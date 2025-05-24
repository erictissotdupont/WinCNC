
typedef enum
{
	CNC_MACHINE_UPDATE = 0,
	CNC_MAX_EVENT
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

unsigned char GetPosCRC(long x, long y, long z, unsigned long t, unsigned long flags);
bool LockMachinePosition(bool bLock);
void registerSocketCallback(CNC_SOCKET_EVENT event, void(*pCallback)(PVOID));
int initSocketCom( );
void ForceStop( );
tStatus postCommand( char* cmd );
void getSocketStatusString(char* szBuffer, size_t cbBuffer);
void getCNCStateString(char* szBuffer, size_t cbBuffer, unsigned long mask);
unsigned long getCNCState( );

