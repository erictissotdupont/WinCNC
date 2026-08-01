
typedef enum {
	cncStatus_Success = 0,
	cncStatus_HeaderDecodingError = -1,
	cncStatus_MessageIsTooShort = -2,
	cncStatus_CommandDecodingError = -3,
	cncStatus_PositionCRCmismatch = -4,
	cncStatus_UnknownCommand = -5,
	cncStatus_SequenceError = -6,
} tCnCCmdStatus;


unsigned char CNC_GetPositionCRC(long x, long y, long z, unsigned long t, unsigned long flags);
bool CNC_LockMachinePosition(bool bLock);
void CNC_GetDisplayPosition(t3DPoint* pPos);
int CNC_InitNetworkCom( );
void CNC_Reboot( );
tStatus CNC_Calibrate(unsigned long axis_flags);
void CNC_SendManualUpdate(int x, int y, int z);
void CNC_ForceStop( );
void CNC_Resume();

tStatus CNC_PostMovementCommand(long x, long y, long z, unsigned long d, unsigned long s);
void CNC_GetNetworkStatusString(char* szBuffer, size_t cbBuffer);
void CNC_GetStateString(char* szBuffer, size_t cbBuffer, unsigned long mask);
unsigned long CNC_GetState( );


