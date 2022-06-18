

#define X_AXIS_RES           0.00043821f
#define Y_AXIS_RES           0.00049271f
#define Z_AXIS_RES           0.00039250f



typedef struct _cmd_t
{
  long dx;
  long dy;
  long dz;
  unsigned long duration;
  unsigned long flags;
} cmd_t;


bool ResetCommand( );
bool OriginCommand( );
bool CheckMachineIsIdle( unsigned long seq, struct sockaddr_in* source );
bool GetAnalogCommand( unsigned long* A0, unsigned long* A1, unsigned long* A2 );
bool GetPositionCommand( long* pX, long* pY, long *pZ, unsigned long* pS, unsigned long *pQ );
bool MoveCommand( cmd_t* pCmd );
bool Calibrate_Z( );

#define DEBUG_UDPx
#define DEBUG_UARTx

extern unsigned long g_Status;

#define STATUS_LITLE_ENDIAN   0x80000000
#define STATUS_GOT_POSITION   0x40000000
