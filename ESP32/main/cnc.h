

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
bool CheckMachineIsIdle( unsigned long seq );
bool GetPositionCommand( long* pX, long* pY, long *pZ, unsigned long* pS, unsigned long *pQ );
bool MoveCommand( cmd_t* pCmd );

#define DEBUG_UDPx
#define DEBUG_UARTx

extern unsigned long g_Status;

#define STATUS_LITLE_ENDIAN   0x80000000
#define STATUS_GOT_POSITION   0x40000000
