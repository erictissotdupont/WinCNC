
#include <time.h>

#define MSGBUFSIZE     256
#define IPSTRSIZE      17
#define MACSTRSIZE     19
#define MAX_OTHERS     16

#define MS_TO_US(m)    ((m)*1000)

typedef void(*stateChangeCallback)(int);

void socketSetStateChangeCallback( stateChangeCallback );
int initSocketCom( );
void signalNewState( unsigned long newState );
void signalState( int retries );
char* getMyIP( );
char* getMyMAC( );
char* getNetworkHealth( );
void resetNetworkHealth( );
char* getTimeStatus( int state );
char* getMyTimeStatus( );

typedef struct _socket_diag {
  unsigned int txCnt;
  int txErrCnt;
  
  unsigned int frmCnt;
  unsigned int errCnt;
  
} socket_diag_t;
