
#include <time.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define MSGBUFSIZE     256
#define IPSTRSIZE      17
#define MACSTRSIZE     19
#define MAX_OTHERS     16

#define MS_TO_US(m)    ((m)*1000)

#define SNTP_TIMEOUT_MS             ( 1000L * 120L )
#define PEER_RESPONSE_TIMEOUT_MS	  ( 3000L )
#define SNTP_BROADCAST_PERIOD_MS    ( 15000L )

#define NACK_INTERVAL_MS            ( 100L )


typedef void(*stateChangeCallback)(int);

void SetPosition( long x, long y, long z );
unsigned char crc8( unsigned char* pt, unsigned int nbytes, unsigned char crc );
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
void sendNAK( unsigned long seq, struct sockaddr_in* target );

typedef struct _socket_diag {
  unsigned int txCnt;
  int txErrCnt;
  
  unsigned int frmCnt;
  unsigned int errCnt;
  
} socket_diag_t;
