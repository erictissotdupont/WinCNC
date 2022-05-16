
#include <time.h>

#define MSGBUFSIZE     256
#define IPSTRSIZE      17
#define MACSTRSIZE     19
#define MAX_OTHERS     16

#define MS_TO_US(m)    ((m)*1000)

int Wifi_Init( );
char* getMyIP( );
char* getMyMAC( );

int isIPConnected( );