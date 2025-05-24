
int UDP_ParseMessage( char* msgbuf, int nbytes, char* outBuf );
int UDP_GetIdleStatus( char* outBuf );
void UDP_IdleTask( );
int UDP_Init( );

unsigned char crc8( unsigned char* pt, unsigned int nbytes, unsigned char crc );
