
uint8_t UDP_GetCRCAndUpdatePosition( cmd_t *pCmd );
int UDP_ParseMessage( char* msgbuf, int nbytes, char* outBuf );
int UDP_GetIdleStatus( char* outBuf );
void UDP_ResetPosition( );
void UDP_IdleTask( );
int UDP_Init( );

unsigned char crc8( unsigned char* pt, unsigned int nbytes, unsigned char crc );
