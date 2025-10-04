
void Events_SetDebug( int64_t value );
int64_t Events_GetDebug( );
void Events_SetState( unsigned long flag );
void Events_ClearState( unsigned long flag );
unsigned long Events_GetState( );

int Events_Init( );

void Events_SignalMotorIdleFromISR( );
void Events_SignalMotorNotIdle( );
bool Events_WaitForMotorIdle( unsigned long timeoutMs );
bool Events_IsMotorIdle( );