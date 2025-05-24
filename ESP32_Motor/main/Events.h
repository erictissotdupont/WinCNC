
void Events_SetState( unsigned long flag );
void ClearState( unsigned long flag );
unsigned long Events_GetState( );
void Events_Init( );
void IRAM_ATTR Events_SignalMotorIdleFromISR( );
void Events_SignalMotorNotIdle( );
bool Events_WaitForMotorIdle( unsigned long timeoutMs );
bool Events_IsMotorIdle( );