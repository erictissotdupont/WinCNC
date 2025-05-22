
void SetState( unsigned long flag );
void ClearState( unsigned long flag );
unsigned long GetState( );
void EventInit( );
void IRAM_ATTR SignalMotorIdleFromISR( );
void SignalMotorNotIdle( );
bool WaitForMotorIdle( unsigned long timeoutMs );
bool IsMotorIdle( );