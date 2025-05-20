
void SetState( unsigned long flag );
void ClearState( unsigned long flag );
unsigned long GetState( );
void EventInit( );
void SignalIPConnected( );
void IRAM_ATTR SignalMotorIdleFromISR( );
void SignalMotorNotIdle( );
void WaitForMotorIdle( );
bool IsMotorIdle( );