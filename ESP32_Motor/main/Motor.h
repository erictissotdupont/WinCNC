

void Motor_PrepareNextCommand( cmd_t *pCmd, uint64_t now );
void Motor_PrepareManualMove( );
void Motor_MoveIfIdle( );
void Motor_GetPosition( long *pX, long *pY, long *pZ );
void Motor_ManualMove( int dX, int dY, int dZ );
void Motor_Init( );
void Motor_Enable( bool on );
