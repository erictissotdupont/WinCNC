
typedef enum {
  COMM_IDLE,
  COMM_RESET_1,
  COMM_RESET_2,
  COMM_CONNECTED,
  COMM_IN_FRAME 
} tCommState;

typedef struct
{
  long x;
  long y;
  long z;
  long d; // Duration in microseconds. 0 means fasst move.
  long s; // Spindle 1=ON 0=OFF, -1 = No change
} tMove;

inline void UART_SendStatus( );
void UART_Init( );
bool UART_Task( );
void Motor_Task( );
