
typedef enum {
  COMM_IDLE,
  COMM_RESET_1,
  COMM_RESET_2,
  COMM_CONNECTED,
  COMM_IN_FRAME 
} tCommState;

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
  int32_t d; // Duration in microseconds. 0 means fasst move.
  int32_t s; // Spindle 1=ON 0=OFF, -1 = No change
} tMove;

inline void UART_SendStatus( );
void UART_Init( );
bool UART_Task( );
void Motor_Task( );
