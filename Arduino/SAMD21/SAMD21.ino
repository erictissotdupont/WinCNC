#include "CNC.h"
#include "motor.h"
#include "LCD.h"
#include "UART.h"

void setup( ) 
{
  //LCD_Init( );
  //LCD_Clear( );
  UART_Init( );
  //Motor_Init( );
}

void loop( ) 
{
  // Process UART commands (until RX buffer is empty)
  while( UART_Task( ));
  
  // Perform the commands (until no more commands in FIFO)
  //Motor_Task( );

  // Refresh the LCD outside of the move loop
  //LCD_UpdateTask( 1000 );
  
  // Scan the keyboard 
  //LCD_ButtonTask( );
}
