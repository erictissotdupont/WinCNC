#include "CNC.h"
#include "motor.h"
#include "LCD.h"
#include "UART.h"

// Board : Metro M0 Express
// Setup : https://learn.adafruit.com/adafruit-metro-m0-express-designed-for-circuitpython/using-with-arduino-ide
// Datasheet : adafruit-metro-m0-express-designed-for-circuitpython.pdf
//             SAMD21-Family-DataSheet-DS40001882D.pdf

void setup( ) 
{
  UART_Init( );
  Motor_Init( );
}

void loop( ) 
{
  // Process UART commands (until RX buffer is empty)
  while( UART_Task( ));
  
  // Perform the commands (until no more commands in FIFO)
  Motor_Task( );
}
