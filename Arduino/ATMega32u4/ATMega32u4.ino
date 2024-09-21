#include "CNC.h"
#include "motor.h"
#include "UART.h"

// Setup (New IDE)
// ---------------
// File > Preferences > Aditional Board Manager URL :
// https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
// Go to Board manager
// Install : Arduino SAMD Boards (32bit ARM Cortex-M0+)
// Install : Adafruit SAMD Boards
// From .\Library, 
// Replace wired_xxx.c/h > .AppData\Local\Arduino15\packages\adafruit\hardware\samd\1.7.16\cores\arduino
// Replace UART.cpp/h SERCOM.cpp/h > .AppData\Local\Arduino15\packages\arduino\hardware\samd\1.8.14\cores\arduino
// 
// Go to Tools
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
