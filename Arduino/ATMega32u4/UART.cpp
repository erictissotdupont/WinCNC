#include "CNC.h"
#include "UART.h"
#include "Motor.h"

#include "LCD.h"

//extern HardwareSerial Serial1;
//extern Uart Serial1;

extern int g_debug[MAX_DEBUG];
extern unsigned int g_error;
// To know the current position
extern Motor X;
extern Motor Y;
extern DualMotor Z;

// FIFO to store the command received. This buffer ensure that when
// the AR9331 doesn't fill the buffer for a few milliseconds the
// ATMEGA doesn't run of things to do. Happens sometimes...
tMove g_fifo[MAX_FIFO_MOVE+1];
int g_fifoIn;
int g_fifoOut;
int g_CRCerror = 0;
unsigned long g_maxUARTtaskTime = 0;

static const uint8_t crc8_table[256] = {
    0x00, 0xF7, 0xB9, 0x4E, 0x25, 0xD2, 0x9C, 0x6B,
    0x4A, 0xBD, 0xF3, 0x04, 0x6F, 0x98, 0xD6, 0x21,
    0x94, 0x63, 0x2D, 0xDA, 0xB1, 0x46, 0x08, 0xFF,
    0xDE, 0x29, 0x67, 0x90, 0xFB, 0x0C, 0x42, 0xB5,
    0x7F, 0x88, 0xC6, 0x31, 0x5A, 0xAD, 0xE3, 0x14,
    0x35, 0xC2, 0x8C, 0x7B, 0x10, 0xE7, 0xA9, 0x5E,
    0xEB, 0x1C, 0x52, 0xA5, 0xCE, 0x39, 0x77, 0x80,
    0xA1, 0x56, 0x18, 0xEF, 0x84, 0x73, 0x3D, 0xCA,
    0xFE, 0x09, 0x47, 0xB0, 0xDB, 0x2C, 0x62, 0x95,
    0xB4, 0x43, 0x0D, 0xFA, 0x91, 0x66, 0x28, 0xDF,
    0x6A, 0x9D, 0xD3, 0x24, 0x4F, 0xB8, 0xF6, 0x01,
    0x20, 0xD7, 0x99, 0x6E, 0x05, 0xF2, 0xBC, 0x4B,
    0x81, 0x76, 0x38, 0xCF, 0xA4, 0x53, 0x1D, 0xEA,
    0xCB, 0x3C, 0x72, 0x85, 0xEE, 0x19, 0x57, 0xA0,
    0x15, 0xE2, 0xAC, 0x5B, 0x30, 0xC7, 0x89, 0x7E,
    0x5F, 0xA8, 0xE6, 0x11, 0x7A, 0x8D, 0xC3, 0x34,
    0xAB, 0x5C, 0x12, 0xE5, 0x8E, 0x79, 0x37, 0xC0,
    0xE1, 0x16, 0x58, 0xAF, 0xC4, 0x33, 0x7D, 0x8A,
    0x3F, 0xC8, 0x86, 0x71, 0x1A, 0xED, 0xA3, 0x54,
    0x75, 0x82, 0xCC, 0x3B, 0x50, 0xA7, 0xE9, 0x1E,
    0xD4, 0x23, 0x6D, 0x9A, 0xF1, 0x06, 0x48, 0xBF,
    0x9E, 0x69, 0x27, 0xD0, 0xBB, 0x4C, 0x02, 0xF5,
    0x40, 0xB7, 0xF9, 0x0E, 0x65, 0x92, 0xDC, 0x2B,
    0x0A, 0xFD, 0xB3, 0x44, 0x2F, 0xD8, 0x96, 0x61,
    0x55, 0xA2, 0xEC, 0x1B, 0x70, 0x87, 0xC9, 0x3E,
    0x1F, 0xE8, 0xA6, 0x51, 0x3A, 0xCD, 0x83, 0x74,
    0xC1, 0x36, 0x78, 0x8F, 0xE4, 0x13, 0x5D, 0xAA,
    0x8B, 0x7C, 0x32, 0xC5, 0xAE, 0x59, 0x17, 0xE0,
    0x2A, 0xDD, 0x93, 0x64, 0x0F, 0xF8, 0xB6, 0x41,
    0x60, 0x97, 0xD9, 0x2E, 0x45, 0xB2, 0xFC, 0x0B,
    0xBE, 0x49, 0x07, 0xF0, 0x9B, 0x6C, 0x22, 0xD5,
    0xF4, 0x03, 0x4D, 0xBA, 0xD1, 0x26, 0x68, 0x9F
};

inline uint8_t crc8( uint8_t* pdata, unsigned int nbytes, uint8_t crc )
{
    while (nbytes-- > 0)
    {
        crc = crc8_table[(crc ^ *pdata++) & 0xff];
    }
    return crc;
}

void UART_Init( )
{
  Serial1.begin(460800);
  g_fifoIn = 0;
  g_fifoOut = 0;

  pinMode( 13, OUTPUT );
  digitalWrite( 13, 0 );
}

inline unsigned int fifoCount( )
{
  if( g_fifoIn >= g_fifoOut )
    return g_fifoIn - g_fifoOut;
  else
    return g_fifoIn - g_fifoOut + MAX_FIFO_MOVE + 1;
}

#if 1

void UART_Print( const char* str )
{
  Serial1.write_noWait( (uint8_t*)str, strlen( str ));
}

#define DEC_TO_STR( pt, n, dec ) { *pt = '0' + (n / dec); pt++; n = n % dec; flag = 1; }

void UART_Print( const char prefix, const long val )
{
  long n = val;
  char str[16];
  char *pt = str;
  bool flag = 0;
  *pt++ = prefix;
  if( n < 0 ) { *pt++ = '-'; n = -n; }
  if( flag || n > (100000000L - 1 )) DEC_TO_STR( pt, n, 100000000L );
  if( flag || n > (10000000L - 1 )) DEC_TO_STR( pt, n, 10000000L );
  if( flag || n > (1000000L - 1 )) DEC_TO_STR( pt, n, 1000000L );
  if( flag || n > (100000L - 1 )) DEC_TO_STR( pt, n, 100000L );
  if( flag || n > (10000L - 1 )) DEC_TO_STR( pt, n, 10000L );
  if( flag || n > (1000L - 1 )) DEC_TO_STR( pt, n, 1000L );
  if( flag || n > (100L - 1 )) DEC_TO_STR( pt, n, 100L );
  if( flag || n > (10L - 1 )) DEC_TO_STR( pt, n, 10L );
  *pt++ = '0' + n;
  Serial1.write_noWait( (uint8_t*)str, pt - str );
}

void UART_Write( char c )
{
  Serial1.write_noWait( (uint8_t*)&c, 1 );
}

#else

#define DEC_TO_STR( n, dec ) { Serial1.write( '0' + (n / dec)); n = n % dec; flag = 1; }

void UART_Print( const char prefix, const long val )
{
  long n = val;
  bool flag = 0;
  Serial1.write( prefix );
  if( n < 0 ) { Serial1.write('-'); n = -n; }
  if( flag || n > (100000000L - 1 )) DEC_TO_STR( n, 100000000L );
  if( flag || n > (10000000L - 1 )) DEC_TO_STR( n, 10000000L );
  if( flag || n > (1000000L - 1 )) DEC_TO_STR( n, 1000000L );
  if( flag || n > (100000L - 1 )) DEC_TO_STR( n, 100000L );
  if( flag || n > (10000L - 1 )) DEC_TO_STR( n, 10000L );
  if( flag || n > (1000L - 1 )) DEC_TO_STR( n, 1000L );
  if( flag || n > (100L - 1 )) DEC_TO_STR( n, 100L );
  if( flag || n > (10L - 1 )) DEC_TO_STR( n, 10L );
  Serial1.write( '0' + n);
}

void UART_Print( const char* str )
{
  Serial1.print( str );
}

#define UART_Write( c ) Serial1.write( c )

#endif

void UART_SendStatus( int command )
{
  static int state = 0;
  switch( state )
  {
    case 0 : 
      if( command == 1 ) state = 1; 
      if( command == 2 ) state = 5;
      break;
    case 1 : UART_Print( 'X', X.GetPos( ));state=2; break;
    case 2 : UART_Print( 'Y', Y.GetPos( ));state=3; break;
    case 3 : UART_Print( 'Z', Z.GetPos( ));state=4; break;
    case 4 : UART_Print( 'S', g_error );UART_Write( '\n' );state = 0;break;
    case 5 : UART_Print( 'a', g_debug[0] );state=6;break;
    case 6 : UART_Print( 'b', g_debug[1] );state=7;break;
    case 7 : UART_Print( 'c', g_debug[2] );state=8;break;
    case 8 : UART_Print( 'd', g_debug[3] );UART_Write( '\n' );state=0;break;
    default :
      state = 0;
  }
}

bool UART_Task( )
{
  static long sign = 0;
  static long *pt = NULL;
  static tCommState state = COMM_IDLE;
  static long receivedCRC;
  static uint8_t calculatedCRC;
  static bool bInCRC = false;
  char c;
  bool ret = false;

#ifdef DISPLAY_TASK_TIME
  unsigned long timeItTook = micros( );
#endif 

  // Flush out status
  UART_SendStatus(0);

  // Write any pending characters to the UART and return 'true' if RX buffer not empty
  if( Serial1.write_noWait( NULL, 0 ))
  {
    c = Serial1.read( );
        
    if( state == COMM_IN_FRAME )
    {
      if( bInCRC == false )
      {
        calculatedCRC = crc8_table[calculatedCRC ^ c];
      }
      
      if( c >= '0' && c <= '9' )
      {
        if( pt )
        { 
          *pt = *pt * 10 + ( c - '0' ) * sign;
        }
        else
        { 
          // We received numbers without an axis indicator first
          g_error |= ERROR_NUMBER;
        }
      }
      else if( c == 'X' ) { pt = &g_fifo[g_fifoIn].x; sign = 1; }
      else if( c == 'Y' ) { pt = &g_fifo[g_fifoIn].y; sign = 1; }
      else if( c == 'Z' ) { pt = &g_fifo[g_fifoIn].z; sign = 1; }
      else if( c == 'D' ) { pt = &g_fifo[g_fifoIn].d; sign = 1; }          // Duration in microseconds
      else if( c == 'S' ) { pt = &g_fifo[g_fifoIn].s; sign = 1; *pt = 0; } // Spindle state (1 = ON 0=OFF)
      else if( c == 'C' ) { pt = &receivedCRC; sign = 1; bInCRC = true; }          // CRC 8 value
      else if( c == '-' ) { sign = -1; }          
      else if( c == '\n' )
      {
        if( g_error )
        {
          // Report the Error state to the host. Ignore the command.     
          UART_Write( 'E' );
        }
        else
        {
          // If we got a CRC, check it
          if( bInCRC && ( calculatedCRC != receivedCRC ))
          {            
            // Report the CRC mismatch. Ignore the command.
            UART_Write( 'C' );
            // Count the error so that it can be displayed
            g_CRCerror++;
          }
          else
          {
    
            // Move the FIFO input index to the next slot.
            if( ++g_fifoIn > MAX_FIFO_MOVE ) g_fifoIn = 0;
  
            // Flow control : If the FIFO contains less than the MAX #
            // of commands send the ACK to the host to indicate that the
            // ATMEGA is ready to receive the next command. 
            if( fifoCount( ) < MAX_FIFO_MOVE ) UART_Write( 'O' );
          }
        }
        
        // Get the buffer ready for the next command.                   
        g_fifo[g_fifoIn].x = 0;
        g_fifo[g_fifoIn].y = 0;
        g_fifo[g_fifoIn].z = 0;
        g_fifo[g_fifoIn].d = 0;
        g_fifo[g_fifoIn].s = -1;
        calculatedCRC = 0xFF;
        receivedCRC = 0;
        bInCRC = false;
        pt = NULL;
        state = COMM_CONNECTED;
      }
      else
      {
        // This is a bit strict but if we receive any other symbol than
        // what should be in a frame then enter error state.
        g_error |= ERROR_SYNTAX;
        g_debug[1] = c;
      }  
    }
    else if( state == COMM_CONNECTED )
    {
      // START OF FRAME
      // -------------- 
      if( c == '@' )
      {
        state = COMM_IN_FRAME;
      }
      // ORIGIN
      // ------
      else if( c == 'O' )
      {
        Motor_Init( );
        UART_SendStatus(1);
      }
      // CLEAR
      // -----
      else if( c == 'C' )
      {
        g_error = 0;
        UART_SendStatus(1);
      }    
      // RESET
      // -----
      else if( c == 'R' )
      {
        state = COMM_RESET_1;
      }
      // POSITION
      // --------
      else if( c == 'P' )
      {
        UART_SendStatus(1);
      }
      // DEBUG
      // -----
      else if( c == 'D' )
      {
        UART_SendStatus(2);
      }
      else if ( c == '\n' )
      {
        // Ignore new lines symbols
      }
      else
      {
        // This symbol is not valid in this state
        g_error |= ERROR_COMM;
        g_debug[0] = c;
      }    
    }
    else if( state == COMM_IDLE )
    {
      if( c == 'R' ) state = COMM_RESET_1;
    }
    else if( state == COMM_RESET_1 )
    {
      if( c == 'S' ) 
        state = COMM_RESET_2;
      else
        state = COMM_IDLE;
    }
    else if( state == COMM_RESET_2 )
    {
      if( c == 'T' )
      {
        UART_Print( "HLO\n" );
        g_fifoIn = 0;
        g_fifoOut = 0;
        g_fifo[g_fifoIn].x = 0;
        g_fifo[g_fifoIn].y = 0;
        g_fifo[g_fifoIn].z = 0;
        g_fifo[g_fifoIn].d = 0;
        g_fifo[g_fifoIn].s = -1;
        calculatedCRC = 0xFF;
        receivedCRC = 0;
        bInCRC = false;
        pt = NULL;
        g_CRCerror = 0;
        state = COMM_CONNECTED;
      }
      else if( c == '\n' )
      {
        // Ignore
      }
      else
      {
        state = COMM_IDLE;
      }
    }
    // The UART buffer was not empty
    ret = true;
  }

#ifdef DISPLAY_TASK_TIME
  timeItTook = micros( ) - timeItTook;
  if( g_maxUARTtaskTime < timeItTook )
  {
    g_maxUARTtaskTime = timeItTook;
  }
#endif
  
  // The UART buffer was empty
  return ret;
}

void Motor_Task( )
{
  static char tmp[10];
  
  // While there are commands in the FIFO
  while( g_fifoIn != g_fifoOut )
  {
    // Update the state of the spindle if needed
    if( g_fifo[g_fifoOut].s >= 0 )
    {
      digitalWrite(TOOL_ON_REPLAY, g_fifo[g_fifoOut].s ? HIGH : LOW);
    }
        
    // Perform the move
    Motor_Move( 
      g_fifo[g_fifoOut].x,
      g_fifo[g_fifoOut].y,
      g_fifo[g_fifoOut].z,
      g_fifo[g_fifoOut].d );

    // If the FIFO was full, send the ACK to the AR9331 so that
    // the next command is sent.
    if( fifoCount( ) == MAX_FIFO_MOVE ) UART_Write( 'O' );
    
    // Move the FIFO out index to the next slot
    if( ++g_fifoOut > MAX_FIFO_MOVE ) g_fifoOut = 0;
  }

  // Manual motion
  Motor_Move( 0, 0, 0, 0 );

  if( g_error )
  {
    sprintf( tmp, "E:%x", g_error );
    LCD_SetStatus( tmp, 0 );
  }
  else if( g_CRCerror )
  {
    sprintf( tmp, "C:%d", g_CRCerror );
    LCD_SetStatus( tmp, 0 );
  }
 
  //LCD_SetStatus( X.IsAtTheEnd( ) ? "X" : " ", 0 );
  //LCD_SetStatus( Y.IsAtTheEnd( ) ? "Y" : " ", 1 );
  //LCD_SetStatus( Z.IsAtTheEnd( ) ? "Z" : " ", 2 );
}
