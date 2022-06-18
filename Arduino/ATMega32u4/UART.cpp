#include "CNC.h"
#include "UART.h"
#include "Motor.h"

#include "LCD.h"

//extern HardwareSerial Serial1;
//extern Uart Serial1;

extern int g_debug[MAX_DEBUG];
extern unsigned long g_error;
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
unsigned long g_timeOfLastAck = 0;
char g_ACKchar = '0';

#define MAX_ADC           3
#define ADC_SAMPLE_AVG    16

uint32_t adc_sample[MAX_ADC][ADC_SAMPLE_AVG];
int adc_pin[MAX_ADC] = {A1,A2,A0};

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
  Serial1.begin(115200);
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

#define USE_FAST_UART
#ifdef USE_FAST_UART

void UART_Print( const char* str )
{
  Serial1.write_noWait( (uint8_t*)str, strlen( str ) + 1 );
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

#define DEC_TO_HEX( pt, n, shift ) { *pt = HEXTOCHAR[(n >> shift) & 0x0F]; pt++; flag = 1; }

void UART_PrintHex( const char prefix, const unsigned long val )
{
  const char HEXTOCHAR[16] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' };
  char str[12];
  char *pt = str;
  unsigned long n = val;
  bool flag = 0;
  *pt++ = prefix;
  if( flag || n > (0x10000000L - 1 )) DEC_TO_HEX( pt, n, 28 );
  if( flag || n > (0x1000000L - 1 )) DEC_TO_HEX( pt, n, 24 );
  if( flag || n > (0x100000L - 1 )) DEC_TO_HEX( pt, n, 20 );
  if( flag || n > (0x10000L - 1 )) DEC_TO_HEX( pt, n, 16 );
  if( flag || n > (0x1000L - 1 )) DEC_TO_HEX( pt, n, 12 );
  if( flag || n > (0x100L - 1 )) DEC_TO_HEX( pt, n, 8 );
  if( flag || n > (0x10L - 1 )) DEC_TO_HEX( pt, n, 4 );
  *pt++ = HEXTOCHAR[ n & 0xF ];
  Serial1.write_noWait( (uint8_t*)str, pt - str );
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

#define DEC_TO_HEX( n, shift ) { Serial1.write( HEXTOCHAR[(n >> shift) & 0x0F]); flag = 1; }

void UART_PrintHex( const char prefix, const unsigned long val )
{
  const char HEXTOCHAR[16] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' };
  unsigned long n = val;
  bool flag = 0;
  Serial1.write( prefix );
  if( flag || n > (0x10000000L - 1 )) DEC_TO_HEX( n, 28 );
  if( flag || n > (0x1000000L - 1 )) DEC_TO_HEX( n, 24 );
  if( flag || n > (0x100000L - 1 )) DEC_TO_HEX( n, 20 );
  if( flag || n > (0x10000L - 1 )) DEC_TO_HEX( n, 16 );
  if( flag || n > (0x1000L - 1 )) DEC_TO_HEX( n, 12 );
  if( flag || n > (0x100L - 1 )) DEC_TO_HEX( n, 8 );
  if( flag || n > (0x10L - 1 )) DEC_TO_HEX( n, 4 );
  Serial1.write( HEXTOCHAR[ n & 0xF ] );
}

void UART_Print( const char* str )
{
  Serial1.print( str );
}

#define UART_Write( c ) Serial1.write( c )

#endif

inline void UART_SendAck( )
{
  UART_Write( g_ACKchar );
  g_ACKchar++;
  if( g_ACKchar > '9' ) g_ACKchar = '0';
}

uint32_t ADC_GetAverage( int idx )
{
  if( idx >= MAX_ADC )
  {
    return 0;
  }
  else
  {
    uint32_t avg = 0;
    for( int n=0; n<ADC_SAMPLE_AVG; n++ ) avg += adc_sample[idx][n];
    return avg / ADC_SAMPLE_AVG;
  } 
}

void UART_SendStatus( int command )
{
  static int state = 0;
  switch( state )
  {
    case 0 : 
      if( command == 1 )
      {
        UART_Write( '#' );
        state = 1; 
      }
      else if( command == 2 )
      {
        UART_Write( '#' );
        state = 10;
      }
      else if( command == 3 )
      {
        UART_Write( '#' );
        state = 14;
      }
      break;
    case 1 : UART_Print( 'X', X.GetPos( ));state=2; break;
    case 2 : UART_Print( 'Y', Y.GetPos( ));state=3; break;
    case 3 : UART_Print( 'Z', Z.GetPos( ));state=4; break;
    case 4 : UART_PrintHex( 'S', g_error );state=5;break;
    case 5 : UART_Print( 'Q', fifoCount( )); UART_Write( '\0' );state = 0;break;
    
    case 10 : UART_Print( 'a', g_debug[0] );state=11;break;
    case 11 : UART_Print( 'b', g_debug[1] );state=12;break;
    case 12 : UART_Print( 'c', g_debug[2] );state=13;break;
    case 13 : UART_Print( 'd', g_debug[3] );UART_Write( '\0' );state=0;break;

    case 14 : UART_Print( 'A', ADC_GetAverage( 0 ));state=15;break;
    case 15 : UART_Print( 'B', ADC_GetAverage( 1 ));state=16;break;
    case 16 : UART_Print( 'C', ADC_GetAverage( 2 ));UART_Write( '\0' );state=0;break;
    
    default :
      state = 0;
  }
}

bool ADC_Task( )
{
  bool bRet = false;
  static int adc_index = -1;
  static int sample_index = 0;
  uint32_t adc;

  if( adc_index == -1 )
  {
    startAnalogRead( A0 );
    adc_index = 0;
  }
  else
  {
    if( fastAnalogRead( adc_pin[adc_index], &adc ))
    {
      adc_sample[adc_index][sample_index] = adc;
      adc_index++;
      if( adc_index >= MAX_ADC )
      {
        adc_index = 0;
        sample_index++;
        if( sample_index >= ADC_SAMPLE_AVG )
        {
          sample_index = 0;       
        }
      }
      bRet = true;
    }
  }
  return bRet;
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
  unsigned long now = micros( );
  static int rxCount = 0;
  static char rxBuffer[16];
#ifdef DISPLAY_TASK_TIME
  unsigned long timeItTook = now;
#endif 

  // Flush out status
  UART_SendStatus(0);

  // Write any pending characters to the UART and return 'true' if RX buffer not empty
  if( Serial1.write_noWait( NULL, 0 ) == 0 )
  {
    // Nothing to read from UART, let's refresh the ADC
    ADC_Task( );    
  }
  else
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
      else if( c == '\0' )
      {
        if( g_error )
        {
          // Report the Error state to the host.
          // Ignore the command.
          UART_Write( 'E' );
          UART_Write( '1' );          
          g_timeOfLastAck = now;
        }
        else
        {
          // If we got a CRC, check it
          if( bInCRC && ( calculatedCRC != receivedCRC ))
          {
            /*
            char str[32];
            sprintf( str, "#%d,%d,%d,%d", 
              g_fifo[g_fifoIn].x,
              g_fifo[g_fifoIn].y,
              g_fifo[g_fifoIn].z,
              g_fifo[g_fifoIn].d );

            UART_Print( str );
            */
            // Report the CRC mismatch. Ignore the command.
            UART_Write( 'E' );
            UART_Write( '2' );            
            g_timeOfLastAck = now;
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
            if( fifoCount( ) < MAX_FIFO_MOVE )
            {
              UART_SendAck( );
              g_timeOfLastAck = now;
            }
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

        rxCount = 0; 
        state = COMM_IDLE;
      }
      else
      {
        // This is a bit strict but if we receive any other symbol than
        // what should be in a frame then enter error state.
        g_error |= ERROR_SYNTAX;
        g_debug[1] = c;
        rxCount = 0; 
        state = COMM_IDLE;
        UART_Write( 'E' );
        UART_Write( '6' );
      }     
    }
    else if( state == COMM_IDLE )
    {
      // START OF FRAME
      // -------------- 
      if( c == '@' )
      {
        state = COMM_IN_FRAME;
      }
      else
      {
        if( rxCount >= sizeof(rxBuffer) - 1 )
        {
          UART_Write( 'E' );
          UART_Write( '3' );
          g_timeOfLastAck = now;
          rxCount = 0;
        }
        else
        {
          rxBuffer[rxCount] = c;
          rxCount++;
          if( c == '\0' )
          {
            if( strcmp( rxBuffer, "RESET" ) == 0 )
            {
              UART_Print( "#Hello" );
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
              g_timeOfLastAck = now;
              g_ACKchar = '0';
            }
            else if( strcmp( rxBuffer, "ORIGIN" ) == 0 )
            {
              Motor_Init( );
              UART_SendStatus(1);      
            }
            else if( strcmp( rxBuffer, "POSITION" ) == 0 )
            {
              UART_SendStatus(1);
            }
            else if( strcmp( rxBuffer, "ANALOG" ) == 0 )
            {
              UART_SendStatus(3);
            }
            else if( strcmp( rxBuffer, "CAL_Z" ) == 0 )
            {
              g_fifo[g_fifoIn].x = 0;
              g_fifo[g_fifoIn].y = 0;
              g_fifo[g_fifoIn].z = 0;
              g_fifo[g_fifoIn].d = -1;
              g_fifo[g_fifoIn].s = 0;

              // Move the FIFO input index to the next slot.
              if( ++g_fifoIn > MAX_FIFO_MOVE ) g_fifoIn = 0;

              UART_SendAck( );
            }
            /*
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
            */
            else // Unknown command
            {
              UART_Write( 'E' );
              UART_Write( '4' );
              g_timeOfLastAck = now;
            }
            rxCount = 0;
          }
        }
      }
    }
    else
    {
      UART_Write( 'E' );
      UART_Write( '5' );
      state = COMM_IDLE;
    }
    // The UART buffer was not empty
    ret = true;
  }

  if( state == COMM_IDLE && fifoCount( ) == MAX_FIFO_MOVE )
  {
    if( g_timeOfLastAck + 500000L < now )
    {
      UART_Write( 'W' );
      g_timeOfLastAck = now;
    }
  }

#ifdef DISPLAY_TASK_TIME
  timeItTook = micros( ) - timeItTook;
  if( g_maxUARTtaskTime < timeItTook )
  {
    g_maxUARTtaskTime = timeItTook;
  }
#endif
  
  return ret;
}

extern tManualMode ManualMode;
extern unsigned long g_MoveStart;

void Motor_Task( )
{
  static char tmp[10];
  
  // While there are commands in the FIFO
  while( g_fifoIn != g_fifoOut )
  {
    // As soon as commands are received via UART, disable manual mode
    ManualMode = Manual_Disabled;
    
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
    if( fifoCount( ) == MAX_FIFO_MOVE )
    {
      UART_SendAck( );
      g_timeOfLastAck = micros( );
    }
    
    // Move the FIFO out index to the next slot
    if( ++g_fifoOut > MAX_FIFO_MOVE ) g_fifoOut = 0;
  }

  X.IsAtTheEnd( );
  Y.IsAtTheEnd( );
  Z.IsAtTheEnd( );

  // Fifo is now empty, reset the timestamp for the start
  // of the next command so that the Motor_Move( ) function
  // will use the current time instead of previous command
  // plus its duration.
  g_MoveStart = 0;

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
